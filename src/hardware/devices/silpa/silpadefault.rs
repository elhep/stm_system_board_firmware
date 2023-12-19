use micromath::F32Ext;
use crate::hardware::{ServMod, self};
use crate::hardware::devices::max1329::adc::AdcCode;
use crate::hardware::devices::{Devices, Variants};
use crate::net::telemetry;
use super::SiLPA;
use embedded_hal::digital::v2::OutputPin;
use miniconf::Miniconf;
use serde::Serialize;
use crate::hardware::ecp5::ECP5;
use crate::hardware::devices::max1329::{self, Max1329,adc, dac};


use embedded_hal::blocking::i2c::{WriteRead, Read};
pub mod AMPLIFIER_PARAMETERS{
    pub const TOTAL_GAIN : f32 = 56.0; //Gain w dB
    pub const DIVIDER_RATIO : f32 = 0.76; //Dzielnik 82 i 1k     
}
pub mod ECP5_OUTPUTS{
    pub const TOGGLE_CH1 : u8 = 0x01;
    pub const TOGGLE_CH2 : u8 = 0x02;
}
#[derive(Copy, Clone)]
pub struct TelemetryBuffer{
    adc: adc::AdcCode,
    telemetry : Telemetry
}

impl Default for TelemetryBuffer {
    fn default() -> Self {
        Self {
            adc: adc::AdcCode(0),
            telemetry: Telemetry::default()
        }
    }
}

impl TelemetryBuffer{
    pub fn finalize(self) -> Telemetry{
        self.telemetry
    }

    pub fn set_adc1_field(&mut self, val : AdcCode){
        self.telemetry.adc1 = vrms_to_dbm_converter(bits_to_f32(val.0));
    }

    pub fn set_adc2_field(&mut self, val : AdcCode){
        self.telemetry.adc2 = vrms_to_dbm_converter(bits_to_f32(val.0));
    }

    pub fn set_input_power(&mut self){
        self.telemetry.input_power1 = self.telemetry.adc1 - AMPLIFIER_PARAMETERS::TOTAL_GAIN;
        self.telemetry.input_power2 = self.telemetry.adc2 - AMPLIFIER_PARAMETERS::TOTAL_GAIN;
    }
}

#[derive(Clone, Copy, Debug, Miniconf, PartialEq)]
pub struct Settings{
    adc_gt_threshold: u16,  // max 0xFFF
    adc_lt_threshold: u16,  // max 0xFFF
    dacs_enable     : [bool; 2],
    dacs_value      : [u16; 2],  // max 0xFFF
    pub channels_locked : [bool; 2],
    pub telemetry_period: u16,
}

impl Default for Settings{
    fn default() -> Self {
        Self{
            adc_gt_threshold: 0xFFF,
            adc_lt_threshold: 0x000,
            dacs_enable     : [false, false],
            channels_locked : [false, false],
            dacs_value      : [0x000, 0x000],
            telemetry_period: 10,
        }
    }
}


#[derive(Serialize, Clone, Copy)]
pub struct Telemetry{
    adc1: f32,
    adc2: f32,
    input_power1: f32,
    input_power2: f32
}

impl Default for Telemetry{
    fn default() -> Self {
        Self {  adc1: 0.0,
                adc2: 0.0,
                input_power1 : 0.0,
                input_power2 : 0.0}
    }
}

impl Telemetry{
    pub fn set_adc1_field(&mut self, val : u16){
        self.adc1 = vrms_to_dbm_converter(bits_to_f32(val));
    }

    pub fn set_adc2_field(&mut self, val : u16){
        self.adc2 = vrms_to_dbm_converter(bits_to_f32(val));
    }

    pub fn set_input_power(&mut self){
        self.input_power1 = self.adc1 - AMPLIFIER_PARAMETERS::TOTAL_GAIN;
        self.input_power2 = self.adc2 - AMPLIFIER_PARAMETERS::TOTAL_GAIN;
    }

}

pub mod ECP5_Interrupts{
    pub const CHANNEL1_INACTIVE : u8 = 0x40;
    pub const CHANNEL2_INACTIVE : u8 = 0x80;
    pub const CHANNEL1_INPUT_BIT : u8 = 4;
    pub const CHANNEL2_INPUT_BIT : u8 = 5;
}

pub mod ECP5_INPUTS{
    pub const CHANNEL1 : u8 = 0x03;
    pub const CHANNEL2 : u8 = 0x02;
}

pub struct SilpaDefault{}
impl Variants for SilpaDefault{
    type VariantSettings = Settings;
    type VariantTelemetry = Telemetry;
    type VariantTelemetryBuffer = TelemetryBuffer;
}

impl Devices <Settings, Telemetry> for SiLPA<SilpaDefault>
{
    fn init(&mut self, ecp5: &mut ECP5) -> bool {
        // TODO IO and switch control
        // Internal OSC, Disabled CLKIO out, ADC clock Divider = 1, Acquisition clocks 4 (G=1,2) or 8 (G=4, 8)

        Max1329::set_clock_control_register(self.slot, ecp5, 0b01000001);
        // Int active low, RST1 interrupt, charge-pump On 3V,
        // CP clock divider: 64 (57 kHz with internal OSC, suggested between 39k - 78kHz)
        Max1329::set_cpvm_control_register(self.slot, ecp5, 0b01000101);
        // ADC Master Clock Cycles - 32 - For Internal OSC -> 115,2 ksps
        // Reference: disable REFADJ and internal REF ADC/DAC buffers (AJD -> REFADC, ADJ -> REFDAC),
        // apply external references directly at REFADC and REFDAC pins
        Max1329::set_adc_control_register(self.slot, ecp5, adc::AutoConversion::Clk32,
                                                           adc::PowerDownConf::Normal,
                                                           adc::RefConf::ExtBuffOff,);
        // Default Setup ADC input: Ain1, ADC gain 1, Unipolar mode (Default MUX SEL is 0) //TODO configure ADC
        // Max1329::set_adc_setup_direct()




        // Enable ADC Data Ready and GT & LT interrupts
        Max1329::set_interrupt_mask_register(self.slot, ecp5, !(max1329::ADD | max1329::GTA | max1329::LTA));

        true
    }

    fn settings_update(&mut self, ecp5: &mut ECP5, new_settings: Settings) -> () {
        // Update MAX1329 only if settings changed
        if self.settings.adc_gt_threshold != new_settings.adc_gt_threshold {
            Max1329::set_adc_gt_alarm_register(self.slot, ecp5, adc::AlarmMode::NonConsecutive,
                                                                1,
                                                                new_settings.adc_gt_threshold);
        }

        if self.settings.adc_lt_threshold != new_settings.adc_lt_threshold {
            Max1329::set_adc_lt_alarm_register(self.slot, ecp5, adc::AlarmMode::NonConsecutive,
                                                                1,
                                                                new_settings.adc_lt_threshold);
        }

        if self.settings.dacs_enable != new_settings.dacs_enable {
            Max1329::set_dac_control(self.slot, ecp5, match new_settings.dacs_enable[0] { true => dac::PowerDownConf::InToOut,
                                                                                          false => dac::PowerDownConf::PowerDown,},
                                                      match new_settings.dacs_enable[1] { true => dac::PowerDownConf::InToOut,
                                                                                          false => dac::PowerDownConf::PowerDown,},
                                                      dac::OpAmp::Disable,
                                                      dac::RefConf::Ext1_0,);
        }

        if self.settings.channels_locked != new_settings.channels_locked {
            let mut ecp5_inputs : [u8; 2] = [0x00, 0x00];
            if ecp5_inputs[0] & (1 << ECP5_INPUTS::CHANNEL1) == 0{
                self.activate_channel(ecp5, 1);
            }

            if ecp5_inputs[0] & (1 << ECP5_INPUTS::CHANNEL2) == 0{
                self.activate_channel(ecp5, 2);
            }
            // activate_channel();           
        }

        if self.settings.dacs_value[0] != new_settings.dacs_value[0] {
            Max1329::set_daca_value(self.slot, ecp5, new_settings.dacs_value[0]);
        }

        if self.settings.dacs_value[1] != new_settings.dacs_value[1] {
            Max1329::set_dacb_value(self.slot, ecp5, new_settings.dacs_value[1]);
        }

        self.settings = new_settings;
    }

    fn telemetry(&mut self, ecp5: &mut ECP5) -> (Telemetry, u16) {

        // Odczyt ADC1
        Max1329::set_adc_setup_register(1, ecp5, max1329::adc::Mux::AIN1_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
        while (Max1329::read_status_register(1, ecp5) | (1 << 20)) == 0 {
            log::info!("W8 for ADC1 in Telemetry");
        }
        
        let adc_val = Max1329::read_adc_data_register(self.slot, ecp5);
        self.telemetry.set_adc1_field(adc_val);

        Max1329::set_adc_setup_register(1, ecp5, max1329::adc::Mux::AIN2_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
        while (Max1329::read_status_register(1, ecp5) | (1 << 20)) == 0 {
            log::info!("W8 for ADC2 in Telemetry");
        } 

        let adc_val = Max1329::read_adc_data_register(self.slot, ecp5);
        self.telemetry.set_adc2_field(adc_val);
        
        self.telemetry.set_input_power();
        
        (self.telemetry.finalize(),
         self.settings.telemetry_period)
    }

    fn check_interrupt(&mut self, ecp5: &mut ECP5) {

        // ODczyt inputow z FPGA //
        let mut ecp5_inputs : [u8; 2] = [0x00, 0x00];
        ecp5.read_inputs(self.slot, &mut ecp5_inputs);

        // Opis lini LVDS:
        // - 4 - input, Kanal 1, '1' - input odciety, '0' - sygnal jest wzmacniany
        // - 5 - input, Kanal 2, '1' - input odciety, '0' - sygnal jest wzmacniany
        // - 6 - output - Kanal 1, '1' ustawienie na '1' resetuje uklad i dziala
        // - 7 - output - Kanal 2, '1' ustawienie na '1' resetuje uklad i dziala
        if (ecp5_inputs[0] & ECP5_Interrupts::CHANNEL1_INACTIVE == ECP5_Interrupts::CHANNEL1_INACTIVE) {
            self.settings.channels_locked[0] = true;
        }

        if (ecp5_inputs[0] & ECP5_Interrupts::CHANNEL2_INACTIVE == ECP5_Interrupts::CHANNEL2_INACTIVE) {
            self.settings.channels_locked[1] = true;
        }
        
        // TODO zmapowanie inputow na odpowiednie piny z Silpy
        // Sprawdzic co dany input robi //
        // WYjscia outputow z lm sa zwarte wiec mamy or i to jest sugestia ze temp zostal przekroczona podwojnie

        let status : u32 = Max1329::read_status_register(self.slot, ecp5);

        if (status & max1329::GTA) != 0 {
            self.adc_gt_alarm(ecp5);
        }

        // if (status & max1329::LTA) != 0 {
        //     self.adc_lt_alarm(ecp5);
        // }

        if (status & max1329::ADD) != 0 {
            self.telemetry.adc = Max1329::read_adc_data_register(self.slot, ecp5);
        }
    }
}

impl SiLPA<SilpaDefault>
{
    fn adc_gt_alarm(&self, _ecp5: &mut ECP5){
        // Sprawdzam IO z FPGA bo tutaj bede mial info ze zostal przekroczony prog
    }

    // fn adc_lt_alarm(&self, _ecp5: &mut ECP5){

    // }

    pub fn check_temperature<T>(&self, i2c: &mut T, servmod: &mut ServMod, channel : u8) -> f32
        where 
        T: Read,    
    { 
        let address : u8;
        match channel{
            1 => address = 0b1001_000,
            2 => address = 0b1001_001,
            _ => panic!("Incorrect LM75 Channel Address")     
        };
        
        match self.slot{
            1 => servmod.0.set_low().unwrap(), 
            2 => servmod.1.set_low().unwrap(),
            3 => servmod.2.set_low().unwrap(),
            4 => servmod.3.set_low().unwrap(),
            5 => servmod.4.set_low().unwrap(),
            6 => servmod.5.set_low().unwrap(),
            7 => servmod.6.set_low().unwrap(),
            8 => servmod.7.set_low().unwrap(),
            _ => log::info!("Incorrect Slot Number")
        };


        match hardware::lm75a::read_temp(i2c, address){ // Addr 0x48
            Ok(temp) => {
                            match self.slot{
                                1 => servmod.0.set_high().unwrap(), 
                                2 => servmod.1.set_high().unwrap(),
                                3 => servmod.2.set_high().unwrap(),
                                4 => servmod.3.set_high().unwrap(),
                                5 => servmod.4.set_high().unwrap(),
                                6 => servmod.5.set_high().unwrap(),
                                7 => servmod.6.set_high().unwrap(),
                                8 => servmod.7.set_high().unwrap(),
                                _ => log::info!("Incorrect Slot Number")
                            };
                            return temp;
                            }
            Err(_e) => panic!("I2C 1st LM75 on Sys_Board error!"),
        };
        
    }

    pub fn activate_channel(&self, ecp5 : &mut ECP5, channel : u8){
        match channel{
            1 => {
                // Dodac odczyt outputow i zrobic or z tym co jest tutaj
                let mut ecp5_inputs : [u8; 2] = [0x00, 0x00];
                ecp5.read_inputs(self.slot, &mut ecp5_inputs);
                ecp5.write_outputs(self.slot, &[ecp5_inputs[0] | ECP5_OUTPUTS::TOGGLE_CH1]);
                // Delay
                ecp5.read_inputs(self.slot, &mut ecp5_inputs);
                ecp5.write_outputs(self.slot, &[ecp5_inputs[0] | 0x00]);
            },
            2 => {
                let mut ecp5_inputs : [u8; 2] = [0x00, 0x00];
                ecp5.read_inputs(self.slot, &mut ecp5_inputs);
                ecp5.write_outputs(self.slot, &[ecp5_inputs[0] | ECP5_OUTPUTS::TOGGLE_CH2]);
                // Delay
                ecp5.read_inputs(self.slot, &mut ecp5_inputs);
                ecp5.write_outputs(self.slot, &[ecp5_inputs[0] | 0x00]);
            }
            _ => log::info!("Incorrect channel number"),
        }
    }
    
}

pub fn vrms_to_dbm_converter(vrms : f32) -> f32{
    let dbm = 30.0 +  20.0 * f32::log10(vrms/(50.0).sqrt());
    return dbm
}

pub fn bits_to_f32(val : u16) -> f32{
    let adc_as_f32 : f32 = (val as f32)/4096.0 * 2.5;
    return adc_as_f32
}

pub fn calculate_output_power(dbm : f32) -> f32{
    let output_power = dbm + 22.0; // 22dB pochodzace z dzielnika 82 i 1k 
    return output_power
}

pub fn calculate_input_power(output_dbm : f32) -> f32{
    let input_power = output_dbm - AMPLIFIER_PARAMETERS::TOTAL_GAIN;
    return input_power
}


