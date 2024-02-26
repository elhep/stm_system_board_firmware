use heapless::sorted_linked_list::Max;
use micromath::F32Ext;
use crate::hardware::setup::BackPlaneI2C;
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
    pub const DIVIDER_RATIO : f32 = 0.0; //Dzielnik 82 i 1k     
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

    pub fn set_ch1_output_power_field(&mut self, val : AdcCode){
        self.telemetry.output_power[0] = calculate_output_power(vrms_to_dbm_converter(bits_to_f32(val.0)));
    }

    pub fn set_ch2_output_power_field(&mut self, val : AdcCode){
        self.telemetry.output_power[1] = calculate_output_power(vrms_to_dbm_converter(bits_to_f32(val.0)));
    }

    pub fn set_ch1_temperature(&mut self, temp : f32){
        self.telemetry.channel_temperature[0] = temp;
    }

    pub fn set_ch2_temperature(&mut self, temp : f32){
        self.telemetry.channel_temperature[1] = temp;
    }
}

#[derive(Clone, Copy, Debug, Miniconf, PartialEq)]
pub struct Settings{
    adc_gt_threshold: u16,  // max 0xFFF
    adc_lt_threshold: u16,  // max 0xFFF
    dacs_enable     : [bool; 2],
    pub dacs_value      : [f32; 2],  // max 0xFFF
    pub channels_locked : [bool; 2],
    pub channels_tos : [f32; 2],
    pub channels_thyst : [f32; 2],
    pub telemetry_period: u16,
}

impl Default for Settings{
    fn default() -> Self {
        Self{
            adc_gt_threshold: 0xFFF,
            adc_lt_threshold: 0x000,
            dacs_enable     : [false, false],
            channels_locked : [false, false],
            channels_thyst  : [31.0, 31.0], // Temperatura powrotu do normalnej pracy
            channels_tos    : [35.0, 35.0], // Temperatura odlaczenia kanalu z powodu przegrzania  
            dacs_value      : [0.0, 0.0],
            telemetry_period: 2,
        }
    }
}


#[derive(Serialize, Clone, Copy)]
pub struct Telemetry{
    output_power: [f32; 2],
    channel_temperature: [f32; 2]
}

impl Default for Telemetry{
    fn default() -> Self {
        Self {  output_power : [0.0, 0.0],
                channel_temperature : [0.0, 0.0]    
            }
    }
}

impl Telemetry{
    pub fn set_ch1_output_power_field(&mut self, val : u16){
        self.output_power[0] = vrms_to_dbm_converter(bits_to_f32(val));
    }

    pub fn set_ch2_output_power_field(&mut self, val : u16){
        self.output_power[1] = vrms_to_dbm_converter(bits_to_f32(val));
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

pub struct SilpaDefault{
    backplane_i2c : BackPlaneI2C,
    detector : hardware::eeprom::SiLPADetector,
}

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

        Max1329::setup_ecp5_spi_master(1, ecp5, 0);
        Max1329::reset_device(1, ecp5);

        Max1329::set_clock_control_register(1, ecp5, 0b0100_0011);
        Max1329::set_cpvm_control_register(1, ecp5, 0b0100_1001);

        Max1329::set_adc_control_register(1, ecp5, max1329::adc::AutoConversion::Disabled, max1329::adc::PowerDownConf::Normal, max1329::adc::RefConf::Int2_5);
        Max1329::set_dac_control(1,  ecp5,
                                max1329::dac::PowerDownConf::InOut,
                                max1329::dac::PowerDownConf::InOut,
                                max1329::dac::OpAmp::Disable,
                                max1329::dac::RefConf::Int2_5);

        Max1329::set_daca_value(1, ecp5, 0b0000_1000_1111_0101); // 1.4 V na wyjsciu DAC, -2 dBm na wejsciu detektora,
        Max1329::set_dacb_value(1, ecp5, 0b0000_0010_1111_0101); 

        Max1329::set_dpio_control_register(1, ecp5, 0xFFFF);
        Max1329::set_dpio_setup_register(1,  ecp5, 0x03);

        Max1329::set_interrupt_mask_register(self.slot, ecp5, !(max1329::ADC | max1329::GTA | max1329::LTA));   
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
            let ecp5_inputs : [u8; 2] = [0x00, 0x00];

            if self.settings.channels_locked[0] != new_settings.channels_locked[0] {
                log::info!("Odblokowywanie CH1");
            }

            if self.settings.channels_locked[1] != new_settings.channels_locked[1] {
                log::info!("Odblokowywanie CH2");
            }
            // if ecp5_inputs[0] & (1 << ECP5_INPUTS::CHANNEL1) == 0{
            //     self.activate_channel(ecp5, 1);
            // }

            // if ecp5_inputs[0] & (1 << ECP5_INPUTS::CHANNEL2) == 0{
            //     self.activate_channel(ecp5, 2);
            // }         
        }

        self.settings = new_settings;
    }

    fn telemetry(&mut self, ecp5: &mut ECP5) -> (Telemetry, u16) {


        Max1329::set_adc_setup_direct(1, ecp5, max1329::adc::Mux::AIN1_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
        while (Max1329::read_status_register(1, ecp5) | (1 << 20)) == 0 {
        }
        
        let adc_val = Max1329::read_adc_data_register(1, ecp5);
        // log::info!("Wartosc z ADC1: {}", adc_val.0);
        // log::info!("Moc wejsciowa na CH1: {}", vrms_to_dbm_converter((adc_val.0 as f32) * 2.5 / 4095.0 /1.5));

        Max1329::set_adc_setup_direct(1, ecp5, max1329::adc::Mux::OUTA_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
        while (Max1329::read_status_register(1, ecp5) | (1 << 20)) == 0 {
        }
        
        let adc_val = Max1329::read_adc_data_register(1, ecp5);
        // log::info!("Wartosc z DAC1: {}", adc_val.0);


        self.telemetry.set_ch1_output_power_field(adc_val);

        Max1329::set_adc_setup_direct(1, ecp5, max1329::adc::Mux::AIN2_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
        while (Max1329::read_status_register(1, ecp5) | (1 << 20)) == 0 {
        } 

        let adc_val = Max1329::read_adc_data_register(1, ecp5);
        // log::info!("Wartosc z ADC2: {}", adc_val.0);
        // log::info!("Moc wejsciowa na CH1: {}", vrms_to_dbm_converter((adc_val.0 as f32) * 2.5 / 4095.0 /1.5));

        Max1329::set_adc_setup_direct(1, ecp5, max1329::adc::Mux::OUTB_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
        while (Max1329::read_status_register(1, ecp5) | (1 << 20)) == 0 {
        }
        
        let adc_val = Max1329::read_adc_data_register(1, ecp5);
        // log::info!("Wartosc z DAC2: {}", adc_val.0);

        let adc_val = Max1329::read_adc_data_register(1, ecp5);
        self.telemetry.set_ch2_output_power_field(adc_val);
        (self.telemetry.finalize(),
         self.settings.telemetry_period)
    }

    fn check_interrupt(&mut self, ecp5: &mut ECP5) {

        // ODczyt inputow z FPGA //
        let mut ecp5_inputs : [u8; 2] = [0x00, 0x00];
        ecp5.read_inputs(1, &mut ecp5_inputs);

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

        let status : u32 = Max1329::read_status_register(1, ecp5);

        if (status & max1329::GTA) != 0 {
            self.adc_gt_alarm(ecp5);
        }

        // if (status & max1329::LTA) != 0 {
        //     self.adc_lt_alarm(ecp5);
        // }

        if (status & max1329::ADD) != 0 {
            self.telemetry.adc = Max1329::read_adc_data_register(1, ecp5);
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

    pub fn check_temperature<T>(&mut self, i2c: &mut T, servmod: &mut ServMod, channel : u8) -> f32
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
                            match channel{
                                1 => self.telemetry.set_ch1_temperature(temp),
                                2 => self.telemetry.set_ch2_temperature(temp),
                                _ => panic!("Incorrect channel number")
                            }
                            
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

    pub fn calculate_dac_value(&mut self, slope : f32, intercept : f32, ptreshold : f32, channel : u8, ecp5: &mut ECP5){
        let vin_detector = slope * (f32::sqrt(0.05 /f32::log10((ptreshold - 26.0)/10.0)) - intercept);
        let bit_value : u16 = (vin_detector/1000.0 * 4095.0/ 2.5) as u16;  // 26 dB pochodzi z dzielnika 1k i 50 Ohm 
        // Dzielenie przez 1000 aby zamienic na V z mV

        match channel{
            1 => {
                Max1329::set_daca_value(1, ecp5, 0b0000_0010_1000_0011);
                // Max1329::set_daca_value(1, ecp5, bit_value);
                self.settings.dacs_value[0] = ptreshold;
            },
            2 => {
                // Max1329::set_dacb_value(1, ecp5, bit_value); // Zamienic na self.slot
                Max1329::set_dacb_value(1, ecp5, 0b0000_0010_1000_0011);
                self.settings.dacs_value[1] = ptreshold;
            },
            _ => log::info!("Incorrect channel nubmer"),  
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
    let output_power = dbm + 26.0; // 26dB pochodzace z dzielnika 50 i 1k 
    return output_power
}

pub fn calculate_input_power(output_dbm : f32) -> f32{
    let input_power = output_dbm - AMPLIFIER_PARAMETERS::TOTAL_GAIN;
    return input_power
}




