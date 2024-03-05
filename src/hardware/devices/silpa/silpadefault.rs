use heapless::sorted_linked_list::Max;
use micromath::F32Ext;
use crate::hardware::eeprom::{read_detector_coefficients, SiLPADetector};
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
            channels_thyst  : [10.0, 10.0], // Temperatura powrotu do normalnej pracy
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
    pub const CHANNEL1_INACTIVE : u8 = 0x10;
    pub const CHANNEL2_INACTIVE : u8 = 0x20;
    pub const CHANNEL1_INPUT_BIT : u8 = 4;
    pub const CHANNEL2_INPUT_BIT : u8 = 5;
}

pub mod ECP5_INPUTS{
    pub const CHANNEL1 : u8 = 0x03;
    pub const CHANNEL2 : u8 = 0x02;
}

pub struct SilpaDefault{
    x : u8
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

        // Piny, bity do write_output, read_output
        //  4 - input, przerwanie z kanalu pierwszego
        //  5 - input, przerwanie z kanalu drugiego
        //  6 - output, resetowanie kanalu pierwszego
        //  7 - output, resetowanie kanalu pierwszego

        ecp5.write_oe(1, &mut [0, 192]);
        ecp5.write_clear_interrupts(1, &mut [0xffu8; 2]);

        ecp5.write_interrupts_mask(1, &mut [0, 48]);

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
        let mut data = Max1329::read_daca_value(1, ecp5);
        assert_eq!(0b0000_1000_1111_0101, data, "DACA correct Value");

        Max1329::set_dacb_value(1, ecp5, 0b0000_0010_1111_0101); 
        data = Max1329::read_dacb_value(1, ecp5);
        assert_eq!(0b0000_0010_1111_0101, data, "DACB correct Value");

        Max1329::set_dpio_control_register(1, ecp5, 0xFFFF);
        Max1329::set_dpio_setup_register(1,  ecp5, 0x03);
        Max1329::set_interrupt_mask_register(1, ecp5, !(max1329::ADC | max1329::GTA | max1329::LTA));   
        
        ecp5.write_outputs(1, &mut [0, 128]);
        ecp5.write_outputs(1, &mut [0, 0]);


        log::info!("Testy LM75 I2C w Init()");
        self.toggle_servmod(0);

        hardware::lm75a::set_tos(&mut self.backplane.i2c, 0b1001_000, self.settings.channels_tos[0]);
        let mut tos = hardware::lm75a::read_tos(&mut self.backplane.i2c, 0b1001_000);
        log::info!("TOS CH1: {}", tos);
        // assert_eq!(self.settings.channels_tos[0], tos, "TOS check CH1");

        hardware::lm75a::set_tos(&mut self.backplane.i2c, 0b1001_001, self.settings.channels_tos[1]);
        tos = hardware::lm75a::read_tos(&mut self.backplane.i2c, 0b1001_001);
        log::info!("TOS CH2: {}", tos);
        // assert_eq!(self.settings.channels_tos[0], tos, "TOS check CH2");
        
        hardware::lm75a::set_thyst(&mut self.backplane.i2c, 0b1001_000, self.settings.channels_thyst[0]);
        let mut thyst = hardware::lm75a::read_thyst(&mut self.backplane.i2c, 0b1001_000);
        log::info!("THYST CH1: {}", thyst);
        // assert_eq!(self.settings.channels_tos[0], thyst, "THYST check CH1");

        hardware::lm75a::set_thyst(&mut self.backplane.i2c, 0b1001_001, self.settings.channels_thyst[1]);
        thyst = hardware::lm75a::read_thyst(&mut self.backplane.i2c, 0b1001_001);
        log::info!("THYST CH2 {}", thyst);
        // assert_eq!(self.settings.channels_tos[0], thyst, "THYST check CH2");    
        
        self.detector = read_detector_coefficients(&mut self.backplane.i2c);
        self.toggle_servmod(1);
        true
    }

    fn settings_update(&mut self, ecp5: &mut ECP5, new_settings: Settings) -> () {
        // Update MAX1329 only if settings changed
        if self.settings.adc_gt_threshold != new_settings.adc_gt_threshold {
            Max1329::set_adc_gt_alarm_register(1, ecp5, adc::AlarmMode::NonConsecutive,
                                                                1,
                                                                new_settings.adc_gt_threshold);
        }

        if self.settings.adc_lt_threshold != new_settings.adc_lt_threshold {
            Max1329::set_adc_lt_alarm_register(1, ecp5, adc::AlarmMode::NonConsecutive,
                                                                1,
                                                                new_settings.adc_lt_threshold);
        }

        if self.settings.dacs_enable != new_settings.dacs_enable {
            Max1329::set_dac_control(1, ecp5, match new_settings.dacs_enable[0] { true => dac::PowerDownConf::InToOut,
                                                                                          false => dac::PowerDownConf::PowerDown,},
                                                      match new_settings.dacs_enable[1] { true => dac::PowerDownConf::InToOut,
                                                                                          false => dac::PowerDownConf::PowerDown,},
                                                      dac::OpAmp::Disable,
                                                      dac::RefConf::Ext1_0,);
            log::info!("DACS Enable: {} {}", new_settings.dacs_enable[0], new_settings.dacs_enable[1]);
        }

        // for n in 0..2 {
        //     if (self.settings.dacs_value[n] != new_settings.dacs_value[n]) & self.settings.dacs_enable[n] == true {
        //         self.calculate_dac_value(new_settings.dacs_value[n], (n + 1) as u8, ecp5);
        //         log::info!("Zmiana Wartosci DAC {}: {}", n, new_settings.dacs_value[n]);
        //     } 

        //     if self.settings.channels_tos[n] != new_settings.channels_tos[n] {
        //         self.toggle_servmod(0);
        //         hardware::lm75a::set_tos(&mut self.backplane.i2c, 0b1001_000, new_settings.channels_tos[n]);
        //         let tos = hardware::lm75a::read_tos(&mut self.backplane.i2c, hardware::lm75a::I2C_ADDR[n]);
        //         log::info!("TOS {}: {}", n, tos);
        //         self.toggle_servmod(1);
        //     }

        //     if self.settings.channels_thyst[n] != new_settings.channels_thyst[n]{
        //         self.toggle_servmod(0);
        //         hardware::lm75a::set_thyst(&mut self.backplane.i2c, hardware::lm75a::I2C_ADDR[n], new_settings.channels_thyst[n]);
        //         let thyst = hardware::lm75a::read_thyst(&mut self.backplane.i2c, hardware::lm75a::I2C_ADDR[n]);
        //         log::info!("THYST {}: {}", n, thyst);
        //         self.toggle_servmod(1);   
        //     }
        // }



        if self.settings.channels_locked != new_settings.channels_locked {
            let mut ecp5_inputs : [u8; 2] = [0x00, 0x00];
            ecp5.read_inputs(1, &mut ecp5_inputs);

            if self.settings.channels_locked[0] != new_settings.channels_locked[0] {
                log::info!("Odblokowywanie CH1");
            }

            if self.settings.channels_locked[1] != new_settings.channels_locked[1] {
                log::info!("Odblokowywanie CH2");
            }
            // if ecp5_inputs[1] & (1 << ECP5_INPUTS::CHANNEL1) == 0{
            //     self.activate_channel(ecp5, 1);
            // }

            // if ecp5_inputs[1] & (1 << ECP5_INPUTS::CHANNEL2) == 0{
            //     self.activate_channel(ecp5, 2);
            // }         
        }

        self.settings = new_settings;
    }

    fn telemetry(&mut self, ecp5: &mut ECP5) -> (Telemetry, u16) {

        for n in 0..2{
            let mut temp1 = self.check_temperature(n + 1);
            log::info!("TEMP {}: {}", n, temp1);
        }
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
        ecp5.write_clear_interrupts(1, &mut [0xffu8; 2]);
        let mut ecp5_inputs : [u8; 2] = [0x00, 0x00];
        ecp5.read_inputs(1, &mut ecp5_inputs);
        log::info!("Wartości na wejściach ECP5: {} {}", ecp5_inputs[0], ecp5_inputs[1]);

        // Opis lini LVDS:
        // - 4 - input, Kanal 1, '1' - input odciety, '0' - sygnal jest wzmacniany
        // - 5 - input, Kanal 2, '1' - input odciety, '0' - sygnal jest wzmacniany
        // - 6 - output - Kanal 1, '1' ustawienie na '1' resetuje uklad i dziala
        // - 7 - output - Kanal 2, '1' ustawienie na '1' resetuje uklad i dziala
        if (ecp5_inputs[1] & ECP5_Interrupts::CHANNEL1_INACTIVE == ECP5_Interrupts::CHANNEL1_INACTIVE) {
            self.settings.channels_locked[0] = true;
            log::info!("Zablokowano kanał 1");
        }

        if (ecp5_inputs[1] & ECP5_Interrupts::CHANNEL2_INACTIVE == ECP5_Interrupts::CHANNEL2_INACTIVE) {
            self.settings.channels_locked[1] = true;
            log::info!("Zablokowano kanał 2");
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
    pub fn toggle_servmod(&mut self, state : u8){
        if state == 0 {
            match self.slot{
                1 => self.backplane.servmod.0.set_low().unwrap(), 
                2 => self.backplane.servmod.1.set_low().unwrap(),
                3 => self.backplane.servmod.2.set_low().unwrap(),
                4 => self.backplane.servmod.3.set_low().unwrap(),
                5 => self.backplane.servmod.4.set_low().unwrap(),
                6 => self.backplane.servmod.5.set_low().unwrap(),
                7 => self.backplane.servmod.6.set_low().unwrap(),
                8 => self.backplane.servmod.7.set_low().unwrap(),
                _ => log::info!("Incorrect Slot Number")
            };
        } else if state == 1 {
            match self.slot{
                1 => self.backplane.servmod.0.set_high().unwrap(), 
                2 => self.backplane.servmod.1.set_high().unwrap(),
                3 => self.backplane.servmod.2.set_high().unwrap(),
                4 => self.backplane.servmod.3.set_high().unwrap(),
                5 => self.backplane.servmod.4.set_high().unwrap(),
                6 => self.backplane.servmod.5.set_high().unwrap(),
                7 => self.backplane.servmod.6.set_high().unwrap(),
                8 => self.backplane.servmod.7.set_high().unwrap(),
                _ => log::info!("Incorrect Slot Number")
            };  
        }
    }

    pub fn check_temperature(&mut self, channel : u8) -> f32   
    { 
        let address : u8;
        match channel{
            1 => address = 0b1001_000,
            2 => address = 0b1001_001,
            _ => panic!("Incorrect LM75 Channel Address")     
        };
        
        match self.slot{
            1 => self.backplane.servmod.0.set_low().unwrap(), 
            2 => self.backplane.servmod.1.set_low().unwrap(),
            3 => self.backplane.servmod.2.set_low().unwrap(),
            4 => self.backplane.servmod.3.set_low().unwrap(),
            5 => self.backplane.servmod.4.set_low().unwrap(),
            6 => self.backplane.servmod.5.set_low().unwrap(),
            7 => self.backplane.servmod.6.set_low().unwrap(),
            8 => self.backplane.servmod.7.set_low().unwrap(),
            _ => log::info!("Incorrect Slot Number")
        };

        match hardware::lm75a::read_temp(&mut self.backplane.i2c, address){ // Addr 0x48
            Ok(temp) => {
                            match channel{
                                1 => self.telemetry.set_ch1_temperature(temp),
                                2 => self.telemetry.set_ch2_temperature(temp),
                                _ => panic!("Incorrect channel number")
                            }
                            
                            match self.slot{
                                2 => self.backplane.servmod.0.set_high().unwrap(),
                                1 => self.backplane.servmod.1.set_high().unwrap(), 
                                3 => self.backplane.servmod.2.set_high().unwrap(),
                                4 => self.backplane.servmod.3.set_high().unwrap(),
                                5 => self.backplane.servmod.4.set_high().unwrap(),
                                6 => self.backplane.servmod.5.set_high().unwrap(),
                                7 => self.backplane.servmod.6.set_high().unwrap(),
                                8 => self.backplane.servmod.7.set_high().unwrap(),
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

    pub fn calculate_dac_value(&mut self, ptreshold : f32, channel : u8, ecp5: &mut ECP5){

        //TODO add calculations to bit value in OP Amp detector
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

    pub fn calculate_dac_detector_value(&mut self, slope : f32, intercept : f32, ptreshold : f32, channel : u8, ecp5: &mut ECP5){
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




