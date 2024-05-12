use core::borrow::BorrowMut;
use core::ops::DerefMut;

use embedded_hal::blocking::delay::DelayMs;
use heapless::sorted_linked_list::Max;
use micromath::F32Ext;
use smoltcp_nal::smoltcp::wire::ArpHardware;
use stm32h7xx_hal::i2c::Stop;
use stm32h7xx_hal::pac::i2c1::cr1::PE_A;
use stm32h7xx_hal::pac::i2c1::CR1;
use crate::hardware::eeprom::{check_device_name, read_ch_calibration, read_detector_coefficients, set_device_name, write_eeprom_addr, SiLPADetector};
use crate::hardware::setup::{SlotsBus};
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
use stm32h7xx_hal::rcc::{rec, CoreClocks, ResetEnable};


use embedded_hal::blocking::i2c::{Read, Write, WriteRead};
pub mod AMPLIFIER_PARAMETERS{
    pub const TOTAL_GAIN : f32 = 56.0; //Gain w dB
    pub const DIVIDER_RATIO : f32 = 0.0; //Dzielnik 82 i 1k     
}
pub mod ECP5_OUTPUTS{
    pub const TOGGLE_CH1 : u8 = 0x40;
    pub const TOGGLE_CH2 : u8 = 0x80;
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

    pub fn set_ch1_output_power_field(&mut self, val : AdcCode, slope : u16, intercept : u16, threshold : u16){
        if val.0 < threshold {
            self.telemetry.output_power[0] = -30.0;
        } else {
            self.telemetry.output_power[0] = ((val.0 as f32 - intercept as f32)/slope as f32) as f32;
        }
    }

    pub fn set_ch2_output_power_field(&mut self, val : AdcCode, slope : u16, intercept : u16, threshold : u16){
        if val.0 < threshold {
            self.telemetry.output_power[1] = -30.0;
        } else {
            self.telemetry.output_power[1] = ((val.0 as f32 - intercept as f32)/slope as f32) as f32;
        }
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
    pub p_threshold      : [f32; 2],  // max 0xFFF
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
            channels_tos    : [75.0, 75.0], // Temperatura odlaczenia kanalu z powodu przegrzania  
            p_threshold     : [0.0, 0.0],
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

// impl Telemetry{
//     pub fn set_ch1_output_power_field(&mut self, val : u16){
//         self.output_power[0] = vrms_to_dbm_converter(bits_to_f32(val));
//     }

//     pub fn set_ch2_output_power_field(&mut self, val : u16){
//         self.output_power[1] = vrms_to_dbm_converter(bits_to_f32(val));
//     }

// }

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

}

impl Variants for SilpaDefault{
    type VariantSettings = Settings;
    type VariantTelemetry = Telemetry;
    type VariantTelemetryBuffer = TelemetryBuffer;
}

impl Devices <Settings, Telemetry> for SiLPA<SilpaDefault>
{
    fn init(&mut self) -> bool {
        // TODO IO and switch control

        // Piny, bity do write_output, read_output
        //  4 - input, przerwanie z kanalu pierwszego
        //  5 - input, przerwanie z kanalu drugiego
        //  6 - output, resetowanie kanalu pierwszego
        //  7 - output, resetowanie kanalu pierwszego
        let x = stm32h7xx_hal::stm32::I2C4::ptr();
        while(unsafe { x.read().isr.read().busy().bit() == true}){
            log::info!("I2C Busy");
        };

        self.bus.lock(| bus| {
            // Jednorazowo ustawic board name w EEPROMIE
            toggle_servmod(&mut bus.servmod, 0, self.slot);

            // Wpisanie jednorazowo danych do eepromu do kalibracji
            write_eeprom_addr(&mut bus.cpcis_i2c, 0x40, u8::MAX  as u8); // wpisanie -1
            write_eeprom_addr(&mut bus.cpcis_i2c, 0x41, (960 & 0x00FF) as u8);
            write_eeprom_addr(&mut bus.cpcis_i2c, 0x42, ((960) >> 8) as u8);
            write_eeprom_addr(&mut bus.cpcis_i2c, 0x43, 23 as u8);
            write_eeprom_addr(&mut bus.cpcis_i2c, 0x44, (2950 & 0x00FF) as u8);
            write_eeprom_addr(&mut bus.cpcis_i2c, 0x45, ((2950) >> 8) as u8);

            write_eeprom_addr(&mut bus.cpcis_i2c, 0x46, u8::MAX  as u8); // wpisanie -1
            write_eeprom_addr(&mut bus.cpcis_i2c, 0x47, (1140 & 0x00FF) as u8);
            write_eeprom_addr(&mut bus.cpcis_i2c, 0x48, ((1140) >> 8) as u8);
            write_eeprom_addr(&mut bus.cpcis_i2c, 0x49, 23 as u8);
            write_eeprom_addr(&mut bus.cpcis_i2c, 0x50, (3110 & 0x00FF) as u8);
            write_eeprom_addr(&mut bus.cpcis_i2c, 0x51, ((3110) >> 8) as u8);


            // set_device_name(&mut bus.cpcis_i2c);
            // check_device_name(&mut bus.cpcis_i2c, "SiLPA".as_bytes());
            toggle_servmod(&mut bus.servmod, 1, self.slot);
            bus.ecp5.write_oe(1, &mut [0, 192]);
            bus.ecp5.write_clear_interrupts(1, &mut [0xffu8; 2]);
            bus.ecp5.write_interrupts_mask(1, &mut [0, 48]);

            Max1329::setup_ecp5_spi_master(1, &mut bus.ecp5, 0);
            Max1329::reset_device(1, &mut bus.ecp5);
            Max1329::set_clock_control_register(1, &mut bus.ecp5, 0b0100_0011);
            Max1329::set_cpvm_control_register(1, &mut bus.ecp5, 0b0100_1001);

            Max1329::set_adc_control_register(1, &mut bus.ecp5, max1329::adc::AutoConversion::Disabled, max1329::adc::PowerDownConf::Normal, max1329::adc::RefConf::Int2_5);
            Max1329::set_dac_control(1,  &mut bus.ecp5,
                                max1329::dac::PowerDownConf::InOut,
                                max1329::dac::PowerDownConf::InOut,
                                max1329::dac::OpAmp::Disable,
                                max1329::dac::RefConf::Int2_5);
            Max1329::set_daca_value(1, &mut bus.ecp5, 0x0FFF); // 1.4 V na wyjsciu DAC, -2 dBm na wejsciu detektora,
            Max1329::set_dacb_value(1, &mut bus.ecp5, 0x0FFF); 

            Max1329::set_dpio_control_register(1, &mut bus.ecp5, 0xFFFF);
            Max1329::set_dpio_setup_register(1,  &mut bus.ecp5, 0x00);
            Max1329::set_interrupt_mask_register(1, &mut bus.ecp5, !(max1329::ADC | max1329::GTA | max1329::LTA));

            bus.ecp5.write_outputs(1, &mut [0, 192]); // Odblookowywanie kanalow
            bus.ecp5.write_outputs(1, &mut [0, 0]);
    
            toggle_servmod(&mut bus.servmod, 0, self.slot);
            hardware::lm75a::set_tos(&mut bus.cpcis_i2c, 0b1001_000, self.settings.channels_tos[0]); // TOS CH1
            hardware::lm75a::set_tos(&mut bus.cpcis_i2c, 0b1001_001, self.settings.channels_tos[1]); // TOS CH2

            hardware::lm75a::set_thyst(&mut bus.cpcis_i2c, 0b1001_000, self.settings.channels_thyst[0]); // THYST CH1
            hardware::lm75a::set_thyst(&mut bus.cpcis_i2c, 0b1001_001, self.settings.channels_thyst[1]);

            // self.detector = read_detector_coefficients(&mut bus.cpcis_i2c);
            (self.slope[0], self.intercept[0]) = set_ch_calibration(&mut bus.cpcis_i2c, 1);
            (self.slope[1], self.intercept[1]) = set_ch_calibration(&mut bus.cpcis_i2c, 2);
            log::info!("CH1 kalibracja: {} {}", self.slope[0], self.intercept[0]);
            log::info!("CH2 kalibracja: {} {}", self.slope[1], self.intercept[1]);

            toggle_servmod(&mut bus.servmod, 1, self.slot);
        });

        true
    }

    fn settings_update(&mut self, new_settings: Settings) -> () {
        // Update MAX1329 only if settings changed
        let mut delay = asm_delay::AsmDelay::new(asm_delay::bitrate::Hertz(
            400000000,
        ));

        self.bus.lock(| bus| {
            log::info!("Bus locked in Settings_update()");
            if self.settings.dacs_enable != new_settings.dacs_enable {
                Max1329::set_dac_control(1, &mut bus.ecp5, match new_settings.dacs_enable[0] { true => dac::PowerDownConf::InToOut,
                                                                                          false => dac::PowerDownConf::PowerDown,},
                                                      match new_settings.dacs_enable[1] { true => dac::PowerDownConf::InToOut,
                                                                                          false => dac::PowerDownConf::PowerDown,},
                                                      dac::OpAmp::Disable,
                                                      dac::RefConf::Ext1_0,);
            log::info!("DACS Enable: {} {}", new_settings.dacs_enable[0], new_settings.dacs_enable[1]);
            log::info!("Chanels locked: {} {}", new_settings.channels_locked[0], new_settings.channels_locked[1]);

            for n in 0..2 {
                if (self.settings.p_threshold[n] != new_settings.p_threshold[n]) & self.settings.dacs_enable[n] == true {
                    calculate_dac_value(new_settings.p_threshold[n], (n + 1) as u8, self.slope[n], self.intercept[n], &mut bus.ecp5);
                    log::info!("Zmiana Wartosci DAC {}: {}", n, new_settings.p_threshold[n]);
                } 

                if self.settings.channels_tos[n] != new_settings.channels_tos[n] {
                    toggle_servmod(&mut bus.servmod, 0, self.slot);
                    hardware::lm75a::set_tos(&mut bus.cpcis_i2c, 0b1001_000, new_settings.channels_tos[n]);
                    toggle_servmod(&mut bus.servmod, 1, self.slot);
                }

                if self.settings.channels_thyst[n] != new_settings.channels_thyst[n]{
                    toggle_servmod(&mut bus.servmod, 0, self.slot);
                    hardware::lm75a::set_thyst(&mut bus.cpcis_i2c, hardware::lm75a::I2C_ADDR[n], new_settings.channels_thyst[n]);
                    toggle_servmod(&mut bus.servmod, 1, self.slot);  
                }
            
            // TODO do przetestowania odblokowywanie kanalow
                if (new_settings.channels_locked[n] == false) && (self.settings.channels_locked[n] == true) {
                    log::info!("Odblokowywanie {}", n);
                    activate_channel(1, &mut bus.ecp5, n as u8);
                }
            }

            let channels_locked_tmp = self.settings.channels_locked;
            self.settings = new_settings;
            self.settings.channels_locked = channels_locked_tmp;
        }
        });        
    }

    fn telemetry(&mut self) -> (Telemetry, u16) {
        let mut delay = asm_delay::AsmDelay::new(asm_delay::bitrate::Hertz(
            400000000,
        ));

        self.bus.lock(| bus| {
            toggle_servmod(&mut bus.servmod, 0, self.slot);

            let x = stm32h7xx_hal::stm32::I2C4::ptr();
            while(unsafe { x.read().isr.read().busy().bit() == true}){
                log::info!("I2C Busy");
            };

            match hardware::lm75a::read_temp(&mut bus.cpcis_i2c, 0b1001_000){
                Ok(temp) => {
                    log::info!("Temp 1: {}", temp);
                    self.telemetry.set_ch1_temperature(temp);
                },
                Err(e) => panic!("{:?}", e),
            };
            match hardware::lm75a::read_temp(&mut bus.cpcis_i2c, 0b1001_001){
                Ok(temp) => {
                    log::info!("Temp 2: {}", temp);
                    self.telemetry.set_ch2_temperature(temp);
                },
                Err(e) => panic!("{:?}", e),
            };
            toggle_servmod(&mut bus.servmod, 1, self.slot);

            Max1329::set_adc_setup_direct(1,&mut bus.ecp5, max1329::adc::Mux::AIN1_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
            while (Max1329::read_status_register(1, &mut bus.ecp5) | (1 << 20)) == 0 {
            }
            
            let mut adc_val = Max1329::read_adc_data_register(1, &mut bus.ecp5);
            log::info!("CH1 ADC : {}", adc_val.0);
            self.telemetry.set_ch1_output_power_field(adc_val, self.slope[0], self.intercept[0], self.signal_absence[0]);
            log::info!("CH1 Power : {}", self.telemetry.telemetry.output_power[0]);

            Max1329::set_adc_setup_direct(1,&mut bus.ecp5, max1329::adc::Mux::AIN2_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
            while (Max1329::read_status_register(1, &mut bus.ecp5) | (1 << 20)) == 0 {
            }
            
            adc_val = Max1329::read_adc_data_register(1, &mut bus.ecp5);
            log::info!("CH2 ADC : {}", adc_val.0);
            self.telemetry.set_ch2_output_power_field(adc_val, self.slope[1], self.intercept[1], self.signal_absence[1]);
            log::info!("CH2 Power : {}", self.telemetry.telemetry.output_power[1]);

        });
        
        (self.telemetry.finalize(),
         self.settings.telemetry_period)
    }

    fn check_interrupt(&mut self) {

        self.bus.lock(| bus| {
            bus.ecp5.write_clear_interrupts(1,  &mut [0xffu8; 2]);
            let mut ecp5_inputs : [u8; 2] = [0x00, 0x00];
            bus.ecp5.read_inputs(1, &mut ecp5_inputs);
            // - 4 - input, Kanal 1, '1' - input odciety, '0' - sygnal jest wzmacniany
            // - 5 - input, Kanal 2, '1' - input odciety, '0' - sygnal jest wzmacniany
            // - 6 - output - Kanal 1, '1' ustawienie na '1' resetuje uklad i wzmacniacz dziala
            // - 7 - output - Kanal 2, '1' ustawienie na '1' resetuje uklad i wzmacniacz dziala
            if (ecp5_inputs[1] & ECP5_Interrupts::CHANNEL1_INACTIVE == ECP5_Interrupts::CHANNEL1_INACTIVE) {
                self.settings.channels_locked[0] = true;
                log::info!("Zablokowano kanał 1");
            }

            if (ecp5_inputs[1] & ECP5_Interrupts::CHANNEL2_INACTIVE == ECP5_Interrupts::CHANNEL2_INACTIVE) {
                self.settings.channels_locked[1] = true;
                log::info!("Zablokowano kanał 2");
            }
        });

    }
}

pub fn calculate_dac_value(ptreshold : f32, channel : u8, slope : u16, intercept : u16, ecp5: &mut ECP5){

    let val : u16 = ((slope as f32) * ptreshold + intercept as f32) as u16;

    match channel{
        1 => {
            Max1329::set_daca_value(1, ecp5, 0b0000_0010_1000_0011);
            // Max1329::set_daca_value(1, ecp5, val);
        },
        2 => {
            // Max1329::set_dacb_value(1, ecp5, val); // Zamienic na self.slot
            Max1329::set_dacb_value(1, ecp5, 0b0000_0010_1000_0011);
        },
        _ => log::info!("Incorrect channel nubmer"),  
    }
}

pub fn toggle_servmod(servmod : &mut ServMod, state : u8, slot : u8){
    if state == 0 {
        match slot{
            1 => servmod.1.set_low().unwrap(),
            2 => servmod.1.set_low().unwrap(),
            3 => servmod.2.set_low().unwrap(),
            4 => servmod.3.set_low().unwrap(),
            5 => servmod.4.set_low().unwrap(),
            6 => servmod.5.set_low().unwrap(),
            7 => servmod.6.set_low().unwrap(),
            8 => servmod.7.set_low().unwrap(),
            _ => log::info!("Incorrect Slot Number")
        };
    } else if state == 1 {
        match slot{
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
    }
}

pub fn set_ch_calibration<T>(i2c : &mut T, ch : u8) -> (u16, u16)
where
    T: WriteRead,
{
    let mut params : [u8; 6] = [0; 6];
    params = read_ch_calibration(i2c, ch);

    let x1 : u8 = params[0];
    let x2 : u8 = params[3];

    let y1 : u16 = (params[2] as u16) << 8 | params[1] as u16;
    let y2 : u16 = (params[5] as u16) << 8 | params[4] as u16;
    let slope = (y2-y1)/(x2 as u16 - x1 as u16);
    let intercept = y1 - slope * x1 as u16;

    return (slope, intercept)
}

pub fn activate_channel(slot : u8, ecp5 : &mut ECP5, channel : u8){
    match channel{
        1 => {
            // Dodac odczyt outputow i zrobic or z tym co jest tutaj
            let mut ecp5_inputs : [u8; 2] = [0x00, 0x00];
            ecp5.read_inputs(slot, &mut ecp5_inputs);
            ecp5.write_outputs(slot, &[ecp5_inputs[0], ecp5_inputs[1] | ECP5_OUTPUTS::TOGGLE_CH1]);
            ecp5.write_outputs(slot, &[ecp5_inputs[0], ecp5_inputs[1] ^ ECP5_OUTPUTS::TOGGLE_CH1]);
        },
        2 => {
            let mut ecp5_inputs : [u8; 2] = [0x00, 0x00];
            ecp5.read_inputs(slot, &mut ecp5_inputs);
            ecp5.write_outputs(slot, &[ecp5_inputs[0], ecp5_inputs[1] | ECP5_OUTPUTS::TOGGLE_CH2]);
            ecp5.write_outputs(slot, &[ecp5_inputs[0], ecp5_inputs[1] ^ ECP5_OUTPUTS::TOGGLE_CH2]);
        }
        _ => log::info!("Incorrect channel number"),
    }
}

pub fn bits_to_f32(val : u16) -> f32{
    let adc_as_f32 : f32 = (val as f32)/4096.0 * 2.5;
    return adc_as_f32
}