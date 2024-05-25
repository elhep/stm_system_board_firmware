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

pub mod EEPROM_ADDR{
    pub const MAGIC_NUMBER : u8 = 4;
    pub const BOARD_NAME : u8 = 6;
    pub const BOARD_ID : u8 = 16; 
    pub const BOARD_MAJOR_REV : u8 = 20;
    pub const BOARD_MINOR_REV : u8 = 21;
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

    pub fn set_ch_locked(&mut self, channel : u8){
        self.telemetry.is_channel_locked[channel as usize] = 1;
    }

    pub fn set_ch_overheated(&mut self, channel : u8){
        self.telemetry.is_channel_overheated[channel as usize] = 1;
    }

}

#[derive(Clone, Copy, Debug, Miniconf, PartialEq)]
pub struct Settings{
    pub p_threshold      : [f32; 2],  // max 0xFFF
    pub channels_locked : [bool; 2],
    pub channels_tos : [f32; 2],
    // pub channels_thyst : [f32; 2],
    pub telemetry_period: u16,
}

impl Default for Settings{
    fn default() -> Self {
        Self{
            channels_locked : [false, false],
            // channels_thyst  : [55.0, 55.0], // Temperatura powrotu do normalnej pracy
            channels_tos    : [75.0, 75.0], // Temperatura odlaczenia kanalu z powodu przegrzania  
            p_threshold     : [-30.0, -30.0],
            telemetry_period: 2,
        }
    }
}


#[derive(Serialize, Clone, Copy)]
pub struct Telemetry{
    output_power: [f32; 2],
    channel_temperature: [f32; 2],
    is_channel_locked: [u8; 2],
    is_channel_overheated : [u8; 2]
}

impl Default for Telemetry{
    fn default() -> Self {
        Self {  output_power : [0.0, 0.0],
                channel_temperature : [0.0, 0.0],
                is_channel_locked : [0, 0],
                is_channel_overheated : [0, 0]
            }
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

}

impl Variants for SilpaDefault{
    type VariantSettings = Settings;
    type VariantTelemetry = Telemetry;
    type VariantTelemetryBuffer = TelemetryBuffer;
}

impl Devices <Settings, Telemetry> for SiLPA<SilpaDefault>
{
    fn init(&mut self) -> bool {
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

            let mut delay = asm_delay::AsmDelay::new(asm_delay::bitrate::Hertz(
                400000000,
            ));

            // Wpisanie jednorazowo danych do eepromu do kalibracji
            // write_eeprom_addr(&mut bus.cpcis_i2c, 0x40, u8::MAX  as u8); // wpisanie -1
            

            // let mut buffer : [u8; 2] = [0; 2];
            // log::info!("0x50 : {} {}", buffer[0], buffer[1]);
            // log::info!("Wpisywana wartosc: {}", ((3110) >> 8) as u8);

            // let _ = bus.cpcis_i2c.write(0x50, &[EEPROM_ADDR::BOARD_ID, 123 as u8]);
            // let _ = bus.cpcis_i2c.write(0x50, &[EEPROM_ADDR::BOARD_ID + 1, 77 as u8]);
            // let _ = bus.cpcis_i2c.write(0x50, &[0x60, u8::MAX  as u8, (1140 & 0x00FF) as u8, ((1140) >> 8) as u8, 23 as u8, (3110 & 0x00FF) as u8, ((3110) >> 8) as u8]);


            // create_name_array(&mut bus.cpcis_i2c, "SiLPA"); -- wpisanie nazwy urzadzenia

            let mut buffer : [u8; 5] = [0; 5];
            let _ = bus.cpcis_i2c.write_read(0x50, &[EEPROM_ADDR::BOARD_NAME], &mut buffer);
            
            let board_name = match core::str::from_utf8(&buffer) {
                Ok(board_name) => board_name,
                Err(_) => panic!("Failed converting board name"),
            };

            if board_name != "SiLPA"{
                panic!("Wrong board name");
            }

            // write_eeprom_addr(&mut bus.cpcis_i2c, EEPROM_ADDR::BOARD_ID, 123 as u8); 
            // delay.delay_ms(100 as u32);
            // write_eeprom_addr(&mut bus.cpcis_i2c, EEPROM_ADDR::BOARD_ID + 1, 77 as u8); 
            delay.delay_ms(100 as u32);
            read_board_id(&mut bus.cpcis_i2c, &[123 as u8, 77 as u8]); // TODO Change board id to other number

            read_major_rev(&mut bus.cpcis_i2c, 1);
            delay.delay_ms(100 as u32);
            read_minor_rev(&mut bus.cpcis_i2c, 0);
            delay.delay_ms(100 as u32);

            // write_eeprom_addr(&mut bus.cpcis_i2c, EEPROM_ADDR::MAGIC_NUMBER, 0x39); 
            // delay.delay_ms(100 as u32);
            // write_eeprom_addr(&mut bus.cpcis_i2c, EEPROM_ADDR::MAGIC_NUMBER + 1, 0x1E); 
            // delay.delay_ms(100 as u32);

            read_magic_number(&mut bus.cpcis_i2c, &[0x39, 0x1E]);

            bus.ecp5.write_oe(self.slot - 1, &mut [0, 192]);
            bus.ecp5.write_clear_interrupts(self.slot - 1, &mut [0xffu8; 2]);
            bus.ecp5.write_interrupts_mask(self.slot - 1, &mut [0, 48]);
            bus.ecp5.read_interrupts_mask(self.slot - 1, &mut buffer);

            Max1329::setup_ecp5_spi_master(self.slot - 1, &mut bus.ecp5, 0);
            Max1329::reset_device(self.slot - 1, &mut bus.ecp5);
            Max1329::set_clock_control_register(self.slot - 1, &mut bus.ecp5, 0b0100_0011);
            Max1329::set_cpvm_control_register(self.slot - 1, &mut bus.ecp5, 0b0100_1001);

            Max1329::set_adc_control_register(self.slot - 1, &mut bus.ecp5, max1329::adc::AutoConversion::Disabled, max1329::adc::PowerDownConf::Normal, max1329::adc::RefConf::Int2_5);
            Max1329::set_dac_control(self.slot - 1,  &mut bus.ecp5,
                                max1329::dac::PowerDownConf::InOut,
                                max1329::dac::PowerDownConf::InOut,
                                max1329::dac::OpAmp::Disable,
                                max1329::dac::RefConf::Int2_5);
            Max1329::set_daca_value(self.slot - 1, &mut bus.ecp5, 0x0FFF);
            Max1329::set_dacb_value(self.slot - 1, &mut bus.ecp5, 0x0FFF); 
            // Max1329::set_daca_value(self.slot - 1, &mut bus.ecp5, 0x0000);
            // Max1329::set_dacb_value(self.slot - 1, &mut bus.ecp5, 0x0000); 

            Max1329::set_dpio_control_register(self.slot - 1, &mut bus.ecp5, 0xFFFF);
            Max1329::set_dpio_setup_register(self.slot - 1,  &mut bus.ecp5, 0x00);
            Max1329::set_interrupt_mask_register(self.slot - 1, &mut bus.ecp5, !(max1329::ADC | max1329::GTA | max1329::LTA));

            // let mut ecp5_inputs : [u8; 2] = [0x00, 0x00];
            // bus.ecp5.read_inputs(self.slot - 1, &mut ecp5_inputs);
            // log::info!("Wejscia FPGA 1 : {} {}", ecp5_inputs[1], ecp5_inputs[0]);
            // bus.ecp5.write_outputs(self.slot - 1, &mut [0, 0]);
            // bus.ecp5.write_outputs(self.slot - 1, &mut [0, 192]); // Odblookowywanie kanalow

            activate_channel(self.slot - 1, &mut bus.ecp5, 0 as u8);
            activate_channel(self.slot - 1, &mut bus.ecp5, 1 as u8);


            // bus.ecp5.write_outputs(self.slot - 1, &mut [0, 0]);

            // bus.ecp5.read_inputs(self.slot - 1, &mut ecp5_inputs);
            // log::info!("Wejscia FPGA 1 : {} {}", ecp5_inputs[1], ecp5_inputs[0]);

            hardware::lm75a::set_tos(&mut bus.cpcis_i2c, 0b1001_000, self.settings.channels_tos[0]); // TOS CH1
            hardware::lm75a::set_tos(&mut bus.cpcis_i2c, 0b1001_001, self.settings.channels_tos[1]); // TOS CH2

            hardware::lm75a::set_thyst(&mut bus.cpcis_i2c, 0b1001_000, 10.0); // THYST CH1
            hardware::lm75a::set_thyst(&mut bus.cpcis_i2c, 0b1001_001, 10.0);
            // self.detector = read_detector_coefficients(&mut bus.cpcis_i2c);
            (self.slope[0], self.intercept[0]) = set_ch_calibration(&mut bus.cpcis_i2c, 1);
            delay.delay_ms(200 as u32);
            (self.slope[1], self.intercept[1]) = set_ch_calibration(&mut bus.cpcis_i2c, 2);
            log::info!("CH1 kalibracja: {} {}", self.slope[0], self.intercept[0]);
            log::info!("CH2 kalibracja: {} {}", self.slope[1], self.intercept[1]);

            toggle_servmod(&mut bus.servmod, 1, self.slot);

            bus.ecp5.write_clear_interrupts(self.slot - 1, &mut [0xffu8; 2]);
        });

        true
    }

    fn settings_update(&mut self, new_settings: Settings) -> () {
        // Update MAX1329 only if settings changed

        self.bus.lock(| bus| {

            for n in 0..2 {
                if (self.settings.p_threshold[n] != new_settings.p_threshold[n]){
                    calculate_dac_value(self.slot, new_settings.p_threshold[n], (n + 1) as u8, self.slope[n], self.intercept[n], &mut bus.ecp5);
                    log::info!("Zmiana Wartosci DAC {}: {}", n, new_settings.p_threshold[n]);
                    if n == 0 {
                        Max1329::set_adc_setup_direct(self.slot - 1,&mut bus.ecp5, max1329::adc::Mux::OUTA_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
                        while (Max1329::read_status_register(self.slot - 1, &mut bus.ecp5) | (1 << 20)) == 0 {}
                        let mut adc_val = Max1329::read_adc_data_register(self.slot - 1, &mut bus.ecp5);
                        log::info!("CH {} ADC po DAC : {}", n + 1, adc_val.0);
                    } else {
                        Max1329::set_adc_setup_direct(self.slot - 1,&mut bus.ecp5, max1329::adc::Mux::OUTB_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
                        while (Max1329::read_status_register(self.slot - 1, &mut bus.ecp5) | (1 << 20)) == 0 {}
                        let mut adc_val = Max1329::read_adc_data_register(self.slot - 1, &mut bus.ecp5);
                        log::info!("CH {} ADC po DAC : {}", n + 1, adc_val.0);
                    }
                } 

                if self.settings.channels_tos[n] != new_settings.channels_tos[n] {
                    toggle_servmod(&mut bus.servmod, 0, self.slot);
                    log::info!("CH {} TOS : {}", n + 1, new_settings.channels_tos[n]);
                    if n == 0 {
                        hardware::lm75a::set_tos(&mut bus.cpcis_i2c, 0b1001_000, new_settings.channels_tos[n]);
                    } else {
                        hardware::lm75a::set_tos(&mut bus.cpcis_i2c, 0b1001_001, new_settings.channels_tos[n]);
                    }
                    
                    toggle_servmod(&mut bus.servmod, 1, self.slot);
                }
            
                if (new_settings.channels_locked[n] == true) && (self.settings.channels_locked[n] != new_settings.channels_locked[n]) {
                    log::info!("Odblokowywanie {}", n);
                    activate_channel(self.slot - 1, &mut bus.ecp5, (n + 1) as u8);
                    if self.telemetry.telemetry.is_channel_overheated[n] == 0{
                        self.telemetry.telemetry.is_channel_locked[n] = 1;
                    }
                }
            }

            self.settings = new_settings;
        });        
    }

    fn telemetry(&mut self) -> (Telemetry, u16) {

        self.bus.lock(| bus| {
            toggle_servmod(&mut bus.servmod, 0, self.slot);

            // bus.ecp5.write_outputs(1, &mut [0, 192]); // Odblookowywanie kanalow
            // bus.ecp5.write_outputs(1, &mut [0, 0]);

            log::info!("--------------");
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
            
            let mut adc_val = AdcCode(0);

            if self.settings.channels_locked[0] == false {
                Max1329::set_adc_setup_direct(self.slot - 1,&mut bus.ecp5, max1329::adc::Mux::AIN1_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
                while (Max1329::read_status_register(self.slot - 1, &mut bus.ecp5) | (1 << 20)) == 0 {
                }
                adc_val = Max1329::read_adc_data_register(self.slot - 1, &mut bus.ecp5);
                log::info!("CH1 ADC : {}", adc_val.0);

                Max1329::set_adc_setup_direct(self.slot - 1,&mut bus.ecp5, max1329::adc::Mux::OUTA_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
                while (Max1329::read_status_register(self.slot - 1, &mut bus.ecp5) | (1 << 20)) == 0 {}
                let adc_val1 = Max1329::read_adc_data_register(self.slot - 1, &mut bus.ecp5);
                log::info!("CH1 ADC po DAC : {}", adc_val1.0);

                
                // Max1329::set_adc_setup_direct(self.slot - 1,&mut bus.ecp5, max1329::adc::Mux::FBA_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
                // while (Max1329::read_status_register(self.slot - 1, &mut bus.ecp5) | (1 << 20)) == 0 {}
                // let adc_val1 = Max1329::read_adc_data_register(self.slot - 1, &mut bus.ecp5);
                // log::info!("CH1 FBA po DAC : {}", adc_val1.0);

                self.telemetry.set_ch1_output_power_field(adc_val, self.slope[0], self.intercept[0], self.signal_absence[0]);
            }   else {
                adc_val.0 = 0;
                self.telemetry.set_ch1_output_power_field(adc_val, self.slope[0], self.intercept[0], self.signal_absence[0]);
            }


            if self.settings.channels_locked[1] == false {
                Max1329::set_adc_setup_direct(self.slot - 1,&mut bus.ecp5, max1329::adc::Mux::AIN2_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
                while (Max1329::read_status_register(self.slot - 1, &mut bus.ecp5) | (1 << 20)) == 0 {}
                let adc_val = Max1329::read_adc_data_register(self.slot - 1, &mut bus.ecp5);
                log::info!("CH2 ADC : {}", adc_val.0);

                Max1329::set_adc_setup_direct(self.slot - 1,&mut bus.ecp5, max1329::adc::Mux::OUTB_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
                while (Max1329::read_status_register(self.slot - 1, &mut bus.ecp5) | (1 << 20)) == 0 {}
                let adc_val1 = Max1329::read_adc_data_register(self.slot - 1, &mut bus.ecp5);
                log::info!("CH2 ADC po DAC : {}", adc_val1.0);

                // Max1329::set_adc_setup_direct(self.slot - 1,&mut bus.ecp5, max1329::adc::Mux::FBB_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
                // while (Max1329::read_status_register(self.slot - 1, &mut bus.ecp5) | (1 << 20)) == 0 {}
                // let adc_val1 = Max1329::read_adc_data_register(self.slot - 1, &mut bus.ecp5);
                // log::info!("CH2 FBB po DAC : {}", adc_val1.0);

                self.telemetry.set_ch2_output_power_field(adc_val, self.slope[1], self.intercept[1], self.signal_absence[1]);
        
            } else {
                adc_val.0 = 0;
                self.telemetry.set_ch2_output_power_field(adc_val, self.slope[1], self.intercept[1], self.signal_absence[1]);
        
            }

            log::info!("Locked inputs: {} {}", self.telemetry.telemetry.is_channel_locked[0], self.telemetry.telemetry.is_channel_locked[1]);
            log::info!("Overheated channels: {} {}", self.telemetry.telemetry.is_channel_overheated[0], self.telemetry.telemetry.is_channel_overheated[1]);
        });

        // self.telemetry.set_ch_locked[0]
        
        (self.telemetry.finalize(),
         self.settings.telemetry_period)
    }

    fn check_interrupt(&mut self) {

        self.bus.lock(| bus| {
            bus.ecp5.write_clear_interrupts(self.slot - 1,  &mut [0xffu8; 2]);
            let mut ecp5_inputs : [u8; 2] = [0x00, 0x00];
            bus.ecp5.read_inputs(self.slot - 1, &mut ecp5_inputs);
            log::info!("Wartości na wejściach ECP5: {} {}", ecp5_inputs[0], ecp5_inputs[1]);

            toggle_servmod(&mut bus.servmod, 0, self.slot);
            match hardware::lm75a::read_temp(&mut bus.cpcis_i2c, 0b1001_000){
                Ok(temp) => {
                    log::info!("Temp 1: {}", temp);
                    if temp > self.settings.channels_tos[0] {
                        log::info!("CH1 overheating");
                        self.telemetry.set_ch_overheated(0);
                    }
                },
                Err(e) => panic!("{:?}", e),
            };

            match hardware::lm75a::read_temp(&mut bus.cpcis_i2c, 0b1001_001){
                Ok(temp) => {
                    log::info!("Temp 2: {}", temp);
                    if temp > self.settings.channels_tos[1] {
                        log::info!("CH2 overheating");
                        self.telemetry.set_ch_overheated(1);
                    }
                },
                Err(e) => panic!("{:?}", e),
            };
            toggle_servmod(&mut bus.servmod, 1, self.slot);
            if (ecp5_inputs[1] & ECP5_Interrupts::CHANNEL1_INACTIVE == ECP5_Interrupts::CHANNEL1_INACTIVE) {
                // self.settings.channels_locked[0] = true;
                log::info!("Stan wejsc: {}", ecp5_inputs[1]);
                self.telemetry.set_ch_locked(0);
                log::info!("Zablokowano kanał 1");
            }

            if (ecp5_inputs[1] & ECP5_Interrupts::CHANNEL2_INACTIVE == ECP5_Interrupts::CHANNEL2_INACTIVE) {
                // self.settings.channels_locked[1] = true;
                log::info!("Stan wejsc: {}", ecp5_inputs[1]);
                self.telemetry.set_ch_locked(1);
                log::info!("Zablokowano kanał 2");
            }
        });

    }
}

pub fn calculate_dac_value(slot : u8, ptreshold : f32, channel : u8, slope : u16, intercept : u16, ecp5: &mut ECP5){

    let val : u16 = ((slope as f32) * ptreshold + intercept as f32) as u16;
    log::info!("DAC Value CH {} : {}", channel, val);
    match channel{
        1 => {
            // Max1329::set_daca_value(1, ecp5, 0b0000_0010_1000_0011);
            Max1329::set_daca_value(slot - 1, ecp5, val);
        },
        2 => {
            Max1329::set_dacb_value(slot - 1, ecp5, val); // Zamienic na self.slot
            // Max1329::set_dacb_value(1, ecp5, 0b0000_0010_1000_0011);
        },
        _ => log::info!("Incorrect channel nubmer"),  
    }
}

pub fn toggle_servmod(servmod : &mut ServMod, state : u8, slot : u8){
    if state == 0 {
        match slot{
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

    let x1 : i8 = params[0] as i8;
    // log::info!("x1 : {}", x1);
    let x2 : i8 = params[3] as i8;
    // log::info!("x2 : {}", x2);
    // let slope = 1;
    // let intercept = 0;
    let y1 : u16 = (params[2] as u16) << 8 | params[1] as u16;
    // log::info!("y1 : {}", y1);
    let y2 : u16 = (params[5] as u16) << 8 | params[4] as u16;
    // log::info!("y2 : {}", y2);
    // log::info!("---------");
    let slope = (y2-y1)/(x2 as u16 - x1 as u16);
    let intercept = y1 - slope * x1 as u16;


    return (slope, intercept)
}

pub fn activate_channel(slot : u8, ecp5 : &mut ECP5, channel : u8){
    match channel{
        1 => {
            // Dodac odczyt outputow i zrobic or z tym co jest tutaj
            let mut ecp5_inputs : [u8; 2] = [0x00, 0x00];
            ecp5.write_outputs(slot, &[0, 0]);
            ecp5.read_inputs(slot, &mut ecp5_inputs);
            log::info!("Wejscia FPGA 1 : {} {}", ecp5_inputs[1], ecp5_inputs[0]);
            // ecp5.write_outputs(slot, &[0, 0x00 | ECP5_OUTPUTS::TOGGLE_CH1]);
            ecp5.write_outputs(slot, &[0, 64]);
            ecp5.read_inputs(slot, &mut ecp5_inputs);
            log::info!("Wejscia FPGA 2: {} {}", ecp5_inputs[1], ecp5_inputs[0]);
            ecp5.write_outputs(slot, &[0, 0]); // 0 << 6
            ecp5.read_inputs(slot, &mut ecp5_inputs);
            log::info!("Wejscia FPGA 3: {} {}", ecp5_inputs[1], ecp5_inputs[0]);
        },
        2 => {
            let mut ecp5_inputs : [u8; 2] = [0x00, 0x00];
            ecp5.write_outputs(slot, &[0, 0]);
            ecp5.read_inputs(slot, &mut ecp5_inputs);
            log::info!("Wejscia FPGA 1 : {} {}", ecp5_inputs[1], ecp5_inputs[0]);
            // ecp5.write_outputs(slot, &[0, 0x00 | ECP5_OUTPUTS::TOGGLE_CH2]);
            ecp5.write_outputs(slot, &[0, 128]);
            ecp5.read_inputs(slot, &mut ecp5_inputs);
            log::info!("Wejscia FPGA 2: {} {}", ecp5_inputs[1], ecp5_inputs[0]);
            ecp5.write_outputs(slot, &[0, 0]); // 0 << 7
            ecp5.read_inputs(slot, &mut ecp5_inputs);
            log::info!("Wejscia FPGA 3: {} {}", ecp5_inputs[1], ecp5_inputs[0]);
        }
        _ => log::info!("Incorrect channel number"),
    }
}

pub fn read_magic_number<T>(i2c : &mut T, id : &[u8]) 
where
    T : WriteRead
{
    let mut buffer : [u8; 2] = [0; 2];
    let _ = i2c.write_read(0x50, &[EEPROM_ADDR::MAGIC_NUMBER], &mut buffer);

    // log::info!("{} {}", buffer[0], buffer[1]);

    if buffer != id{
        panic!("Wrong magic number")
    }
}

pub fn create_name_array<T>(i2c : &mut T, name : &str) 
where
    T : Write
{
    let mut delay = asm_delay::AsmDelay::new(asm_delay::bitrate::Hertz(400000000));

    let name_bytes = name.bytes();

    for (i, byte) in name_bytes.enumerate() {
        // log::info!("{} {}", i , byte);
        let _ = write_eeprom_addr(i2c, EEPROM_ADDR::BOARD_NAME + i as u8, byte);
        delay.delay_ms(100 as u32);
    }

}

pub fn read_board_id<T>(i2c : &mut T, id : &[u8]) 
where
    T : WriteRead
{
    let mut buffer : [u8; 2] = [0; 2];
    let _ = i2c.write_read(0x50, &[EEPROM_ADDR::BOARD_ID], &mut buffer);

    // log::info!("{} {}", buffer[0], buffer[1]);

    if buffer != id{
        panic!("Wrong board ID")
    }
}

pub fn read_major_rev<T>(i2c : &mut T, rev : u8) 
where
    T : WriteRead
{
    let mut buffer : [u8; 1] = [0; 1];
    let _ = i2c.write_read(0x50, &[EEPROM_ADDR::BOARD_MAJOR_REV], &mut buffer);

    // log::info!("{}", buffer[0]);

    if buffer[0] != rev{
        panic!("Wrong major rev")
    }
}

pub fn read_minor_rev<T>(i2c : &mut T, rev : u8) 
where
    T : WriteRead
{
    let mut buffer : [u8; 1] = [0; 1];
    let _ = i2c.write_read(0x50, &[EEPROM_ADDR::BOARD_MINOR_REV], &mut buffer);

    // log::info!("{}", buffer[0]);

    if buffer[0] != rev{
        panic!("Wrong minor rev")
    }
}