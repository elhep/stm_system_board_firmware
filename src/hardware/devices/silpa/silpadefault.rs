use core::borrow::BorrowMut;
use core::ops::DerefMut;

use embedded_hal::blocking::delay::DelayMs;
use heapless::sorted_linked_list::Max;
use micromath::F32Ext;
use smoltcp_nal::smoltcp::wire::ArpHardware;
use stm32h7xx_hal::i2c::Stop;
use stm32h7xx_hal::pac::i2c1::cr1::PE_A;
use stm32h7xx_hal::pac::i2c1::CR1;
use crate::hardware::eeprom::{check_device_name, read_detector_coefficients, set_device_name, SiLPADetector};
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
use stm32h7xx_hal::pac::I2C4 as bp_i2c;


use embedded_hal::blocking::i2c::{Read, Write, WriteRead};
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
            channels_tos    : [55.0, 55.0], // Temperatura odlaczenia kanalu z powodu przegrzania  
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
            set_device_name(&mut bus.cpcis_i2c);
            check_device_name(&mut bus.cpcis_i2c, "SiLPA".as_bytes());
            toggle_servmod(&mut bus.servmod, 1, self.slot);
            log::info!("Bus locked in Init()");
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

            toggle_servmod(&mut bus.servmod, 0, self.slot);
            hardware::lm75a::set_tos(&mut bus.cpcis_i2c, 0b1001_000, self.settings.channels_tos[0]); // TOS CH1
            hardware::lm75a::set_tos(&mut bus.cpcis_i2c, 0b1001_001, self.settings.channels_tos[1]); // TOS CH2

            hardware::lm75a::set_thyst(&mut bus.cpcis_i2c, 0b1001_000, self.settings.channels_thyst[0]); // THYST CH1
            hardware::lm75a::set_thyst(&mut bus.cpcis_i2c, 0b1001_001, self.settings.channels_thyst[1]);

            self.detector = read_detector_coefficients(&mut bus.cpcis_i2c);
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
                if (self.settings.dacs_value[n] != new_settings.dacs_value[n]) & self.settings.dacs_enable[n] == true {
                    calculate_dac_value(new_settings.dacs_value[n], (n + 1) as u8, &mut bus.ecp5);
                    log::info!("Zmiana Wartosci DAC {}: {}", n, new_settings.dacs_value[n]);
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
                    self.telemetry.set_ch2_temperature(temp);
                },
                Err(e) => panic!("{:?}", e),
            };
            toggle_servmod(&mut bus.servmod, 1, self.slot);

            Max1329::set_adc_setup_direct(1,&mut bus.ecp5, max1329::adc::Mux::AIN1_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
            while (Max1329::read_status_register(1, &mut bus.ecp5) | (1 << 20)) == 0 {
            }
            
            let mut adc_val = Max1329::read_adc_data_register(1, &mut bus.ecp5);
            self.telemetry.set_ch1_output_power_field(adc_val);


            Max1329::set_adc_setup_direct(1,&mut bus.ecp5, max1329::adc::Mux::AIN2_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
            while (Max1329::read_status_register(1, &mut bus.ecp5) | (1 << 20)) == 0 {
            }
            
            adc_val = Max1329::read_adc_data_register(1, &mut bus.ecp5);
            self.telemetry.set_ch2_output_power_field(adc_val);

        });
        
        // let device = stm32h7xx_hal::stm32::Peripherals::take();
        // let i2c = self.backplane.i2c;
        // let mut i2c_ptr = stm32h7xx_hal::stm32::I2C4::ptr();
        // (unsafe { *i2c_ptr }).cr1.write(|w| w.pe().disabled());

        // device.I2C4.cr1.write(|w| unsafe {w.pe().disabled()});
        // delay.delay_ms(1 as u32);
        // device.I2C4.cr1.write(|w| unsafe {w.pe().enabled()});
        // let y = &(self.backplane.i2c);
        // let x = stm32h7xx_hal::stm32::I2C4::ptr();
        // unsafe { x.read().cr1.write(|w| w.pe().disabled()) };
        // delay.delay_ms(2 as u32);
        // unsafe { x.read().cr1.write(|w| w.pe().enabled()) };
        // self.backplane.i2c.master_stop();
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

impl SiLPA<SilpaDefault>
{

    // pub fn check_temperature(&mut self, channel : u8) -> f32   
    // { 
    //     let address : u8;
    //     match channel{
    //         1 => address = 0b1001_000,
    //         2 => address = 0b1001_001,
    //         _ => panic!("Incorrect LM75 Channel Address")     
    //     };
        
    //     self.toggle_servmod(0);

    //     let temp = hardware::lm75a::read_temp(&mut self.backplane.i2c, address); // Addr 0x48
    //     self.toggle_servmod(1);
    //     return temp;
    // }

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

pub fn calculate_dac_value(dbm_value : f32, channel : u8, ecp5: &mut ECP5){

    //TODO add calculations to bit value in OP Amp detector
    match channel{
        1 => {
            Max1329::set_daca_value(1, ecp5, 0b0000_0010_1000_0011);
            // Max1329::set_daca_value(1, ecp5, bit_value);
        },
        2 => {
            // Max1329::set_dacb_value(1, ecp5, bit_value); // Zamienic na self.slot
            Max1329::set_dacb_value(1, ecp5, 0b0000_0010_1000_0011);
        },
        _ => log::info!("Incorrect channel nubmer"),  
    }    
}

pub fn activate_channel(slot : u8, ecp5 : &mut ECP5, channel : u8){
    match channel{
        1 => {
            // Dodac odczyt outputow i zrobic or z tym co jest tutaj
            let mut ecp5_inputs : [u8; 2] = [0x00, 0x00];
            ecp5.read_inputs(slot, &mut ecp5_inputs);
            ecp5.write_outputs(slot, &[ecp5_inputs[0] | ECP5_OUTPUTS::TOGGLE_CH1]);
            // Delay
            ecp5.read_inputs(slot, &mut ecp5_inputs);
            ecp5.write_outputs(slot, &[ecp5_inputs[0] | 0x00]);
        },
        2 => {
            let mut ecp5_inputs : [u8; 2] = [0x00, 0x00];
            ecp5.read_inputs(slot, &mut ecp5_inputs);
            ecp5.write_outputs(slot, &[ecp5_inputs[0] | ECP5_OUTPUTS::TOGGLE_CH2]);
            // Delay
            ecp5.read_inputs(slot, &mut ecp5_inputs);
            ecp5.write_outputs(slot, &[ecp5_inputs[0] | 0x00]);
        }
        _ => log::info!("Incorrect channel number"),
    }
}

pub fn vrms_to_dbm_converter(vrms : f32) -> f32{
    let dbm = 30.0 +  20.0 * f32::log10(vrms/((50.0).sqrt()));
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




