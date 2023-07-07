/*
I2C:
    Addr: - 0xA0 IC16 EEPROM 24AA02E48
    Addr: - IC26 TEMP SENS LM75AIMM
    Addr: - IC27 TEMP SENS LM75AIMM

Scenario:
    Test System Reset BRST_N
        - check RFSW_DIS0 and RFSW_DIS1 - should be 0; LED_DIS0 LED_DIS1 LED_OVT OFF
        - set RESET
        - check RFSW_DIS0 and RFSW_DI1 - should be 1; LED_DIS0 LED_DIS1 LED_OVT ON
        - release RESET
    Test I2C CPCIS + servmod:
        - Write to EEPROM + readout
        - 2 x: check temp. Set temp threshold to 10 C - check OVT diode.
    Test MAX1329
        - Configure IOs (APIO inputs DPIO outputs)
        - Read HWREV expected 0x01
    Test DET_RST (2x)
        - set threshold level 0
        - check RFSW_DIS0 and RFSW_DI1 - should be 1; LED_DIS0 LED_DIS1 ON
        - set threshold to expected value
        - set DET_RST 1 and realase it
        - check RFSW_DIS0 and RFSW_DI1 - should be 0; LED_DIS0 LED_DIS1 OFF
*/
pub use stm32h7xx_hal as hal;
use crate::hardware::devices::{Devices, Variants};
use super::SiLPA;
use miniconf::Miniconf;
use serde::Serialize;
use crate::hardware::ecp5::ECP5;
use crate::hardware::devices::max1329::{self, Max1329,adc, dac};

#[derive(Copy, Clone)]
pub struct TelemetryBuffer{
    adc: adc::AdcCode,
}

impl Default for TelemetryBuffer {
    fn default() -> Self {
        Self {
            adc: adc::AdcCode(0),
        }
    }
}

impl TelemetryBuffer{
    pub fn finalize(self) -> Telemetry{
        Telemetry{
            adc: self.adc.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Miniconf, PartialEq)]
pub struct Settings{
    adc_gt_threshold: u16,  // max 0xFFF
    adc_lt_threshold: u16,  // max 0xFFF
    dacs_enable     : [bool; 2],
    dacs_value      : [u16; 2],  // max 0xFFF
    pub telemetry_period: u16,
}

impl Default for Settings{
    fn default() -> Self {
        Self{
            adc_gt_threshold: 0xFFF,
            adc_lt_threshold: 0x000,
            dacs_enable     : [false, false],
            dacs_value      : [0x000, 0x000],
            telemetry_period: 10,
        }
    }
}


#[derive(Serialize)]
pub struct Telemetry{
    adc: u16
}

pub struct SilpaTester{}
impl Variants for SilpaTester {
    type VariantSettings = Settings;
    type VariantTelemetry = Telemetry;
    type VariantTelemetryBuffer = TelemetryBuffer;
}

impl Devices <Settings, Telemetry> for SiLPA<SilpaTester>
{
    fn init(&mut self, ecp5: &mut ECP5) -> bool {
        // TODO IO and switch control
        // Internal OSC, Disabled CLKIO out, ADC clock Divider = 1, Acquisition clocks 4 (G=1,2) or 8 (G=4, 8)
        Max1329::set_clock_control_register(self.slot, ecp5, 0b01000001);
        // Int active low, RST1 interrupt, charge-pump On 4V,  //TODO should be at silpa default?
        // CP clock divider: 64 (57 kHz with internal OSC, suggested between 39k - 78kHz)
        Max1329::set_cpvm_control_register(self.slot, ecp5, 0b01001001);
        // ADC Master Clock Cycles - 32 - For Internal OSC -> 115,2 ksps
        // Reference: disable REFADJ and internal REF ADC/DAC buffers (AJD -> REFADC, ADJ -> REFDAC),
        // apply external references directly at REFADC and REFDAC pins
        Max1329::set_adc_control_register(self.slot, ecp5, adc::AutoConversion::Disabled,
                                                           adc::PowerDownConf::FastPowerDown,
                                                           adc::RefConf::ExtBuffOff,);
        // Default Setup ADC input: Ain1, ADC gain 1, Unipolar mode (Default MUX SEL is 0) //TODO configure ADC
        // Max1329::set_adc_setup_direct()
        //Max1329::



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

        if self.settings.dacs_value[0] != new_settings.dacs_value[0] {
            Max1329::set_daca_value(self.slot, ecp5, new_settings.dacs_value[0]);
        }

        if self.settings.dacs_value[1] != new_settings.dacs_value[1] {
            Max1329::set_dacb_value(self.slot, ecp5, new_settings.dacs_value[1]);
        }

        self.settings = new_settings;
    }

    fn telemetry(&mut self, _ecp5: &mut ECP5) -> (Telemetry, u16) {
        (self.telemetry.finalize(),
         self.settings.telemetry_period)
    }

    fn check_interrupt(&mut self, ecp5: &mut ECP5) {
        let status : u32 = Max1329::read_status_register(self.slot, ecp5);

        if (status & max1329::GTA) != 0 {
            self.adc_gt_alarm(ecp5);
        }

        if (status & max1329::LTA) != 0 {
            self.adc_lt_alarm(ecp5);
        }

        if (status & max1329::ADD) != 0 {
            self.telemetry.adc = Max1329::read_adc_data_register(self.slot, ecp5);
        }
    }
}

impl SiLPA<SilpaTester>
{
    fn adc_gt_alarm(&self, _ecp5: &mut ECP5){

    }

    fn adc_lt_alarm(&self, _ecp5: &mut ECP5){

    }

    // pub fn silpa_test(
//     i2c: hal::i2c::I2c<hal::stm32::I2C4>,
//     servmod: _       ) {
//     test_i2c_cpcis(i2c, servmod);
// }


    fn test_eeprom(&self){
        let eeprom_addr: u8 = 0b1010_000;
    }

    fn test_temp_sensors(&self){
        let temp_addr = [0b1001_000, 0b1001_001];
    }

    fn test_i2c_cpcis(&mut self,
                i2c: hal::i2c::I2c<hal::stm32::I2C4>,
    ){//    servmod: _      ){
        //self.test_system_reset();
        self.test_eeprom();
        self.test_temp_sensors();
    }

    // fn check_hwrev(&mut self, ecp5: &mut ECP5):
}





