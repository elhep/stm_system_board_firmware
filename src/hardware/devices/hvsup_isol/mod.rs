use crate::hardware::devices::Variants;
use miniconf::Miniconf;
use crate::hardware::devices::max1329::adc::{self, AdcCode};
use serde::Serialize;
use stm32h7xx_hal as hal;
use crate::hardware::ServMod;
use crate::hardware::lm75a;

pub mod hvsuppospos;
pub mod hvsupnegneg;
pub mod hvsupposneg;
pub mod hvsupnegpos;

type Cpcis_I2C = hal::i2c::I2c<hal::stm32::I2C4>;

#[derive(Copy, Clone)]
pub struct TelemetryBuffer{
    /// The latest input samples of AIN1 on both MAX( U_MEAS CH1 / U_MEAS CH2).
    u_meas: [AdcCode; 2],
    /// The latest input samples
    /// of AIN2 on both MAX( I_MEAS CH1 / I_MEAS CH2).
    i_meas: [AdcCode; 2],
    /// Current state of ADCs input Multiplexers (MAX1 / MAX2).
    current_meas: [adc::Mux; 2],
    // Current temperature
}

impl Default for TelemetryBuffer{
    fn default() -> Self {
        Self {
            u_meas: [AdcCode(0), AdcCode(0)],
            i_meas: [AdcCode(0), AdcCode(0)],
            current_meas: [adc::Mux::AIN1_AGND, adc::Mux::AIN1_AGND],
        }
    }
}


impl TelemetryBuffer{
    /// Convert ADC code to Si-unit for telemetry reporting
    ///
    /// # Args
    /// * `outputs_variant` - OutputVariant of both channels (based on PCB Variant)
    ///
    /// # Returns
    /// The finalized telemetry structure that can be serialized and reported.
    pub fn finalize(self, output_variants: [OutputVariant; 2], temp: f32) -> Telemetry{
        Telemetry {
            channels: [HvChannelTelemetry::new(output_variants[0], self.u_meas[0], self.i_meas[0]),
                       HvChannelTelemetry::new(output_variants[1], self.u_meas[1], self.i_meas[1])],
            temp
        }
    }
}


#[derive(Serialize)]
pub struct Telemetry {
    channels: [HvChannelTelemetry; 2],
    temp: f32,
}

#[derive(Serialize)]
pub struct HvChannelTelemetry{
    voltage_v: f32,
    voltage_b: u16,
    current_a: f32,
    current_b: u16,
}

impl HvChannelTelemetry{
    pub fn new(output_variant: OutputVariant, u_meas: AdcCode, i_meas: AdcCode) -> Self {
        let voltage = u_meas.voltage(adc::Gain::G1, 2.5) * 600.0;
        let current = i_meas.voltage(adc::Gain::G1, 2.5) * 0.000004;
        match output_variant{
           OutputVariant::Positive => {
               HvChannelTelemetry {
                   voltage_v: voltage,
                   voltage_b: u_meas.0,
                   current_a: current,
                   current_b: i_meas.0,
               }
           }
           OutputVariant::Negative => {
               HvChannelTelemetry {
                   voltage_v: -voltage,
                   voltage_b: u_meas.0,
                   current_a: -current,
                   current_b: i_meas.0,
                }
            }
        }
    }
}

#[derive(Clone, Copy, Debug, Miniconf, PartialEq)]
pub struct Settings{
    channels_settings: [HvChannelSettings; 2],
    pub telemetry_period: u16,
}

impl Default for Settings{
    fn default() -> Self {
        Self {
            channels_settings: [HvChannelSettings::default(); 2],
            telemetry_period: 250,
        }
    }
}

impl Settings{
    // pub fn get_gains(self) -> [adc::Gain; 4] {
    //     [self.channels_settings[0].u_gain, self.channels_settings[0].i_gain,
    //      self.channels_settings[1].u_gain, self.channels_settings[1].i_gain]
    // }
}

#[derive(Clone, Copy, Debug, Miniconf, PartialEq)]
pub struct HvChannelSettings{
    enable: bool,
    u_ctrl: u16,
    i_ctrl: u16,
}

impl Default for HvChannelSettings{
    fn default() -> Self{
        Self{
            enable: false,
            u_ctrl: 0,
            i_ctrl: 0,
        }
    }
}


pub struct HVSUP_ISOL<T: Variants>
{
    slot: u8,
    pub settings: T::VariantSettings,
    pub telemetry: T::VariantTelemetryBuffer,
    cpcis_i2c: Cpcis_I2C,
    servmod: ServMod,
}

impl <T> HVSUP_ISOL <T>
where
    T: Variants,
{
    pub fn new(
        slot_number : u8,
        cpcis_i2c : Cpcis_I2C,
        servmod : ServMod,
    ) -> Self
    {
        Self{
            slot: 1,
            settings: T::VariantSettings::default(),
            telemetry: T::VariantTelemetryBuffer::default(),
            cpcis_i2c,
            servmod
        }
    }
}

#[macro_export]
macro_rules! hvsup_telemetry {
    (HvSupPosPos) => {
        paste::paste!{
            [OutputVariant::Positive, OutputVariant::Positive]
        }
    };
    (HvSupNegNeg) => {
        paste::paste!{
            [OutputVariant::Negative, OutputVariant::Negative]
        }
    };
    (HvSupPosNeg) => {
        paste::paste!{
            [OutputVariant::Positive, OutputVariant::Negative]
        }
    };
    (HvSupNegPos) => {
        paste::paste!{
            [OutputVariant::Negative, OutputVariant::Positive]
        }
    };
}

#[macro_export]
 macro_rules! hvsup_devices_trait {
    ($variant:ident) => {
        paste::paste!{

impl HVSUP_ISOL<$variant>
{
    const THERM_ADDRESS: u8 = 0x48;
    /// Read new ADC sample and change state of the MUX
    fn read_adc_data(&mut self, ecp5: &mut ECP5, max_nr: usize){
        if self.telemetry.current_meas[max_nr] == adc::Mux::AIN1_AGND{
            self.telemetry.u_meas[max_nr] = Max1329::read_adc_data_register(self.slot, ecp5);
            Max1329::set_adc_setup_direct(self.slot, ecp5,
                                            adc::Mux::AIN2_AGND,
                                            adc::Gain::G1,
                                            adc::Bip::Unipolar);
            self.telemetry.current_meas[max_nr] = adc::Mux::AIN2_AGND;
        } else {
            self.telemetry.i_meas[max_nr] = Max1329::read_adc_data_register(self.slot, ecp5);
            Max1329::set_adc_setup_direct(self.slot, ecp5,
                                            adc::Mux::AIN1_AGND,
                                            adc::Gain::G1,
                                            adc::Bip::Unipolar);
            self.telemetry.current_meas[max_nr] = adc::Mux::AIN1_AGND;
        }
    }

    /// Switch HV Output On/Off.
    /// Change CS POL for correct MAX1329 before use switch HV output function!
    /// True - switch ON
    /// False - switch OFF
    fn switch_hv_output(&self, ecp5: &mut ECP5, state: bool){
        if state {
            ecp5.write_outputs(self.slot, &[0, 0b0011_0000]);
            log::info!("IO switch ON");
            // TODO(Adrian) - enable HV output with MAX
            Max1329::set_dpio_setup_register(self.slot, ecp5, 0xF0);
        } else {
            ecp5.write_outputs(self.slot, &[0, 0b0000_0000]);
            log::info!("IO switch OFF");
            // TODO(Adrian) - enable HV output with MAX
            Max1329::set_dpio_setup_register(self.slot, ecp5, 0xFF);
        }
    }

    fn wait_for_spi(&self, ecp5: &mut ECP5) {
        let mut data: [u8; 2] = [0; 2];
        
        let address = ecp5::OFFSET_TO_SLOT * self.slot + ecp5::OFFSET_TO_SPI + ecp5::SPI::IDLE;
        ecp5.read_from_ecp5(address, &mut data).unwrap();
        while data[1] != 1 {
            log::info!("HVSUP waiting for SPI");
            ecp5.read_from_ecp5(address, &mut data).unwrap();
        }
    }

    fn read_temp(&mut self) -> f32 {
        self.switch_servmod(true);
        let ret = match lm75a::read_temp(&mut self.cpcis_i2c, HVSUP_ISOL::THERM_ADDRESS) {
            Ok(val) => val,
            Err(_) => {
                log::info!("HVSUP failed to read temp!");
                0.0
            }
        };
        self.switch_servmod(false);
        ret
    }

    fn switch_servmod(&mut self, on: bool) {
        self.servmod.0.set_low().unwrap();
        self.servmod.1.set_low().unwrap();
        self.servmod.2.set_low().unwrap();
        self.servmod.3.set_low().unwrap();
        self.servmod.4.set_low().unwrap();
        self.servmod.5.set_low().unwrap();
        self.servmod.6.set_low().unwrap();
        self.servmod.7.set_low().unwrap();

        if (!on) {
            return
        }

        if (self.slot == 1) {
            self.servmod.0.set_high().unwrap();
        } else if (self.slot == 2) {
            self.servmod.1.set_high().unwrap();
        } else if (self.slot == 3) {
            self.servmod.2.set_high().unwrap();
        } else if (self.slot == 4) {
            self.servmod.3.set_high().unwrap();
        } else if (self.slot == 5) {
            self.servmod.4.set_high().unwrap();
        } else if (self.slot == 6) {
            self.servmod.5.set_high().unwrap();
        } else if (self.slot == 7) {
            self.servmod.6.set_high().unwrap();
        } else if (self.slot == 8) {
            self.servmod.7.set_high().unwrap();
        }
    }
}

impl Devices<Settings, Telemetry> for HVSUP_ISOL<$variant>{
    fn init(&mut self, ecp5: &mut ECP5) -> bool {
        // Configure firts MAX1329 APIO as SPI extender
        ecp5.set_spi_cs_pol(self.slot, 0);
        self.wait_for_spi(ecp5);
        ecp5.set_spi_cs_pol(self.slot, 0);
        Max1329::set_apio_control_register(self.slot, ecp5, u8::MAX);
        for i in 0..2 {
            if i == 1 {
                self.wait_for_spi(ecp5);
                ecp5.set_spi_cs_pol(self.slot, 1);
            }

            // Turn HV output off
            self.switch_hv_output(ecp5, false);

            // Configure MAX1329 clocks
            // Internal OSC, Disabled CLKIO out, ADC clock Divider = 1, Acquisition clocks 4 (G=1,2) or 8 (G=4, 8)
            Max1329::set_clock_control_register(self.slot, ecp5, 0b01000001);
            
            // Configure MAX1329 charge pump and voltage monitoring
            // Int active high (because of MOSFET), RST1 interrupt, charge-pump On 3V,
            // CP clock divider: 64 (57 kHz with internal OSC, suggested between 39k - 78kHz)
            Max1329::set_cpvm_control_register(self.slot, ecp5, 0b11000101);

            // Configure MAX1329 DPIO pins
            // DPIO1 output
            Max1329::set_dpio_control_register(self.slot, ecp5, 0x000F);
            
            // TODO(Adrian) - decide if we need interrupts
            // Max1329::set_interrupt_mask_register(self.slot, ecp5, !(max1329::ADD));

            // Enable MAX1329 DACs
            Max1329::set_dac_control(self.slot, ecp5, dac::PowerDownConf::InOut,
                                                      dac::PowerDownConf::InOut,
                                                      dac::OpAmp::Disable,
                                                      dac::RefConf::Ext1_0);

            // Disable autoconversion - only sample during telemetry
            // Reference: disable REFADJ and internal REF ADC/DAC buffers (AJD -> REFADC, ADJ -> REFDAC),
            // apply external references directly at REFADC and REFDAC pins
            Max1329::set_adc_control_register(self.slot, ecp5, adc::AutoConversion::Disabled,
                                                               adc::PowerDownConf::Normal,
                                                               adc::RefConf::ExtBuffOff,);

            // Change CS pol for first Max again (default for idle)
            if i == 1{
                self.wait_for_spi(ecp5);
                ecp5.set_spi_cs_pol(self.slot, 0);
            }
        }
        // TODO - remove
        // REG TEST
        self.wait_for_spi(ecp5);
        ecp5.set_spi_cs_pol(self.slot, 0);
        Max1329::set_interrupt_mask_register(self.slot, ecp5, !(max1329::ADD));
        self.wait_for_spi(ecp5);
        ecp5.set_spi_cs_pol(self.slot, 1);
        Max1329::set_interrupt_mask_register(self.slot, ecp5, !(max1329::AFF));
        self.wait_for_spi(ecp5);
        ecp5.set_spi_cs_pol(self.slot, 0);
        let max1 = Max1329::read_interrrupt_mask_register(self.slot, ecp5);
        self.wait_for_spi(ecp5);
        ecp5.set_spi_cs_pol(self.slot, 1);
        let max2 = Max1329::read_interrrupt_mask_register(self.slot, ecp5);
        self.wait_for_spi(ecp5);
        ecp5.set_spi_cs_pol(self.slot, 0);
        log::info!("Max1 int flags: {}-{}-{}", max1[0], max1[1], max1[2]);
        log::info!("Max2 int flags: {}-{}-{}", max2[0], max2[1], max2[2]);

        true
    }

    fn settings_update(&mut self, ecp5: &mut ECP5, new_settings: Settings) -> () {
        self.wait_for_spi(ecp5);
        ecp5.set_spi_cs_pol(self.slot, 0);
        for i in 0..2 {
            if i == 1 {
                self.wait_for_spi(ecp5);
                ecp5.set_spi_cs_pol(self.slot, 1);
            }

            if self.settings.channels_settings[i].enable != new_settings.channels_settings[i].enable {
                self.switch_hv_output(ecp5, new_settings.channels_settings[i].enable);
            }

            // Change DACA value ( U )
            if self.settings.channels_settings[i].u_ctrl != new_settings.channels_settings[i].u_ctrl {
                const U_MAX: f64 = 1500.0;
                const DAC_MAX: f64 = 0b111111111111 as f64;
                let new_u = new_settings.channels_settings[i].u_ctrl as f64;
                let u_control = (new_u / U_MAX * DAC_MAX) as u16;
                log::info!("Setting DAC-A to: {}", u_control);

                Max1329::set_daca_value(self.slot, ecp5, u_control);
            }

            // Change DACB value ( I )
            if self.settings.channels_settings[i].i_ctrl != new_settings.channels_settings[i].i_ctrl{
                const I_MAX: f64 = 15.0;
                const DAC_MAX: f64 = 0b111111111111 as f64;
                let new_i = new_settings.channels_settings[i].i_ctrl as f64;
                let i_control = (new_i / I_MAX * DAC_MAX) as u16;
                log::info!("Setting DAC-B to: {}", i_control);
                Max1329::set_dacb_value(self.slot, ecp5, i_control);
            }

            // Change CS pol to first Max again (default for idle)
            if i == 1 {
                self.wait_for_spi(ecp5);
                ecp5.set_spi_cs_pol(self.slot, 0);
            }
            // ADC Gain will be changed when receiveng next sample
        }
        self.settings = new_settings;
    }

    fn telemetry(&mut self, ecp5: &mut ECP5) -> (Telemetry, u16) {
        self.wait_for_spi(ecp5);
        ecp5.set_spi_cs_pol(self.slot, 0);
        for i in 0..2 {
            if i == 1 {
                self.wait_for_spi(ecp5);
                ecp5.set_spi_cs_pol(self.slot, 1);
            }

            Max1329::set_adc_setup_direct(self.slot, ecp5, max1329::adc::Mux::AIN1_AGND,
                                          max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
            while (Max1329::read_status_register(self.slot, ecp5) | (1 << 20)) == 0 {
                log::info!("HVSUP waiting for ADC...");
            }
            self.telemetry.u_meas[i] = Max1329::read_adc_data_register(self.slot, ecp5);

            Max1329::set_adc_setup_direct(self.slot, ecp5, max1329::adc::Mux::AIN2_AGND,
                                          max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
            while (Max1329::read_status_register(self.slot, ecp5) | (1 << 20)) == 0 {
                log::info!("HVSUP waiting for ADC...");
            }
            self.telemetry.i_meas[i] = Max1329::read_adc_data_register(self.slot, ecp5);

            if i == 1 {
                self.wait_for_spi(ecp5);
                ecp5.set_spi_cs_pol(self.slot, 0);
            }
        }
        let temp = self.read_temp();

            (self.telemetry.finalize(hvsup_telemetry!($variant), temp),
             self.settings.telemetry_period)

    }

    fn check_interrupt(&mut self, _ecp5: &mut ECP5) {
        // Empty - don't need interrupts
    }
}
    }};
 }

#[derive(Copy, Clone)]
pub enum OutputVariant{
    Positive,
    Negative,
}

