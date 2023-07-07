use crate::hardware::devices::Variants;
use miniconf::Miniconf;
use crate::hardware::devices::max1329::adc::{self, AdcCode};
use serde::Serialize;

pub mod hvsuppospos;
pub mod hvsupnegneg;
pub mod hvsupposneg;
pub mod hvsupnegpos;


#[derive(Copy, Clone)]
pub struct TelemetryBuffer{
    /// The latest input samples of AIN1 on both MAX( U_MEAS CH1 / U_MEAS CH2).
    u_meas: [AdcCode; 2],
    /// The latest input samples
    /// of AIN2 on both MAX( I_MEAS CH1 / I_MEAS CH2).
    i_meas: [AdcCode; 2],
    /// Current state of ADCs input Multiplexers (MAX1 / MAX2).
    current_meas: [adc::Mux; 2],
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
    pub fn finalize(self, output_variants: [OutputVariant; 2]) -> Telemetry{
        Telemetry {
            channels: [HvChannelTelemetry::new(output_variants[0], self.u_meas[0], self.i_meas[0]),
                       HvChannelTelemetry::new(output_variants[1], self.u_meas[1], self.i_meas[1])]
        }
    }
}


#[derive(Serialize)]
pub struct Telemetry {
    channels: [HvChannelTelemetry; 2],
}

#[derive(Serialize)]
pub struct HvChannelTelemetry{
    voltage_v: f32,
    voltage_b: u16,
    current_a: f32,
    current_b: u16
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
                   current_b: i_meas.0
               }
           }
           OutputVariant::Negative => {
               HvChannelTelemetry {
                   voltage_v: -voltage,
                   voltage_b: u_meas.0,
                   current_a: -current,
                   current_b: i_meas.0
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
            telemetry_period: 1,
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
}

impl <T> HVSUP_ISOL <T>
where
    T: Variants,
{
    pub fn new(
        slot_number : u8,
    ) -> Self
    {
        Self{
            slot: slot_number,
            settings: T::VariantSettings::default(),
            telemetry: T::VariantTelemetryBuffer::default(),
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
            ecp5.write_outputs(1, &[0, 0b0011_0000]); // enable hv
            //Max1329::set_dpio_setup_register(self.slot, ecp5, 0xF0);
        } else {
            ecp5.write_outputs(1, &[0, 0b0000_0000]); // enable hv
            //Max1329::set_dpio_setup_register(self.slot, ecp5, 0xFF);
        }
    }
}

impl Devices<Settings, Telemetry> for HVSUP_ISOL<$variant>{
    fn init(&mut self, ecp5: &mut ECP5) -> bool {
        // First device APIO (SPI extender).
        // TODO check APIO SETUP during tests - should work without changes
        Max1329::set_apio_control_register(self.slot, ecp5, u8::MAX);
        for i in 0..2{
            // Change CS pol for second Max
            if i == 1{
                ecp5.set_spi_cs_pol(self.slot, 1);
            }

            // Internal OSC, Disabled CLKIO out, ADC clock Divider = 1, Acquisition clocks 4 (G=1,2) or 8 (G=4, 8)
            Max1329::set_clock_control_register(self.slot, ecp5, 0b01000001);
            // Int active high (because of MOSFET), RST1 interrupt, charge-pump On 3V,
            // CP clock divider: 64 (57 kHz with internal OSC, suggested between 39k - 78kHz)
            Max1329::set_cpvm_control_register(self.slot, ecp5, 0b11000101);
            // Set pullup for all inputs and logic high for outputs (low would enable HV)
            self.switch_hv_output(ecp5, false);
            // DPIO1 output, DPIO2-4 input.
            Max1329::set_dpio_control_register(self.slot, ecp5, 0x000F);
            // Enable ADC Data Ready interrupt
            Max1329::set_interrupt_mask_register(self.slot, ecp5, !(max1329::ADD));

            // Enable DACs
            Max1329::set_dac_control(self.slot, ecp5, dac::PowerDownConf::InToOut,
                                                      dac::PowerDownConf::InToOut,
                                                      dac::OpAmp::Disable,
                                                      dac::RefConf::Ext1_0);

            // ADC Master Clock Cycles - 32 - For Internal OSC -> 115,2 ksps
            // Reference: disable REFADJ and internal REF ADC/DAC buffers (AJD -> REFADC, ADJ -> REFDAC),
            // apply external references directly at REFADC and REFDAC pins
            Max1329::set_adc_control_register(self.slot, ecp5, adc::AutoConversion::Clk32,
                                                               adc::PowerDownConf::Normal,
                                                               adc::RefConf::ExtBuffOff,);
            // Default Setup ADC input: Ain1, ADC gain 1, Unipolar mode (Default MUX SEL is 0)
            //TODO configure ADC
            // Max1329::set_adc_setup_direct()


            // Change CS pol for first Max again (default for idle)
            if i == 1{
                ecp5.set_spi_cs_pol(self.slot, 0);
            }
        }

        true
    }

    fn settings_update(&mut self, ecp5: &mut ECP5, new_settings: Settings) -> () {
        for i in 0..1{ // TODO change for loop 0..2 after tests
            log::info!("Settings loop");
            // Change CS pol for second Max
            if i == 1{
                ecp5.set_spi_cs_pol(self.slot, 1);
            }

            if self.settings.channels_settings[i].enable != new_settings.channels_settings[i].enable{
                self.switch_hv_output(ecp5, new_settings.channels_settings[i].enable);
            }

            // Change DACA value ( U )
            if self.settings.channels_settings[i].u_ctrl != new_settings.channels_settings[i].u_ctrl{ // TODO change slot number 1 to self.slot
                Max1329::set_daca_value(1, ecp5, new_settings.channels_settings[i].u_ctrl);
            }

            // Change DACB value ( I )
            if self.settings.channels_settings[i].i_ctrl != new_settings.channels_settings[i].i_ctrl{
                Max1329::set_dacb_value(1, ecp5, new_settings.channels_settings[i].i_ctrl);
            }

            // Change CS pol to first Max again (default for idle)
            if i == 1{
                ecp5.set_spi_cs_pol(self.slot, 0);
            }
            // ADC Gain will be changed when receiveng next sample
        }
        self.settings = new_settings;
        log::info!("Exit Settings");
    }

    fn telemetry(&mut self, ecp5: &mut ECP5) -> (Telemetry, u16) {
            Max1329::set_adc_setup_direct(1, ecp5, max1329::adc::Mux::AIN1_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);
            while (Max1329::read_status_register(1, ecp5) | (1 << 20)) == 0 {
                log::info!("W8 for ADC");
            }
            let x = Max1329::read_adc_data_register(1, ecp5);
            self.telemetry.u_meas[0] = AdcCode(x.0);

            Max1329::set_adc_setup_direct(1,
            ecp5, max1329::adc::Mux::AIN2_AGND, max1329::adc::Gain::G1, max1329::adc::Bip::Unipolar);

            let x = Max1329::read_adc_data_register(1, ecp5);
            self.telemetry.i_meas[0] = AdcCode(x.0);

            (self.telemetry.finalize(hvsup_telemetry!($variant)),
             self.settings.telemetry_period)

    }
    fn check_interrupt(&mut self, ecp5: &mut ECP5){
        for i in 0..2{
            // Change CS pol for second Max
            if i == 1{
                ecp5.set_spi_cs_pol(self.slot, 1);
            }

            let status : u32 = Max1329::read_status_register(self.slot, ecp5);

            if (status & max1329::ADD) != 0 {
                    self.read_adc_data(ecp5, i);
            }

            // Change CS pol for first Max again (default for idle)
            if i == 1{
                ecp5.set_spi_cs_pol(self.slot, 0);
            }
        }
    }
}
    }};
 }

#[derive(Copy, Clone)]
pub enum OutputVariant{
    Positive,
    Negative,
}

