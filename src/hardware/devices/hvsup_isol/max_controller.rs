use crate::hardware::devices::max1329::*;
use crate::hardware::{ecp5, ecp5::ECP5};

#[derive(Clone, Copy)]
pub enum Channel {
    A,
    B,
}

pub struct MaxController {
    slot: u8,
}

impl MaxController {
    const U_MAX: f64 = 1500.0;
    const I_MAX: f64 = 15.0;
    const DAC_MAX: f64 = 0b111111111111 as f64;

    pub fn new(slot: u8) -> MaxController {
        MaxController { slot }
    }

    pub fn init(&self, ecp5: &mut ECP5) {
        Max1329::setup_ecp5_spi_master(1, ecp5, 1);
        self.switch_cs(Channel::A, ecp5);
        Max1329::set_apio_control_register(self.slot, ecp5, u8::MAX);

        for channel in [Channel::A, Channel::B] {
            self.switch_cs(channel, ecp5);

            // Turn HV output off
            self.switch_output(channel, false, ecp5);

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
            Max1329::set_dac_control(
                self.slot,
                ecp5,
                dac::PowerDownConf::InOut,
                dac::PowerDownConf::InOut,
                dac::OpAmp::Disable,
                dac::RefConf::Ext1_0,
            );

            // Disable autoconversion - only sample during telemetry
            // Reference: disable REFADJ and internal REF ADC/DAC buffers (AJD -> REFADC, ADJ -> REFDAC),
            // apply external references directly at REFADC and REFDAC pins
            Max1329::set_adc_control_register(
                self.slot,
                ecp5,
                adc::AutoConversion::Disabled,
                adc::PowerDownConf::Normal,
                adc::RefConf::ExtBuffOff,
            );
        }
    }

    pub fn switch_output(&self, channel: Channel, state: bool, ecp5: &mut ECP5) {
        self.switch_cs(channel, ecp5);
        let val: u8 = if state { 0xF0 } else { 0xFF };
        Max1329::set_dpio_setup_register(self.slot, ecp5, val);
    }

    pub fn set_voltage(&self, channel: Channel, voltage: u16, ecp5: &mut ECP5) {
        self.switch_cs(channel, ecp5);
        let new_u = voltage as f64;
        let u_control = (new_u / MaxController::U_MAX * MaxController::DAC_MAX) as u16;
        Max1329::set_daca_value(self.slot, ecp5, u_control);
    }

    pub fn set_current(&self, channel: Channel, current: u16, ecp5: &mut ECP5) {
        self.switch_cs(channel, ecp5);
        let new_i = current as f64;
        let i_control = (new_i / MaxController::I_MAX * MaxController::DAC_MAX) as u16;
        Max1329::set_dacb_value(self.slot, ecp5, i_control);
    }

    pub fn read_voltage(&self, channel: Channel, ecp5: &mut ECP5) -> f32 {
        self.switch_cs(channel, ecp5);
        Max1329::set_adc_setup_direct(
            self.slot,
            ecp5,
            adc::Mux::AIN1_AGND,
            adc::Gain::G1,
            adc::Bip::Unipolar,
        );
        while (Max1329::read_status_register(self.slot, ecp5) | (1 << 20)) == 0 {}
        let u_meas = Max1329::read_adc_data_register(self.slot, ecp5);
        u_meas.voltage(adc::Gain::G1, 2.5) * 600.0
    }

    pub fn read_current(&self, channel: Channel, ecp5: &mut ECP5) -> f32 {
        self.switch_cs(channel, ecp5);
        Max1329::set_adc_setup_direct(
            self.slot,
            ecp5,
            adc::Mux::AIN2_AGND,
            adc::Gain::G1,
            adc::Bip::Unipolar,
        );
        while (Max1329::read_status_register(self.slot, ecp5) | (1 << 20)) == 0 {}
        let i_meas = Max1329::read_adc_data_register(self.slot, ecp5);
        i_meas.voltage(adc::Gain::G1, 2.5) * 0.000004
    }

    fn wait_for_spi(&self, ecp5: &mut ECP5) {
        let mut data: [u8; 2] = [0; 2];

        let address = ecp5::OFFSET_TO_SLOT * self.slot + ecp5::OFFSET_TO_SPI + ecp5::SPI::IDLE;

        while {
            ecp5.read_from_ecp5(address, &mut data).unwrap();
            data[1] != 1
        } {}
    }

    fn switch_cs(&self, channel: Channel, ecp5: &mut ECP5) {
        let index = match channel {
            Channel::A => 0,
            Channel::B => 1,
        };

        self.wait_for_spi(ecp5);
        ecp5.set_spi_cs_pol(self.slot, index);
    }
}
