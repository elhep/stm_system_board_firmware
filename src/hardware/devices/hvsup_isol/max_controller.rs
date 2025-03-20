use minimq::embedded_time::Clock;
use stm32h7xx_hal::device::bdma::ch;

use super::timer::Timer;
use super::{CalibrateData, CalibrateEntry};
use crate::hardware::devices::max1329::*;
use crate::hardware::{ecp5, ecp5::ECP5};
use mono_clock::embedded_time::{duration::Milliseconds, Instant};

#[derive(Clone, Copy, Debug)]
pub enum Channel {
    A,
    B,
}

impl Channel {
    pub fn get_all() -> [Channel; 2] {
        [Channel::A, Channel::B]
    }
}

#[derive(Clone, Copy)]
struct ChannelSettings {
    current_voltage: f32,
    target_voltage: f32,
    step: f32,
    delay: Milliseconds,
    last_update: Instant<Timer>,
    on_since: Instant<Timer>,
    on: bool,
}

impl Default for ChannelSettings {
    fn default() -> Self {
        ChannelSettings {
            current_voltage: 0.0,
            target_voltage: 0.0,
            step: 0.0,
            delay: Milliseconds::new(0),
            last_update: Instant::new(0),
            on_since: Instant::new(0),
            on: false,
        }
    }
}

#[derive(Clone, Copy)]
pub struct Coefs {
    a_coef: f32,
    b_coef: f32,
}

impl Default for Coefs {
    fn default() -> Self {
        Coefs {
            a_coef: 1.0,
            b_coef: 0.0,
        }
    }
}

impl Coefs {
    pub fn serialize(&self) -> [u8; 8] {
        let a = f32::to_be_bytes(self.a_coef);
        let b = f32::to_be_bytes(self.b_coef);
        let mut arr = [0u8; 8];
        for i in 0..4 {
            arr[i] = a[i];
            arr[i + 4] = b[i];
        }
        arr
    }

    pub fn deserialize(data: [u8; 8]) -> Coefs {
        let mut coefs = Coefs::default();
        let mut tmp = [0u8; 4];
        tmp.clone_from_slice(&data[0..4]);
        coefs.a_coef = f32::from_be_bytes(tmp);
        tmp.clone_from_slice(&data[4..8]);
        coefs.b_coef = f32::from_be_bytes(tmp);
        coefs
    }
}

#[derive(Default, Clone, Copy)]
struct Calibration {
    v_dac: Coefs,
    v_adc: Coefs,
}

pub struct MaxController {
    slot: u8,
    timer: Timer,
    channel_settings: [ChannelSettings; 2],
    calibration: [Calibration; 2],
}

impl MaxController {
    pub const U_MAX: f32 = 1500.0;
    pub const I_MAX: f32 = 15.0;
    const DAC_MAX: f32 = 0b111111111111 as f32;
    const UPDATE_PERIOD: u32 = 100;

    pub fn new(slot: u8) -> MaxController {
        MaxController {
            slot,
            timer: Timer::new(),
            channel_settings: [ChannelSettings::default(); 2],
            calibration: [Calibration::default(); 2],
        }
    }

    pub fn init(&mut self, ecp5: &mut ECP5) {
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

    pub fn reset_calibration(&mut self, channel: Channel) {
        self.calibration[channel as usize] = Calibration::default();
    }

    pub fn calibrate(
        &mut self,
        channel: Channel,
        data: [CalibrateEntry; CalibrateData::POINTS_NUM],
    ) {
        // User measurement is source of truth and maps to ys

        let mut user_mean = 0.0 as f32;
        let mut dac_mean = 0.0 as f32;
        let mut adc_mean = 0.0 as f32;
        for point in &data {
            user_mean += point.user_read;
            dac_mean += point.expected as f32;
            adc_mean += point.adc_read;
        }

        user_mean = user_mean / CalibrateData::POINTS_NUM as f32;
        dac_mean = dac_mean / CalibrateData::POINTS_NUM as f32;
        adc_mean = adc_mean / CalibrateData::POINTS_NUM as f32;

        let mut sum_dac = 0.0 as f32;
        let mut sum_dac_x = 0.0 as f32;
        let mut sum_adc = 0.0 as f32;
        let mut sum_adc_x = 0.0 as f32;
        for point in &data {
            sum_dac += (point.expected as f32 - dac_mean) * (point.user_read - user_mean);
            sum_dac_x += (point.expected as f32 - dac_mean) * (point.expected as f32 - dac_mean);
            sum_adc += (point.adc_read - adc_mean) * (point.user_read - user_mean);
            sum_adc_x += (point.adc_read - adc_mean) * (point.adc_read - adc_mean);
        }

        let a_dac = sum_dac / sum_dac_x;
        let b_dac = user_mean - (a_dac * dac_mean);

        let a_adc = sum_adc / sum_adc_x;
        let b_adc = user_mean - (a_adc * adc_mean);

        let b_dac = -b_dac / a_dac;
        let a_dac = 1.0 / a_dac;
        let b_adc = -b_adc / a_adc;
        let a_adc = 1.0 / a_adc;

        let ch = match channel {
            Channel::A => "CH_A",
            Channel::B => "CH_B",
        };

        log::info!("Calibrating channel {}", ch);
        log::info!("DAC a: {}", a_dac);
        log::info!("DAC b: {}", b_dac);
        log::info!("ADC a: {}", a_adc);
        log::info!("DAC b: {}", b_adc);

        let calib = &mut self.calibration[channel as usize];
        calib.v_dac.a_coef = a_dac;
        calib.v_dac.b_coef = b_dac;
        calib.v_adc.a_coef = a_adc;
        calib.v_adc.b_coef = b_adc;
    }

    pub fn switch_output(&mut self, channel: Channel, state: bool, ecp5: &mut ECP5) {
        self.switch_cs(channel, ecp5);
        let val: u8 = if state { 0xF0 } else { 0xFF };
        Max1329::set_dpio_setup_register(self.slot, ecp5, val);

        let settings = &mut self.channel_settings[channel as usize];

        if state && !settings.on {
            settings.on = true;
            settings.last_update = self.timer.try_now().unwrap();
            settings.on_since = self.timer.try_now().unwrap();
        } else if !state {
            settings.on = false;
            settings.current_voltage = 0.0;
            self.set_voltage(channel, 0.0, ecp5);
        }
    }

    pub fn set_target_voltage(
        &mut self,
        channel: Channel,
        voltage: u16,
        step: f32,
        delay: Milliseconds,
    ) {
        let settings = &mut self.channel_settings[channel as usize];
        settings.target_voltage = voltage as f32;
        settings.step = step;
        settings.delay = delay;
    }

    pub fn set_current(&self, channel: Channel, current: f32, ecp5: &mut ECP5) {
        self.switch_cs(channel, ecp5);
        let i_control = (current / MaxController::I_MAX * MaxController::DAC_MAX) as u16;
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

        // No need to calibrate ADC because of small errors
        let _ = &self.calibration[channel as usize].v_adc;
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

    pub fn update_voltage(&mut self, ecp5: &mut ECP5) -> u32 {
        const EPS: f32 = 0.1;
        self.timer.update(MaxController::UPDATE_PERIOD);
        let now = self.timer.try_now().unwrap();
        for channel in Channel::get_all() {
            let i = channel as usize;
            let settings = &mut self.channel_settings[i];

            // let on = settings.on;
            // let target_vol = settings.target_voltage;
            // let set_delay = settings.delay;
            // let set_step = settings.step;
            // log::info!("----------------");
            // log::info!("Channel: {i}");
            // log::info!("On: {on}");
            // log::info!("Target voltage: {target_vol}");
            // log::info!("Set delay: {set_delay}");

            let delay = Milliseconds::<u32>::try_from(now - settings.on_since).unwrap();
            // log::info!("Delay: {delay}");

            if !settings.on
                || abs(settings.current_voltage - settings.target_voltage) < EPS
                || delay < settings.delay
            {
                settings.last_update = now;
                continue;
            }

            let elapsed = Milliseconds::<u32>::try_from(now - settings.last_update)
                .unwrap()
                .0 as f32;
            // log::info!("Set step: {set_step}");
            let step = elapsed * settings.step / 1000.0;
            // log::info!("Step: {step}");

            let new_voltage = if settings.current_voltage < settings.target_voltage {
                match settings.current_voltage + step {
                    new @ _ if new > settings.target_voltage => settings.target_voltage,
                    new @ _ => new,
                }
            } else {
                match settings.current_voltage - step {
                    new @ _ if new < settings.target_voltage => settings.target_voltage,
                    new @ _ => new,
                }
            };

            settings.current_voltage = new_voltage;
            settings.last_update = now;

            // log::info!("New voltage: {new_voltage}");
            // log::info!("**");

            self.set_voltage(channel, new_voltage, ecp5);
        }
        // log::info!("----------------");

        MaxController::UPDATE_PERIOD
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
        let index = channel as u8;
        self.wait_for_spi(ecp5);
        ecp5.set_spi_cs_pol(self.slot, index);
    }

    fn set_voltage(&self, channel: Channel, voltage: f32, ecp5: &mut ECP5) {
        self.switch_cs(channel, ecp5);
        let calib = &self.calibration[channel as usize].v_dac;
        let new_u = voltage as f32 * calib.a_coef + calib.b_coef;

        let u_control = (new_u / MaxController::U_MAX * MaxController::DAC_MAX) as u16;
        Max1329::set_daca_value(self.slot, ecp5, u_control);
    }

    pub fn get_coefs(&self) -> [Coefs; 2] {
        [self.calibration[0].v_dac, self.calibration[1].v_dac]
    }

    pub fn set_coefs(&mut self, coefs: [Coefs; 2]) {
        log::info!("Setting calib V_DAC for A a: {}", coefs[0].a_coef);
        log::info!("Setting calib V_DAC for A b: {}", coefs[0].b_coef);
        log::info!("Setting calib V_DAC for B a: {}", coefs[1].a_coef);
        log::info!("Setting calib V_DAC for B b: {}", coefs[1].b_coef);
        self.calibration[0].v_dac = coefs[0];
        self.calibration[1].v_dac = coefs[1];
    }
}

fn abs(num: f32) -> f32 {
    if num >= 0.0 {
        num
    } else {
        -num
    }
}
