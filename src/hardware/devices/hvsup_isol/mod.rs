use crate::hardware::devices::Devices;
use crate::hardware::setup::BusReference;
use miniconf::Miniconf;
use serde::Serialize;

use self::board_controller::{BoardController, IoPin};
use self::max_controller::{Channel, MaxController};

pub mod board_controller;
pub mod max_controller;

pub mod hvsupnegneg;
pub mod hvsupnegpos;
pub mod hvsupposneg;
pub mod hvsuppospos;

// New arch - https://play.rust-lang.org/?version=stable&mode=debug&edition=2021&gist=c8d62c0eb8ca0491012db151d96df473

#[derive(Serialize, Default)]
pub struct TelemetryBuffer {}

#[derive(Serialize, Default)]
pub struct Telemetry {
    channels: [HvChannelTelemetry; 2],
    temp: f32,
}

#[derive(Serialize, Default)]
pub struct HvChannelTelemetry {
    voltage: f32,
    current: f32,
}

#[derive(Clone, Copy, Debug, Miniconf, PartialEq)]
pub struct Settings {
    channels_settings: [HvChannelSettings; 2],
    master_mode: bool,
    hv_enable: bool,
    pub telemetry_period: u16,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            channels_settings: [HvChannelSettings::default(); 2],
            master_mode: true,
            hv_enable: false,
            telemetry_period: 250,
        }
    }
}

#[derive(Clone, Copy, Debug, Miniconf, PartialEq)]
pub struct HvChannelSettings {
    enable: bool,
    u_ctrl: u16,
    i_ctrl: u16,
}

impl Default for HvChannelSettings {
    fn default() -> Self {
        Self {
            enable: false,
            u_ctrl: 0,
            i_ctrl: 0,
        }
    }
}

#[derive(Copy, Clone)]
pub enum OutputVariant {
    Positive,
    Negative,
}

pub struct HvSupIsol {
    pub settings: Settings,
    bus: BusReference,
    interlock_high: bool,
    board_controller: BoardController,
    max_controller: MaxController,
    a_variant: OutputVariant,
    b_variant: OutputVariant,
}
impl HvSupIsol {
    pub fn new(slot_number: u8, bus: BusReference, a_variant: OutputVariant, b_variant: OutputVariant) -> Self {
        Self {
            settings: Settings::default(),
            bus,
            interlock_high: false,
            board_controller: BoardController::new(slot_number),
            max_controller: MaxController::new(slot_number),
            a_variant,
            b_variant,
        }
    }
}

impl Devices<Settings, Telemetry> for HvSupIsol {
    fn init(&mut self) -> bool {
        self.bus.lock(|bus| {
            let dev_name = self
                .board_controller
                .read_device_name(&mut bus.cpcis_i2c, &mut bus.servmod);
            let dev_name = match core::str::from_utf8(&dev_name) {
                Ok(name) => name,
                Err(_) => return false,
            };
            if dev_name != "HVSUP_ISOL" {
                log::info!("HVSUP wrong board name");
                return false;
            }
            log::info!("HVSUP correct board name");

            self.board_controller.switch_hv_enable(false, &mut bus.ecp5);
            self.board_controller.switch_psu_enable(true, &mut bus.ecp5);

            self.board_controller
                .enable_interrupt(IoPin::Interlock, &mut bus.ecp5);

            self.max_controller.init(&mut bus.ecp5);
            true
        })
    }

    fn settings_update(&mut self, new_settings: Settings) -> () {
        let hv_en_changed = self.settings.hv_enable != new_settings.hv_enable;
        let master_mode_changed = self.settings.master_mode != new_settings.master_mode;
        self.bus.lock(|bus| {
            if hv_en_changed || master_mode_changed {
                if new_settings.master_mode {
                    self.board_controller
                        .switch_hv_enable(new_settings.hv_enable, &mut bus.ecp5);
                } else {
                    self.board_controller.switch_hv_enable(
                        self.interlock_high && self.settings.hv_enable,
                        &mut bus.ecp5,
                    );
                }
            }

            for (i, channel) in [(0, Channel::A), (1, Channel::B)] {
                let channel_settings = &self.settings.channels_settings[i];
                let new_channel_settings = &new_settings.channels_settings[i];

                if channel_settings.enable != new_channel_settings.enable {
                    self.max_controller.switch_output(
                        channel,
                        new_channel_settings.enable,
                        &mut bus.ecp5,
                    );
                }

                if channel_settings.u_ctrl != new_channel_settings.u_ctrl {
                    self.max_controller.set_voltage(
                        channel,
                        new_channel_settings.u_ctrl,
                        &mut bus.ecp5,
                    );
                }

                if channel_settings.i_ctrl != new_channel_settings.i_ctrl {
                    self.max_controller.set_current(
                        channel,
                        new_channel_settings.i_ctrl,
                        &mut bus.ecp5,
                    );
                }
            }
        });
        self.settings = new_settings;
    }
    fn telemetry(&mut self) -> (Telemetry, u16) {
        let mut telemetry = Telemetry::default();
        self.bus.lock(|bus| {
            for (i, channel) in [(0, Channel::A), (1, Channel::B)] {
                telemetry.channels[i].voltage =
                    self.max_controller.read_voltage(channel, &mut bus.ecp5);
                telemetry.channels[i].current =
                    self.max_controller.read_current(channel, &mut bus.ecp5);
            }

            if matches!(self.a_variant, OutputVariant::Negative) {
                telemetry.channels[0].voltage *= -1.0;
                telemetry.channels[0].current *= -1.0;
            }

            if matches!(self.b_variant, OutputVariant::Negative) {
                telemetry.channels[1].voltage *= -1.0;
                telemetry.channels[1].current *= -1.0;
            }

            telemetry.temp = self
                .board_controller
                .read_temp(&mut bus.cpcis_i2c, &mut bus.servmod);
        });

        (telemetry, self.settings.telemetry_period)
    }

    fn check_interrupt(&mut self) {
        self.bus.lock(|bus| {
            self.interlock_high = self
                .board_controller
                .read_io(IoPin::Interlock, &mut bus.ecp5);

            // TODO(Adrian) - remove this log
            log::info!("HVSUP INT: interock = {}", self.interlock_high);

            if !self.settings.master_mode {
                let state = self.interlock_high && self.settings.hv_enable;
                self.board_controller.switch_hv_enable(state, &mut bus.ecp5);
            }
        });
    }
}
