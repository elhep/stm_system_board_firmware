use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use miniconf::Miniconf;
use serde::Serialize;
use core::fmt::Debug;
use crate::hardware::devices::{Devices, Variants};
use crate::hardware::devices::nucleo_tester::Nucleo;
use crate::hardware::ecp5::ECP5;

#[derive(Clone, Copy, Debug, Miniconf, PartialEq)]
pub struct Settings{
    red_led: bool,  // max 0xFFF
    pub telemetry_period: u16,
}

impl Default for Settings{
    fn default() -> Self {
        Self{
            red_led: false,
            telemetry_period: 10,
        }
    }
}

#[derive(Serialize, Copy, Clone)]
pub struct Telemetry{
    input1: bool,
    input2: bool,
    telemetry_period: u16,
}

impl Default for Telemetry{
    fn default() -> Self {
        Self{
            input1: false,
            input2: false,
            telemetry_period: 10,
        }
    }
}

impl Telemetry {
    pub fn finalize(self) -> Telemetry{
        Telemetry{..self}
    }
}

pub struct Red{}
impl Variants for Red{
    type VariantSettings = Settings;
    type VariantTelemetry = Telemetry;
    type VariantTelemetryBuffer = Telemetry;
}

impl <U> Devices  <Settings, Telemetry> for Nucleo<Red, U>
where
    U: OutputPin,
    U::Error: Debug,
{
    fn init(&mut self, _ecp5: &mut ECP5) -> bool {true}

    fn settings_update(&mut self, _ecp5: &mut ECP5, _new_settings: Settings) -> () {
        if self.settings.red_led{
            self.output.set_high().unwrap();
        } else {
            self.output.set_low().unwrap();
        }
    }


    fn telemetry(&mut self, _ecp5: &mut ECP5) -> (Telemetry, u16) {
        (self.telemetry, 10)
    }

    fn check_interrupt(&mut self, _ecp5: &mut ECP5){}
}