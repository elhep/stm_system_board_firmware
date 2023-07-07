use crate::hardware::devices::{Devices, Variants};
use super::SiLPA;
use miniconf::Miniconf;
use serde::Serialize;
use crate::hardware::ecp5::ECP5;

#[derive(Copy, Clone)]
pub struct TelemetryBuffer{
    dac: [f32; 2],
}

impl Default for TelemetryBuffer {
    fn default() -> Self {
        Self {
            dac: [0.0, 1.0],
        }
    }
}

impl TelemetryBuffer{
    pub fn finalize(self) -> Telemetry{
        Telemetry{
            dac: [self.dac[0].into(), self.dac[1].into()],
        }
    }
}

#[derive(Serialize)]
pub struct Telemetry{
    dac: [f32; 2]
}


#[derive(Clone, Copy, Debug, Miniconf, PartialEq)]
pub struct Settings{
    adc_lt_threshold: u16,  // max 0xFFF
}

impl Default for Settings{
    fn default() -> Self {
        Self{
            adc_lt_threshold: 0x000,
        }
    }
}

pub struct SilpaAlternative{}
impl Variants for SilpaAlternative{
    type VariantSettings = Settings;
    type VariantTelemetry = Telemetry;
    type VariantTelemetryBuffer = TelemetryBuffer;
}

impl Devices  <Settings, Telemetry> for SiLPA<SilpaAlternative>
{
    fn init(&mut self, _ecp5: &mut ECP5) -> bool {true}
    fn settings_update(&mut self, _ecp5: &mut ECP5, _new_settings: Settings) -> () {

    }

    fn telemetry(&mut self, _ecp5: &mut ECP5) -> (Telemetry, u16) {
        (self.telemetry.finalize(), 10)
    }
    fn check_interrupt(&mut self, _ecp5: &mut ECP5){}
}


