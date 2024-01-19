use super::{OutputVariant, HVSUP_ISOL};
use crate::{hvsup_telemetry, hvsup_devices_trait};
use crate::hardware::ecp5::ECP5;
use crate::hardware::ecp5;
use crate::hardware::ServMod;
use crate::hardware::devices::{Variants, Devices};
use crate::hardware::devices::max1329::{self, Max1329, dac};
use crate::hardware::devices::max1329::adc::{self, AdcCode};
use stm32h7xx_hal as hal;
use embedded_hal::digital::v2::OutputPin;
use crate::hardware::lm75a;

pub type Settings = super::Settings;
pub type Telemetry = super::Telemetry;
pub type TelemetryBuffer = super::TelemetryBuffer;

pub struct HvSupPosPos{}
impl Variants for HvSupPosPos{
    type VariantSettings = Settings;
    type VariantTelemetry = Telemetry;
    type VariantTelemetryBuffer = TelemetryBuffer;
}

hvsup_devices_trait!(HvSupPosPos);