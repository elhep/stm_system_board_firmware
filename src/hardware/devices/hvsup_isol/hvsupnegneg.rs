use super::{OutputVariant, HVSUP_ISOL};
use crate::{hvsup_telemetry, hvsup_devices_trait};
use crate::hardware::ecp5::ECP5;
use crate::hardware::devices::{Variants, Devices};
use crate::hardware::devices::max1329::{self, Max1329, dac};
use crate::hardware::devices::max1329::adc::{self, AdcCode};

pub type Settings = super::Settings;
pub type Telemetry = super::Telemetry;
pub type TelemetryBuffer = super::TelemetryBuffer;

pub struct HvSupNegNeg{}
impl Variants for HvSupNegNeg{
    type VariantSettings = Settings;
    type VariantTelemetry = Telemetry;
    type VariantTelemetryBuffer = TelemetryBuffer;
}

hvsup_devices_trait!(HvSupNegNeg);