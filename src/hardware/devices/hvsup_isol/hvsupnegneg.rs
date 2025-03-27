
use crate::hardware::{setup::BusReference, SystemTimer};
use core::ops::{Deref, DerefMut};
use super::{HvSupIsol, OutputVariant};

pub type Settings = super::Settings;
pub type Telemetry = super::Telemetry;

pub struct HvSupNegNeg(HvSupIsol);

impl HvSupNegNeg {
    pub fn new(slot_number: u16, bus: BusReference) -> HvSupNegNeg {
        HvSupNegNeg(HvSupIsol::new(
            slot_number,
            bus,
            OutputVariant::Negative,
            OutputVariant::Negative,
        ))
    }
}

impl Deref for HvSupNegNeg {
    type Target = HvSupIsol;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for HvSupNegNeg {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
