
use crate::hardware::{setup::BusReference, SystemTimer};
use core::ops::{Deref, DerefMut};
use super::{HvSupIsol, OutputVariant};

pub type Settings = super::Settings;
pub type Telemetry = super::Telemetry;

pub struct HvSupNegPos(HvSupIsol);

impl HvSupNegPos {
    pub fn new(slot_number: u16, bus: BusReference) -> HvSupNegPos {
        HvSupNegPos(HvSupIsol::new(
            slot_number,
            bus,
            OutputVariant::Negative,
            OutputVariant::Positive,
        ))
    }
}

impl Deref for HvSupNegPos {
    type Target = HvSupIsol;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for HvSupNegPos {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
