
use crate::hardware::{setup::BusReference, SystemTimer};
use core::ops::{Deref, DerefMut};
use super::{HvSupIsol, OutputVariant};

pub type Settings = super::Settings;
pub type Telemetry = super::Telemetry;

pub struct HvSupPosPos(HvSupIsol);

impl HvSupPosPos {
    pub fn new(slot_number: u16, bus: BusReference) -> HvSupPosPos {
        HvSupPosPos(HvSupIsol::new(
            slot_number,
            bus,
            OutputVariant::Positive,
            OutputVariant::Positive,
        ))
    }
}

impl Deref for HvSupPosPos {
    type Target = HvSupIsol;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for HvSupPosPos {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
