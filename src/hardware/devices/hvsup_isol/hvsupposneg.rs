
use crate::hardware::{setup::BusReference, SystemTimer};
use core::ops::{Deref, DerefMut};
use super::{HvSupIsol, OutputVariant};

pub type Settings = super::Settings;
pub type Telemetry = super::Telemetry;

pub struct HvSupPosNeg(HvSupIsol);

impl HvSupPosNeg {
    pub fn new(slot_number: u16, bus: BusReference) -> HvSupPosNeg {
        HvSupPosNeg(HvSupIsol::new(
            slot_number,
            bus,
            OutputVariant::Positive,
            OutputVariant::Negative,
        ))
    }
}

impl Deref for HvSupPosNeg {
    type Target = HvSupIsol;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for HvSupPosNeg {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
