pub mod silpadefault;
pub mod alternative;
pub mod tester;

use crate::hardware::{devices::Variants, eeprom::SiLPADetector, setup::BackPlaneI2C};


pub struct SiLPA<U>
where
    U: Variants,
{
    pub slot: u8,
    pub settings: U::VariantSettings,
    pub telemetry: U::VariantTelemetryBuffer,
    pub backplane : BackPlaneI2C,
    pub detector : SiLPADetector
}

impl<U> SiLPA <U>
where
    U: Variants,
{
    pub fn new(
        slot_number : u8,
        back_plane : BackPlaneI2C
    ) -> Self
    {
        Self{
            slot: slot_number,
            settings: U::VariantSettings::default(),
            telemetry: U::VariantTelemetryBuffer::default(),
            backplane: back_plane,
            detector: SiLPADetector::new(0.0, 0.0, 0.0, 0.0)
        }

    }


}


