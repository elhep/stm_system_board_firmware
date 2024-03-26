pub mod silpadefault;
pub mod alternative;
pub mod tester;

use crate::hardware::{devices::Variants, eeprom::SiLPADetector, setup::{BusReference}};


pub struct SiLPA<U>
where
    U: Variants,
{
    pub slot: u8,
    pub settings: U::VariantSettings,
    pub telemetry: U::VariantTelemetryBuffer,
    pub bus: BusReference,
    pub detector : SiLPADetector
}

impl<U> SiLPA <U>
where
    U: Variants,
{
    pub fn new(
        slot_number : u8,
        bus: BusReference,
    ) -> Self
    {
        Self{
            slot: slot_number,
            settings: U::VariantSettings::default(),
            telemetry: U::VariantTelemetryBuffer::default(),
            bus: bus,
            detector: SiLPADetector::new(0.0, 0.0, 0.0, 0.0)
        }

    }


}


