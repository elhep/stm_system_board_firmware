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
    pub slope : [u16; 2],
    pub intercept : [u16; 2],
    pub signal_absence : [u16; 2],
    pub are_channel_activated : bool
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
            slope: [0; 2],
            intercept: [0; 2],
            signal_absence: [700, 700],
            are_channel_activated: false
        }

    }


}


