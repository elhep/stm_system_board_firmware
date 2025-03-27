pub mod silpadefault;
pub mod alternative;
pub mod tester;

use crate::hardware::devices::Variants;


pub struct SiLPA<U>
where
    U: Variants,
{
    slot: u16,
    pub settings: U::VariantSettings,
    pub telemetry: U::VariantTelemetryBuffer,
}

impl<U> SiLPA <U>
where
    U: Variants,
{
    pub fn new(
        slot_number : u16,
    ) -> Self
    {
        Self{
            slot: slot_number,
            settings: U::VariantSettings::default(),
            telemetry: U::VariantTelemetryBuffer::default(),
        }

    }


}


