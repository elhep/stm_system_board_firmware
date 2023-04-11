pub mod red;
pub mod yellow;
pub mod green;

use core::fmt::Debug;
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use crate::hardware::devices::Variants;


pub struct Nucleo<T, U>
where
    T: Variants,
    U: OutputPin,
    U::Error: Debug,
{
    pub settings: T::VariantSettings,
    pub telemetry: T::VariantTelemetryBuffer,
    pub output: U,
}

impl<T, U> Nucleo<T, U>
where
    T: Variants,
    U: OutputPin,
    U::Error: Debug,
{
    pub fn new(
        _slot_number: u8,
        output: U,
    ) -> Self {
        Self {
            settings: T::VariantSettings::default(),
            telemetry: T::VariantTelemetryBuffer::default(),
            output,
        }
    }
}
    