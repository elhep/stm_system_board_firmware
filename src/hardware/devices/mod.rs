pub mod max1329;
pub mod silpa;
pub mod nucleo_tester;
pub mod hvsup_isol;

use miniconf::Miniconf;
use serde::Serialize;
use crate::hardware::ecp5::ECP5;

// TODO finish trait

pub trait Devices<T, U>
where
    T: Default,
    U: Serialize,
{
//    fn init(&self) -> ();
//    fn process(&self) -> ();
    fn init(&mut self, ecp5: &mut ECP5) -> bool;
    fn settings_update(&mut self, ecp5: &mut ECP5, new_settings: T) -> ();
    fn telemetry(&mut self) -> (U, u16);
    fn check_interrupt(&mut self, ecp5: &mut ECP5) -> ();
}

pub trait Variants
{
    type VariantSettings: Default+Miniconf;
    type VariantTelemetry: Serialize;
    type VariantTelemetryBuffer: Default;
}

pub struct EmptySlot{}
impl EmptySlot{
    pub fn new(_: u8) -> Self{ Self{}}
}

impl Devices<bool, bool> for EmptySlot{
    fn init(&mut self, _ecp5: &mut ECP5) -> bool {false}
    fn settings_update(&mut self, _: &mut ECP5, set: bool) -> () {
        log::info!("value of none option: {}", set);
    }
    fn telemetry(&mut self) -> (bool, u16) {(false,0)}
    fn check_interrupt(&mut self, _ecp5: &mut ECP5) -> () {}
}