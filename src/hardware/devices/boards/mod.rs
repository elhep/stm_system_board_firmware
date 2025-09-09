pub mod hvsup_isol;
pub mod magneto;
pub mod templogger;
pub mod isospi_8ch;

use miniconf::Miniconf;
use serde::Serialize;
use crate::hardware::setup::BusReference;

// TODO finish trait

pub trait Devices<T, U>
where
    T: Default,
    U: Serialize,
{
//    fn init(&self) -> ();
//    fn process(&self) -> ();
    fn init(&mut self) -> bool;
    fn settings_update(&mut self, new_settings: T) -> ();
    fn telemetry(&mut self) -> (U, u16);
    fn check_interrupt(&mut self) -> ();
    fn poll(&mut self) -> u32;
}

pub trait Variants
{
    type VariantSettings: Default+Miniconf;
    type VariantTelemetry: Serialize;
    type VariantTelemetryBuffer: Default;
}

pub struct EmptySlot{}
impl EmptySlot{
    pub fn new(_: u8, _: BusReference) -> Self{ Self{}}
}

impl Devices<bool, bool> for EmptySlot{
    fn init(&mut self) -> bool {false}
    fn settings_update(&mut self, set: bool) -> () {
        log::info!("value of none option: {}", set);
    }
    fn telemetry(&mut self) -> (bool, u16) {(false,0)}
    fn check_interrupt(&mut self) -> () {}
    fn poll(&mut self) -> u32 {0}
}
