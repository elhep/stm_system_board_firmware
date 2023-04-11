 use miniconf::Miniconf;
 //use heapless::String;
 use crate::hardware::devices::silpa::{SiLPA, silpadefault::{self, SilpaDefault}};
 use crate::hardware::devices::nucleo_tester::green::{self, Green};
 use crate::hardware::devices::nucleo_tester::yellow::{self, Yellow};
 use crate::hardware::devices::nucleo_tester::red::{self, Red};
 use crate::hardware::devices::hvsup_isol::{HVSUP_ISOL, hvsuppospos::{self, HvSupPosPos},
                                                        hvsupnegneg::{self, HvSupNegNeg},
                                                        hvsupposneg::{self, HvSupPosNeg},
                                                        hvsupnegpos::{self, HvSupNegPos},};
 use crate::hardware::devices::EmptySlot;
 use crate::hardware::devices::nucleo_tester::{Nucleo};
 use crate::hardware::hal;

 macro_rules! settings {
    (EmptySlot) => {
        paste::paste!{
        Option<bool>
    }};
    ($class:ident) => {
        paste::paste!{
        Option<[< $class:lower >]::Settings>
    }
    };
 }

 macro_rules! settings_default {
    (EmptySlot) => {
        paste::paste!{
        None
    }};
    ($class:ident) => {
        paste::paste!{
        Some([< $class:lower >]::Settings::default())
    }
    };
 }

 macro_rules! device_type {
 (Green) => {
      paste::paste! {
           Nucleo<Green, hal::gpio::gpiob::PB0<hal::gpio::Output<hal::gpio::PushPull>>>
      }
  };
 (Yellow) => {
      paste::paste! {
           Nucleo<Yellow, hal::gpio::gpioe::PE1<hal::gpio::Output<hal::gpio::PushPull>>>
      }
  };
 (Red) => {
      paste::paste! {
           Nucleo<Red, hal::gpio::gpiob::PB14<hal::gpio::Output<hal::gpio::PushPull>>>
      }
 };
 (SilpaDefault) => {
      paste::paste! {
           SiLPA<SilpaDefault>
      }
 };
  (SilpaAlternative) => {
      paste::paste! {
           SiLPA<SilpaAlternative>
      }
 };
  (EmptySlot) => {
      paste::paste! {
           EmptySlot
      }
 };
   (HvSupPosPos) => {
      paste::paste! {
           HVSUP_ISOL<HvSupPosPos>
      }
   };
   (HvSupPosNeg) => {
      paste::paste! {
           HVSUP_ISOL<HvSupPosNeg>
      }
   };
   (HvSupNegPos) => {
      paste::paste! {
           HVSUP_ISOL<HvSupNegPos>
      }
   };
   (HvSupNegNeg) => {
      paste::paste! {
           HVSUP_ISOL<HvSupNegNeg>
      }
   };
 }

 macro_rules! devices_list {
     ($first:ident, $second:ident, $third:ident, $fourth:ident, $fifth:ident, $sixth:ident,
      $seventh:ident, $eighth:ident) => {
         paste::paste! {
             pub const DEVICE0_TELEMETRY_PREFIX: &str = stringify!([< s0_ $first:lower >]);
             pub const DEVICE1_TELEMETRY_PREFIX: &str = stringify!([< s1_ $second:lower >]);
             pub const DEVICE2_TELEMETRY_PREFIX: &str = stringify!([< s2_ $third:lower >]);
             pub const DEVICE3_TELEMETRY_PREFIX: &str = stringify!([< s3_ $fourth:lower >]);
             pub const DEVICE4_TELEMETRY_PREFIX: &str = stringify!([< s4_ $fifth:lower >]);
             pub const DEVICE5_TELEMETRY_PREFIX: &str = stringify!([< s5_ $sixth:lower >]);
             pub const DEVICE6_TELEMETRY_PREFIX: &str = stringify!([< s6_ $seventh:lower >]);
             pub const DEVICE7_TELEMETRY_PREFIX: &str = stringify!([< s7_ $eighth:lower >]);
             pub type Device0Type = device_type!($first);
             pub type Device1Type = device_type!($second);
             pub type Device2Type = device_type!($third);
             pub type Device3Type = device_type!($fourth);
             pub type Device4Type = device_type!($fifth);
             pub type Device5Type = device_type!($sixth);
             pub type Device6Type = device_type!($seventh);
             pub type Device7Type = device_type!($eighth);

             #[derive(Clone, Copy, Debug, Miniconf)]
             pub struct Settings{
                 pub [< s0_ $first:lower >]: settings!($first),
                 pub [< s1_ $second:lower >]: settings!($second),
                 pub [< s2_ $third:lower >]: settings!($third),
                 pub [< s3_ $fourth:lower >]: settings!($fourth),
                 pub [< s4_ $fifth:lower >]: settings!($fifth),
                 pub [< s5_ $sixth:lower >]: settings!($sixth),
                 pub [< s6_ $seventh:lower >]: settings!($seventh),
                 pub [< s7_ $eighth:lower >]: settings!($eighth),
             }

             impl Settings {
                 pub fn device0_settings(&self) -> settings!($first) {
                       self.[< s0_ $first:lower >]
                 }
                 pub fn device1_settings(&self) -> settings!($second) {
                       self.[< s1_ $second:lower >]
                 }
                 pub fn device2_settings(&self) -> settings!($third) {
                       self.[< s2_ $third:lower >]
                 }
                 pub fn device3_settings(&self) -> settings!($fourth) {
                       self.[< s3_ $fourth:lower >]
                 }
                 pub fn device4_settings(&self) -> settings!($fifth) {
                       self.[< s4_ $fifth:lower >]
                 }
                 pub fn device5_settings(&self) -> settings!($sixth) {
                       self.[< s5_ $sixth:lower >]
                 }
                 pub fn device6_settings(&self) -> settings!($seventh) {
                       self.[< s6_ $seventh:lower >]
                 }
                 pub fn device7_settings(&self) -> settings!($eighth) {
                       self.[< s7_ $eighth:lower >]
                 }

             }

             impl Default for Settings {
                 fn default() -> Self{
                     Self{
                         [< s0_ $first:lower >]: settings_default!($first),
                         [< s1_ $second:lower >]: settings_default!($second),
                         [< s2_ $third:lower >]: settings_default!($third),
                         [< s3_ $fourth:lower >]: settings_default!($fourth),
                         [< s4_ $fifth:lower >]: settings_default!($fifth),
                         [< s5_ $sixth:lower >]: settings_default!($sixth),
                         [< s6_ $seventh:lower >]: settings_default!($seventh),
                         [< s7_ $eighth:lower >]: settings_default!($eighth),
                     }
                 }
             }
         }
     };
 }

 /// Supported devices:
 /// HVSUP_ISOL:
 ///     - HvSupPosPos
 ///     - HvSupNegNeg
 ///     - HvSupPosNeg
 ///     - HvSupNegPos
 /// SiLPA:
 ///     - SilpaDefault - work in progress

devices_list!(
     HvSupPosPos,HvSupPosPos,HvSupPosPos,SilpaDefault,HvSupNegNeg,HvSupNegPos,HvSupPosNeg,HvSupPosPos
);

