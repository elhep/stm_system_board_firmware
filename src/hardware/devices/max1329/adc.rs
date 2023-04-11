use serde::{Deserialize, Serialize};
use num_enum::TryFromPrimitive;
use miniconf::Miniconf;

// ADC
// u16 for unipolar, i16 for bipolar
#[derive(Copy, Clone)]
pub struct AdcCode (pub u16);

impl AdcCode{
    pub fn voltage(self, gain: Gain, vref: f32) -> f32{
        Into::<f32>::into(self.0) * vref / (gain.as_multiplier() * 4096.0)
    }

    pub fn diff_voltage(self, gain: Gain, vref: f32) -> f32{
        -vref + self.voltage(gain, vref)
    }
}

//impl From<u16> for AdcCode<i16> {
//    fn from(value: u16) -> Self { Self(value as i16)}
//}
//
//impl From<u16> for AdcCode<u16> {
//    fn from(value: u16) -> Self { Self(value)}
//}

//impl From<u16> for i16 {
//    fn from(value: u16) -> Self { value as i16}
//}

//impl AdcCode<i16>{
//    pub fn voltage(self, gain: Gain, vref: f32) -> f32{
//        Into::<f32>::into(self.0) * vref / (gain.as_multiplier() * 4096.0)
//    }
//}

pub enum AutoConversion{
    Disabled = 0b000,
    Clk32 = 0b001,
    Clk64 = 0b010,
    Clk128 = 0b011,
    Clk256 = 0b100,
    Clk512 = 0b101,
    Clk1024 = 0b110,
    Clk2048 = 0b111,
}

pub enum RefConf{
    ExtBuffOff = 0b000, // drive REFADC directly with an external reference
    ExtAdj0_5     = 0b010,
    ExtAdj0_8192  = 0b100,
    ExtAdj1_0     = 0b110,
    IntHiZ     = 0b001,
    Int1_25    = 0b011,
    Int2_048   = 0b101,
    Int2_5     = 0b111,
}

pub enum PowerDownConf{
    PowerDown = 0b00,
    FastPowerDown = 0b01,
    Normal = 0b10,
    Burst = 0b11,
}

pub enum Refe{
    Internal = 0b1,
    External = 0b0,
}

// It's one enum for MSEL and MUX bits for MAX1329
#[repr(u8)]
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, PartialEq)]
pub enum Mux{
    AIN1_AGND     = 0b00000,
    AIN2_AGND     = 0b00001,
    OUTA_AGND     = 0b00010,
    FBA_AGND      = 0b00011,
    OUT1_AGND     = 0b00100,
    IN1_AGND      = 0b00101,
    OUTB_AGND     = 0b00110,
    FBB_AGND      = 0b00111,
    AIN1_AIN2     = 0b01000,
    AIN2_AIN1     = 0b01001,
    OUTA_FBA      = 0b01010,
    FBA_OUTA      = 0b01011,
    OUT1_IN1      = 0b01100,
    IN1_OUT1      = 0b01101,
    OUTB_OUT2     = 0b01110,
    FBB_OUTB      = 0b01111,
    AIN1_REFADC   = 0b10000,
    OUTA_REFADC   = 0b10001,
    OUT1_REFADC   = 0b10010,
    OUTB_REFADC   = 0b10011,
    AIN1_REFDAC   = 0b10100,
    OUTA_REFDAC   = 0b10101,
    OUT1_REFDAC   = 0b10110,
    OUTB_REFDAC   = 0b10111,
    TEMP1p_TEMP1n = 0b11000,
    TEMP2p_TEMP2n = 0b11001,
    TEMP3p_AGND   = 0b11010,
    TEMP4p_AGND   = 0b11011,
    DVdd4_AGND    = 0b11100,
    AVdd4_AGND    = 0b11101,
    REFADC_AGND   = 0b11110,
    REFDAC_AGND   = 0b11111,
}

/// A type representing an ADC sample.
#[derive(Copy, Clone, Debug, Serialize, Deserialize, TryFromPrimitive, Miniconf, PartialEq)]
#[repr(u8)]
pub enum Gain{
    G1 = 0b00,
    G2 = 0b01,
    G4 = 0b10,
    G8 = 0b11,
}

impl Gain{
    /// Get the MAX1329 ADC internal AFE gain as a numerical value
    pub fn as_multiplier(self) -> f32 {
        match self {
            Gain::G1 => 1.0,
            Gain::G2 => 2.0,
            Gain::G4 => 4.0,
            Gain::G8 => 8.0,
        }
    }
}

#[repr(u8)]
pub enum Bip{
    Unipolar = 0b0,
    Bipolar  = 0b1,
}

pub enum AlarmMode{
    NonConsecutive = 0b0,
    Consecutive    = 0b1,
}