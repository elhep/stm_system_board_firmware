#[repr(u8)]
pub enum PowerDownConf{
    PowerDown = 0b00,
    InOut     = 0b01,
    In        = 0b10,
    InToOut   = 0b11,
}

#[repr(u8)]
pub enum OpAmp{
    Enable  = 0b1,
    Disable = 0b0,
}

#[repr(u8)]
pub enum RefConf{
    ExtBuffOff = 0b000, // drive REFADC directly with an external reference
    Ext0_5     = 0b010,
    Ext0_8192  = 0b100,
    Ext1_0     = 0b110,
    IntHiZ     = 0b001,
    Int1_25    = 0b011,
    Int2_048   = 0b101,
    Int2_5     = 0b111,
    // Ext0_5      = 0b010,
    // Ext0_8192   = 0b100,
    // Ext1_0      = 0b110,
    // Int1_25     = 0b011,
    // Int2_048    = 0b101,
    // Int2_5      = 0b111,
}

pub enum FifoAEnable{
    Enable  = 0b1,
    Disable = 0b0,
}

pub enum Bip{
    Unipolar = 0b0,
    Bipolar  = 0b1,
}

pub enum Symmetry{
    Asymmetrical = 0b0,
    Symmetry     = 0b1,
}

pub enum Continuous{
    Single   = 0b0,
    Periodic = 0b1,
}