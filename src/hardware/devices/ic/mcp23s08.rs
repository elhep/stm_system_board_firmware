use crate::hardware::setup::BusReference;
use crate::hardware::{ecp5, ecp5::ECP5};

pub mod REGS{
    pub const IODIR:    u8 = 0;
    pub const IPOL:     u8 = 1;
    pub const GPINTEN:  u8 = 2;
    pub const DEFVAL:   u8 = 3;
    pub const INTCON:   u8 = 4;
    pub const IOCON:    u8 = 5;
    pub const GPPU:     u8 = 6;
    pub const INTF:     u8 = 7;
    pub const INCAP:    u8 = 8;
    pub const GPIO:     u8 = 9;
    pub const OLAT:     u8 = 10;
}

pub struct Mcp23s08 {    
    adr: u8,
    slot: u16,
}

impl Mcp23s08 {
    pub fn new(
        adr: u8,
        slot: u16
    ) -> Self {
        Self {
            adr,
            slot
        }
    }

    pub fn set_register(&mut self, ecp5: &mut ECP5, reg_adr: u8, value: u8) {
        let data : [u8; 3] = [self.adr | (1 << 6), reg_adr, value];
        ecp5.write_spi(self.slot, &data);
    }

    pub fn read_register(&mut self, ecp5: &mut ECP5, reg_adr: u8, value: &mut [u8]){
        let address = [self.adr | (1 << 6) | 1, reg_adr];
        ecp5.read_spi(self.slot, &address, value);
    }
}