use super::max_controller::Coefs;
use crate::hardware::ecp5::ECP5;
use crate::hardware::{lm75a, setup::CpcisI2C, ServMod};
use embedded_hal::{digital::v2::OutputPin, prelude::*};

#[derive(Clone, Copy)]
pub enum IoPin {
    Interlock,
}

impl IoPin {
    fn mask(&self) -> u8 {
        match self {
            IoPin::Interlock => 0b1000_0000u8,
        }
    }
}

pub struct BoardController {
    slot: u8,
    outputs: [u8; 2],
    interrupts_mask: u8,
}

impl BoardController {
    const EEPROM_ADDRESS: u8 = 0x50;
    const THERM_ADDRESS: u8 = 0x48;

    const HV_EN_MASK: u8 = 0b0010_0000;
    const PSU_EN_MASK: u8 = 0b0001_0000;

    pub fn new(slot: u8) -> BoardController {
        BoardController {
            slot,
            outputs: [0u8; 2],
            interrupts_mask: 0,
        }
    }

    pub fn init(&self, ecp5: &mut ECP5) {
        let data = BoardController::HV_EN_MASK | BoardController::PSU_EN_MASK;
        ecp5.write_oe(self.slot, &[0, data]);
        self.clear_interrupts(ecp5);
    }

    pub fn switch_psu_enable(&mut self, state: bool, ecp5: &mut ECP5) {
        if state {
            self.outputs[1] |= BoardController::PSU_EN_MASK;
        } else {
            self.outputs[1] &= !BoardController::PSU_EN_MASK;
        }
        ecp5.write_outputs(self.slot, &self.outputs);
    }

    pub fn switch_hv_enable(&mut self, state: bool, ecp5: &mut ECP5) {
        if state {
            self.outputs[1] |= BoardController::HV_EN_MASK;
        } else {
            self.outputs[1] &= !BoardController::HV_EN_MASK;
        }
        ecp5.write_outputs(self.slot, &self.outputs);
    }

    pub fn enable_interrupt(&mut self, io_pin: IoPin, ecp5: &mut ECP5) {
        self.interrupts_mask |= io_pin.mask();

        ecp5.write_interrupts_mask(self.slot, &[0, self.interrupts_mask]);
    }

    pub fn read_device_name(&self, cpcis_i2c: &mut CpcisI2C, servmod: &mut ServMod) -> [u8; 10] {
        self.switch_servmod(true, servmod);
        let mut buff = [0u8; 10];
        let ret = if cpcis_i2c
            .write_read(BoardController::EEPROM_ADDRESS, &[6], &mut buff)
            .is_ok()
        {
            buff
        } else {
            log::info!("Failed to read eeprom!");
            [0u8; 10]
        };
        self.switch_servmod(false, servmod);
        ret
    }

    pub fn read_temp(&self, cpcis_i2c: &mut CpcisI2C, servmod: &mut ServMod) -> f32 {
        self.switch_servmod(true, servmod);
        let ret = match lm75a::read_temp(cpcis_i2c, BoardController::THERM_ADDRESS) {
            Ok(val) => val,
            Err(_) => {
                log::info!("HVSUP failed to read temp!");
                0.0
            }
        };
        self.switch_servmod(false, servmod);
        ret
    }

    pub fn read_io(&self, pin: IoPin, ecp5: &mut ECP5) -> bool {
        let mut data = [0u8; 2];
        ecp5.read_inputs(self.slot, &mut data);
        (data[1] & pin.mask()) > 0
    }

    fn switch_servmod(&self, on: bool, servmod: &mut ServMod) {
        servmod.0.set_low().unwrap();
        servmod.1.set_low().unwrap();
        servmod.2.set_low().unwrap();
        servmod.3.set_low().unwrap();
        servmod.4.set_low().unwrap();
        servmod.5.set_low().unwrap();
        servmod.6.set_low().unwrap();
        servmod.7.set_low().unwrap();

        if !on {
            return;
        }

        // TODO(Adrian) - Change to self.slot
        let _ = &self.slot;
        match 5 {
            1 => servmod.0.set_high().unwrap(),
            2 => servmod.1.set_high().unwrap(),
            3 => servmod.2.set_high().unwrap(),
            4 => servmod.3.set_high().unwrap(),
            5 => servmod.4.set_high().unwrap(),
            6 => servmod.5.set_high().unwrap(),
            7 => servmod.6.set_high().unwrap(),
            8 => servmod.7.set_high().unwrap(),
            _ => log::info!("HVSUP received incorrect slot!"),
        };
    }

    pub fn clear_interrupts(&self, ecp5: &mut ECP5) {
        let mut data = [0xffu8; 2];
        ecp5.write_clear_interrupts(self.slot, &mut data);
    }

    pub fn save_coefs(&self, coefs: &[Coefs; 2], cpcis_i2c: &mut CpcisI2C, servmod: &mut ServMod) {
        let mut delay = asm_delay::AsmDelay::new(asm_delay::bitrate::Hertz(400000000));
        self.switch_servmod(true, servmod);

        let data1 = coefs[0].serialize();
        let data2 = coefs[1].serialize();
        let offset = 24;

        // log::info!("Saving to EEPROM: ");
        // for i in 0..data1.len() {
        //     log::info!("[{}] = {}", i, data1[i]);
        // }
        // for i in 0..data2.len() {
        //     log::info!("[{}] = {}", i, data2[i]);
        // }
        // log::info!("Finished");

        for i in 0..data1.len() {
            let _ = cpcis_i2c
                .write(
                    BoardController::EEPROM_ADDRESS,
                    &[offset + i as u8, data1[i]],
                )
                .is_ok();
            delay.delay_ms(100 as u32);
            let _ = cpcis_i2c
                .write(
                    BoardController::EEPROM_ADDRESS,
                    &[offset + 8 + i as u8, data2[i]],
                )
                .is_ok();
            delay.delay_ms(100 as u32);
        }
        self.switch_servmod(false, servmod);
    }

    pub fn read_coefs(&self, cpcis_i2c: &mut CpcisI2C, servmod: &mut ServMod) -> [Coefs; 2] {
        self.switch_servmod(true, servmod);
        let mut buff = [0u8; 8];
        // log::info!("Reading EEPROM");

        let ret = if cpcis_i2c
            .write_read(BoardController::EEPROM_ADDRESS, &[24], &mut buff)
            .is_ok()
        {
            buff
        } else {
            log::info!("Failed to read eeprom!");
            [0u8; 8]
        };
        // for i in 0..ret.len() {
        //     log::info!("[{}] = {}", i, ret[i]);
        // }
        let channel_a = Coefs::deserialize(ret);
        let ret = if cpcis_i2c
            .write_read(BoardController::EEPROM_ADDRESS, &[32], &mut buff)
            .is_ok()
        {
            buff
        } else {
            log::info!("Failed to read eeprom!");
            [0u8; 8]
        };
        // for i in 0..ret.len() {
        //     log::info!("[{}] = {}", i+8, ret[i]);
        // }
        // log::info!("Finished");
        let channel_b = Coefs::deserialize(ret);
        self.switch_servmod(false, servmod);
        [channel_a, channel_b]
    }

    // TODO(Adrina) - Remove
    pub fn program_eeprom(&self, cpcis_i2c: &mut CpcisI2C, servmod: &mut ServMod) {
        let mut delay = asm_delay::AsmDelay::new(asm_delay::bitrate::Hertz(400000000));
        self.switch_servmod(true, servmod);

        let data = "HVSUP_ISOL".bytes();
        let name_offset = 6;

        for (i, byte) in data.enumerate() {
            let _ = cpcis_i2c
                .write(
                    BoardController::EEPROM_ADDRESS,
                    &[name_offset + i as u8, byte],
                )
                .is_ok();
            delay.delay_ms(100 as u32);
        }
        self.switch_servmod(false, servmod);
    }
}
