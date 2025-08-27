pub use miniconf::Miniconf;
use serde::Serialize;
// use crate::hardware::devices::Devices;
use crate::hardware::ecp5::{self, ECP5};
use crate::hardware::setup::BusReference;
use embedded_hal::blocking::delay::DelayMs;

// pub type SPIInterface = hal::xspi::Qspi<hal::device::QUADSPI>;

// #[derive(Copy, Clone)]
// pub struct TempTelemetryBuffer {
//     pub temp: u16,
//     pub status: u8
// }

// impl Default for TempTelemetryBuffer {
//     fn default() -> Self {
//         Self {
//             temp: 0,
//             status: 0
//         }
//     }
// }

#[derive(Serialize, Default, Clone, Copy)]
pub struct Telemetry {
    pub t: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub active: bool,
}

pub const WRITE :u8 = 0;
pub const READ :u8 = 1 << 7;


// Registers:
pub const XOUT0     :u8 = 0;
pub const XOUT1     :u8 = 1;
pub const YOUT0     :u8 = 2;
pub const YOUT1     :u8 = 3;
pub const ZOUT0     :u8 = 4;
pub const ZOUT1     :u8 = 5;
pub const XYZOUT2   :u8 = 6;
pub const TOUT      :u8 = 7;
pub const STATUS    :u8 = 8;
pub const INTERNAL_CONTROL_0    :u8 = 9;
pub const INTERNAL_CONTROL_1    :u8 = 10;
pub const INTERNAL_CONTROL_2    :u8 = 11;
pub const INTERNAL_CONTROL_3    :u8 = 12;
pub const PRODUCT_ID_1          :u8 = 0x2F;

#[derive(Clone, Copy, Debug, Miniconf, PartialEq)]
pub struct Settings {
    pub x_inhibit: bool,
    pub active : bool,
    // pub a   : f32,      //
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            x_inhibit: false,
            active: false,
            // a: 3.9083e-3,
        }
    }
}

#[derive(Clone, Copy)]
pub struct Mmc5983ma {
    slot: u16,
    pub settings: Settings,
    internal_control_registers: [u8; 4],
}

impl Mmc5983ma {
    pub fn new(
        slot_number: u16,
    ) -> Self {
        Self {
            slot: slot_number,
            settings: Settings::default(),
            internal_control_registers: [0; 4],
        }
    }

    pub fn write_register(&mut self, ecp5: &mut ECP5, reg_adr: u8, value: u8) {
        let data : [u8; 2] = [reg_adr | WRITE, value];
        ecp5.write_spi(self.slot, &data);
    }

    pub fn read_register(&mut self, ecp5: &mut ECP5, reg_adr: u8, value: &mut [u8]){
        let address = [reg_adr | READ];
        ecp5.read_spi(self.slot, &address, value);
    }

    pub fn set_continuous_mode(&mut self, ecp5: &mut ECP5, freq: u16, enable: bool) {
        // Frequency is in Hz
        let freq_register: u8 = match freq {
            0 => 0,
            1 => 1,
            10 => 2,
            20 => 3,
            50 => 4,
            100 => 5,
            200 => 6,
            1000 => 7,
            _ => 0
        };

        // log::info!("Magnetometer: set_continuous_mode: data before: {:b}", self.internal_control_registers[2]);
        self.internal_control_registers[2] = (self.internal_control_registers[2] & !(1 << 3)) | ((enable as u8) << 3);
        self.internal_control_registers[2] = (self.internal_control_registers[2] & !(0b111)) | freq_register;
        // log::info!("Magnetometer: set_continuous_mode: data after: {:b}", self.internal_control_registers[2]);

        self.write_register(ecp5, INTERNAL_CONTROL_2, self.internal_control_registers[2]);
        // log::info!("Magnetometer: set_continuous_mode frequency: {} enable: {}", freq, enable);

        // Interrupt must be enabled for continuous mode to work
        self.enable_interrupt(ecp5, enable);
    }

    pub fn reset(&mut self, ecp5: &mut ECP5) {
        self.write_register(ecp5, INTERNAL_CONTROL_1, self.internal_control_registers[1] | (1 << 7));
        // log::info!("Magnetometer: Reset");
    }

    pub fn enable_interrupt(&mut self, ecp5: &mut ECP5, enable: bool) {
        self.internal_control_registers[0] = self.internal_control_registers[0] | ((enable as u8) << 2);
        self.write_register(ecp5, INTERNAL_CONTROL_0, self.internal_control_registers[0]);
        // log::info!("Magnetometer: Reset");
    }

    pub fn set_x_inhibit(&mut self, ecp5: &mut ECP5, inhibit: bool) {
        self.internal_control_registers[1] = (self.internal_control_registers[1] & !(1 << 2)) | ((inhibit as u8) << 2);
        self.write_register(ecp5, INTERNAL_CONTROL_1, self.internal_control_registers[1]);
        // log::info!("Magnetometer: set_x_inhibit: {}"", inhibit);
    }

    pub fn set_yz_inhibit(&mut self, ecp5: &mut ECP5, inhibit: bool) {
        // log::info!("Magnetometer: set_yz_inhibit: {}, register: 0x{:X}", inhibit, self.internal_control_registers[1]);
        // self.internal_control_registers[1] = (self.internal_control_registers[1] & !(1 << 3)) | ((inhibit as u8) << 3);
        self.internal_control_registers[1] = (self.internal_control_registers[1] & !(0b11 << 3)) | ((inhibit as u8) << 4) | ((inhibit as u8) << 3);
        self.write_register(ecp5, INTERNAL_CONTROL_1, self.internal_control_registers[1]);
        // self.write_register(ecp5, INTERNAL_CONTROL_1, self.internal_control_registers[1]);
        // log::info!("Magnetometer: set_yz_inhibit: {}, register: 0x{:X}", inhibit, self.internal_control_registers[1]);
    }

    pub fn set_z_inhibit(&mut self, ecp5: &mut ECP5, inhibit: bool) {
        log::info!("Magnetometer: set_z_inhibit: {}, register: 0x{:X}", inhibit, self.internal_control_registers[1]);
        self.internal_control_registers[1] = (self.internal_control_registers[1] & !(1 << 4)) | ((inhibit as u8) << 4);
        self.write_register(ecp5, INTERNAL_CONTROL_1, self.internal_control_registers[1]);
        log::info!("Magnetometer: set_z_inhibit: {}, register: 0x{:X}", inhibit, self.internal_control_registers[1]);
    }

    pub fn measure_m_field(&mut self, ecp5: &mut ECP5) -> (f32, f32, f32) {
        // Take magnetic field measurement
        self.write_register(ecp5, INTERNAL_CONTROL_0, self.internal_control_registers[0] | 1 << 0);
        self.read_m_field(ecp5)
    }

    pub fn read_m_field(&mut self, ecp5: &mut ECP5) -> (f32, f32, f32) {
        let mut delay = asm_delay::AsmDelay::new(asm_delay::bitrate::Hertz(
            400000000,
        ));
        let mut data = [0];

        while (data[0] & 1) != 1 {
            self.read_register(ecp5, STATUS, &mut data);
            // log::info!("Magnetometer: Wait for m meas done, status: 0x{:X}", data[0]);
            delay.delay_ms(10 as u32);
        }

        let mut m_field: [u8; 7] = [0; 7];

        self.read_register(ecp5, XOUT0, &mut m_field);
        let x_field: f32 = (((u32::from_be_bytes([0, m_field[0], m_field[1], m_field[6] & 0b11000000]) >> 6) as f32) - 131072.) / 16384.;
        let y_field: f32 = (((u32::from_be_bytes([0, m_field[2], m_field[3], (m_field[6] & 0b00110000) << 2]) >> 6) as f32) - 131072.) / 16384.;
        let z_field: f32 = (((u32::from_be_bytes([0, m_field[4], m_field[5], (m_field[6] & 0b00001100) << 4]) >> 6) as f32) - 131072.) / 16384.;
        // log::info!("Magnetometer: raw Field: {}, {}, {}, {}, {}, {}, {}", m_field[0], m_field[1], m_field[2], m_field[3], m_field[4], m_field[5], m_field[6]);
        log::info!("Magnetometer: Field: X {} Y {} Z {}", x_field, y_field, z_field);
        (x_field, y_field, z_field)
    }

    // Temperature cannot be measured when automatic magnetic measurements are enabled
    pub fn measure_temperature(&mut self, ecp5: &mut ECP5) -> f32 {
        let mut delay = asm_delay::AsmDelay::new(asm_delay::bitrate::Hertz(
            400000000,
        ));
        let mut data: [u8;1] = [0];
        self.write_register(ecp5, INTERNAL_CONTROL_0, data[0] | (1 << 1));

        data = [0];

        while (data[0] >> 1) & 1 != 1 {
            self.read_register(ecp5, STATUS, &mut data);
            // log::info!("Magnetometer: Wait for t meas done, status: 0x{:X}", data[0]);
            delay.delay_ms(100 as u32);
        }

        self.read_register(ecp5, TOUT, &mut data);
        let result = -75.0 + (data[0] as f32)*200.0/255.0;
        log::info!("Magnetometer: Temperature: {} -> {} deg C", data[0], result);
        result
    }

    // Expecting 0b00110000 = 0x30
    pub fn read_product_id(&mut self, ecp5: &mut ECP5) -> u8 {
        let mut data: [u8;1] = [0];
        self.read_register(ecp5, PRODUCT_ID_1, &mut data);
        data[0]
    }
}
