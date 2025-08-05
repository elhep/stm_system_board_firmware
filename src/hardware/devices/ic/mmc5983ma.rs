pub use miniconf::Miniconf;
use serde::Serialize;
// use crate::hardware::devices::Devices;
use crate::hardware::ecp5::{self, ECP5};
use crate::hardware::setup::BusReference;

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
    pub temp: f32,
    x_field: f32,
    y_field: f32,
    z_field: f32,
    // is_calibration_on: bool,
}

pub const WRITE :u8 = 0;
pub const READ :u8 = 1 << 7;


// Registers:
pub const Xout0     :u8 = 0;
pub const Xout1     :u8 = 1;
pub const Yout0     :u8 = 2;
pub const Yout1     :u8 = 3;
pub const Zout0     :u8 = 4;
pub const Zout1     :u8 = 5;
pub const XYZout2   :u8 = 6;
pub const Tout      :u8 = 7;
pub const Status    :u8 = 8;
pub const Internal_control_0    :u8 = 9;
pub const Internal_control_1    :u8 = 10;
pub const Internal_control_2    :u8 = 11;
pub const Internal_control_3    :u8 = 12;
pub const Product_ID_1          :u8 = 0x2F;

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
}

impl Mmc5983ma {
    pub fn new(
        slot_number: u16,
    ) -> Self {
        Self {
            slot: slot_number,
            settings: Settings::default(),
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

    // pub fn setup_spi_cs_pol(slot: u8, ecp: &mut ECP5, pol: u8){
    //     let offset = slot * ecp5::OFFSET_TO_SLOT + ecp5::OFFSET_TO_SPI;
    //     ecp.write_to_ecp5(offset + ecp5::SPI::CS_POL, &mut [0x00, pol]).unwrap();
    //
    // }

    

    // pub fn set_adc_control_register(slot: u8,
    //                        ecp5: &mut ECP5,
    //                        auto: adc::AutoConversion,
    //                        apd: adc::PowerDownConf,
    //                        aref: adc::RefConf) {
    //     let data: u8;
    //     data = ((auto as u8) << 5) | ((apd as u8) << 3) | (aref as u8);
    //     let data = [ADC_CONTROL | WRITE, data];
    //     ecp5.write_spi(slot, &data);
    // }

    pub fn set_continuous_mode(&mut self, ecp5: &mut ECP5, freq: u16, enable: bool) {
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
        let mut data: [u8;1] = [0];
        // ecp5.read_spi(self.slot, &[Internal_control_2 | READ], &mut data);
        self.read_register(ecp5, Internal_control_2, &mut data);

        // log::info!("Magnetometer: set_continuous_mode: data before: {:b}", data[0]);
        data[0] = (data[0] & !(1 << 3)) | ((enable as u8) << 3);
        data[0] = (data[0] & !(0b111)) | freq_register;
        // log::info!("Magnetometer: set_continuous_mode: data after: {:b}", data[0]);

        // ecp5.write_spi(self.slot, &[Internal_control_2 | WRITE, data[0] | 1 << 7]);
        self.write_register(ecp5, Internal_control_2, data[0] | (1 << 7));
        // log::info!("Magnetometer: set_continuous_mode frequency: {} enable: {}", freq, enable);
    }

    pub fn reset(&mut self, ecp5: &mut ECP5) {
        let mut data: [u8;1] = [0];
        self.read_register(ecp5, Internal_control_1, &mut data);
        self.write_register(ecp5, Internal_control_1, data[0] | (1 << 7));
        // log::info!("Magnetometer: Reset");
    }

    pub fn set_x_inhibit(&mut self, ecp5: &mut ECP5, inhibit: bool) {
        let mut data: [u8;1] = [0];
        self.read_register(ecp5, Internal_control_1, &mut data);
        data[0] = (data[0] & !(1 << 2)) | ((inhibit as u8) << 2);
        self.write_register(ecp5, Internal_control_1, data[0]);
        // log::info!("Magnetometer: set_x_inhibit: {}", inhibit);
    }

    pub fn set_y_inhibit(&mut self, ecp5: &mut ECP5, inhibit: bool) {
        let mut data: [u8;1] = [0];
        self.read_register(ecp5, Internal_control_1, &mut data);
        data[0] = (data[0] & !(1 << 3)) | ((inhibit as u8) << 3);
        self.write_register(ecp5, Internal_control_1, data[0]);
        // log::info!("Magnetometer: set_y_inhibit: {}", inhibit);
    }

    pub fn set_z_inhibit(&mut self, ecp5: &mut ECP5, inhibit: bool) {
        let mut data: [u8;1] = [0];
        self.read_register(ecp5, Internal_control_1, &mut data);
        data[0] = (data[0] & !(1 << 4)) | ((inhibit as u8) << 4);
        self.write_register(ecp5, Internal_control_1, data[0]);
        // log::info!("Magnetometer: set_z_inhibit: {}", inhibit);
    }

    // pub fn measure_m_field(slot: u16, ecp5: &mut ECP5) -> (f32, f32, f32) {
    //     let mut data: [u8;1] = [0];
    //     ecp5.read_spi(slot, &[Internal_control_0 | READ], &mut data);
    //     ecp5.write_spi(slot, &[Internal_control_0 | WRITE, data[0] | 1 << 0]);

    //     data = [0];

    //     while (data[0] & 1) != 1 {
    //         // log::info!("Magnetometer: Wait for m meas done, status: {}", data[0]);
    //         ecp5.read_spi(slot, &[Status | READ], &mut data);
    //     }

    //     let mut m_field: [u8; 7] = [0; 7];

    //     ecp5.read_spi(slot, &[Xout0 | READ], &mut m_field);
    //     let x_field: f32 = (((u32::from_be_bytes([0, m_field[0], m_field[1], m_field[6] & 0b11000000]) >> 6) as f32) - 131072.) / 16384.;
    //     let y_field: f32 = (((u32::from_be_bytes([0, m_field[2], m_field[3], (m_field[6] & 0b00110000) << 2]) >> 6) as f32) - 131072.) / 16384.;
    //     let z_field: f32 = (((u32::from_be_bytes([0, m_field[4], m_field[5], (m_field[6] & 0b00001100) << 4]) >> 6) as f32) - 131072.) / 16384.;
    //     // log::info!("Magnetometer: raw Field: {}, {}, {}, {}, {}, {}, {}", m_field[0], m_field[1], m_field[2], m_field[3], m_field[4], m_field[5], m_field[6]);
    //     log::info!("Magnetometer: Field: X {} Y {} Z {}", x_field, y_field, z_field);
    //     (x_field, y_field, z_field)
    // }

    // pub fn read_m_field(slot: u16, ecp5: &mut ECP5) -> (f32, f32, f32) {
    //     let mut data: [u8;1] = [0];

    //     while (data[0] & 1) != 1 {
    //         // log::info!("Magnetometer: Wait for m meas done, status: {}", data[0]);
    //         ecp5.read_spi(slot, &[Status | READ], &mut data);
    //     }

    //     let mut m_field: [u8; 7] = [0; 7];

    //     ecp5.read_spi(slot, &[Xout0 | READ], &mut m_field);
    //     let x_field: f32 = (((u32::from_be_bytes([0, m_field[0], m_field[1], m_field[6] & 0b11000000]) >> 6) as f32) - 131072.) / 16384.;
    //     let y_field: f32 = (((u32::from_be_bytes([0, m_field[2], m_field[3], (m_field[6] & 0b00110000) << 2]) >> 6) as f32) - 131072.) / 16384.;
    //     let z_field: f32 = (((u32::from_be_bytes([0, m_field[4], m_field[5], (m_field[6] & 0b00001100) << 4]) >> 6) as f32) - 131072.) / 16384.;
    //     // log::info!("Magnetometer: raw Field: {}, {}, {}, {}, {}, {}, {}", m_field[0], m_field[1], m_field[2], m_field[3], m_field[4], m_field[5], m_field[6]);
    //     log::info!("Magnetometer: Field: X {} Y {} Z {}", x_field, y_field, z_field);
    //     (x_field, y_field, z_field)
    // }


    pub fn measure_temperature(&mut self, ecp5: &mut ECP5) -> f32 {
        let mut data: [u8;1] = [0];
        self.read_register(ecp5, Internal_control_0, &mut data);
        log::info!("Magnetometer: Internal_control_0: 0x{:X}", data[0]);
        self.write_register(ecp5, Internal_control_0, data[0] | (1 << 1));

        data = [0];

        while (data[0] >> 1) & 1 != 1 {
            self.read_register(ecp5, Status, &mut data);
            log::info!("Magnetometer: Wait for t meas done, status: 0x{:X}", data[0]);
        }

        self.read_register(ecp5, Tout, &mut data);
        let result = -75.0 + (data[0] as f32)*200.0/255.0;
        log::info!("Magnetometer: Temperature: {} -> {} deg C", data[0], result);
        result
    }

    // Expecting 0b00110000 = 0x30
    pub fn read_product_id(&mut self, ecp5: &mut ECP5) -> u8 {
        let mut data: [u8;1] = [0];
        self.read_register(ecp5, Product_ID_1, &mut data);
        data[0]
    }
}
