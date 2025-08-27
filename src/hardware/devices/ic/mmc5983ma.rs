pub use miniconf::Miniconf;
use serde::Serialize;
use serde::Deserialize;
use crate::hardware::ecp5::{self, ECP5};
use crate::hardware::setup::BusReference;
use embedded_hal::blocking::delay::DelayMs;

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

#[derive(Copy, Clone, Debug, Serialize, Deserialize, Miniconf)]
pub enum Bandwidth {
    BW100Hz,
    BW200Hz,
    BW400Hz,
    BW800Hz,
}

#[derive(Copy, Clone, Debug, Serialize, Deserialize, Miniconf, PartialEq)]
pub enum ContinuousMeasurementFrequency {
    CM_Off,
    CM_1Hz,
    CM_10Hz,
    CM_20Hz,
    CM_50Hz,
    CM_100Hz,
    CM_200Hz,
    CM_1000Hz,
}

#[derive(Copy, Clone, Debug, Serialize, Deserialize, Miniconf, PartialEq)]
pub enum PeriodicSet {
    Off,
    SetEvery1Meas,
    SetEvery25Meas,
    SetEvery75Meas,
    SetEvery100Meas,
    SetEvery250Meas,
    SetEvery500Meas,
    SetEvery1000Meas,
    SetEvery2000Meas,
}

#[derive(Serialize, Default, Clone, Copy)]
pub struct Telemetry {
    pub t: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub present: bool,
}

#[derive(Clone, Copy, Debug, Miniconf)]
pub struct Settings {
    /// If device should be used
    pub enable: bool,
    /// Device will be reset once (only if enabled)
    // pub reset_device: bool,
    /// Perform bridge offset calculation once
    pub bridge_offset_calculation: bool,
    /// Measurement bandwidth
    pub bandwidth: Bandwidth,
    /// How often should automatic measurement be performed
    pub continuous_measurement_frequency: ContinuousMeasurementFrequency,
    /// How often should automatic SET operation be performed (every N measurements)
    pub periodic_set_frequency: PeriodicSet,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            enable: true,
            // reset_device: false,
            bridge_offset_calculation: false,
            bandwidth: Bandwidth::BW100Hz,
            continuous_measurement_frequency: ContinuousMeasurementFrequency::CM_Off,
            periodic_set_frequency: PeriodicSet::SetEvery1000Meas,
        }
    }
}

#[derive(Clone, Copy)]
pub struct Mmc5983ma {
    slot: u16,
    internal_control_registers: [u8; 4],
    offset: [f32; 3],
}

impl Mmc5983ma {
    pub fn new(
        slot_number: u16,
    ) -> Self {
        Self {
            slot: slot_number,
            internal_control_registers: [0; 4],
            offset: [0.0; 3],
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

    pub fn set_meas_bandwidth(&mut self, ecp5: &mut ECP5, bandwidth: Bandwidth) {
        let register: u8 = match bandwidth {
            Bandwidth::BW100Hz => 0,
            Bandwidth::BW200Hz => 1,
            Bandwidth::BW400Hz => 2,
            Bandwidth::BW800Hz => 3,
        };

        // log::info!("Magnetometer: set_meas_bandwidth: data before: {:b}", self.internal_control_registers[1]);
        self.internal_control_registers[1] = (self.internal_control_registers[1] & !(0b11)) | register;
        // log::info!("Magnetometer: set_meas_bandwidth: data after: {:b}", self.internal_control_registers[1]);

        self.write_register(ecp5, INTERNAL_CONTROL_1, self.internal_control_registers[1]);
    }

    pub fn set_continuous_mode(&mut self, ecp5: &mut ECP5, freq: ContinuousMeasurementFrequency) {
        let freq_register: u8 = match freq {
            ContinuousMeasurementFrequency::CM_Off => 0,
            ContinuousMeasurementFrequency::CM_1Hz => 1,
            ContinuousMeasurementFrequency::CM_10Hz => 2,
            ContinuousMeasurementFrequency::CM_20Hz => 3,
            ContinuousMeasurementFrequency::CM_50Hz => 4,
            ContinuousMeasurementFrequency::CM_100Hz => 5,
            ContinuousMeasurementFrequency::CM_200Hz => 6,
            ContinuousMeasurementFrequency::CM_1000Hz => 7,
        };

        // log::info!("Magnetometer: set_continuous_mode: data before: {:b}", self.internal_control_registers[2]);
        let enable: bool = freq != ContinuousMeasurementFrequency::CM_Off;
        self.internal_control_registers[2] = (self.internal_control_registers[2] & !(1 << 3)) | ((enable as u8) << 3);
        self.internal_control_registers[2] = (self.internal_control_registers[2] & !(0b111)) | freq_register;
        // log::info!("Magnetometer: set_continuous_mode: data after: {:b}", self.internal_control_registers[2]);

        self.write_register(ecp5, INTERNAL_CONTROL_2, self.internal_control_registers[2]);
        // log::info!("Magnetometer: set_continuous_mode frequency: {} enable: {}", freq_register, enable);

        // Interrupt must be enabled for continuous mode to work
        self.enable_interrupt(ecp5, enable);
    }

    // These functions also include taking a measurement
    pub fn magnetic_set(&mut self, ecp5: &mut ECP5) {
        self.write_register(ecp5, INTERNAL_CONTROL_0, self.internal_control_registers[0] | (1 << 3) | 1);
    }

    pub fn magnetic_reset(&mut self, ecp5: &mut ECP5) {
        self.write_register(ecp5, INTERNAL_CONTROL_0, self.internal_control_registers[0] | (1 << 4) | 1);
    }

    pub fn remove_bridge_offset(&mut self, ecp5: &mut ECP5) {
        self.magnetic_set(ecp5);
        let set_result = self.read_m_field(ecp5);
        self.magnetic_reset(ecp5);
        let reset_result = self.read_m_field(ecp5);
        self.offset[0] = (set_result.0 + reset_result.0) / 2.0;
        self.offset[1] = (set_result.1 + reset_result.1) / 2.0;
        self.offset[2] = (set_result.2 + reset_result.2) / 2.0;
        log::info!("Magnetometer: offsets: {} {} {}", self.offset[0], self.offset[1], self.offset[2],);
    }

    pub fn set_periodic_set(&mut self, ecp5: &mut ECP5, freq: PeriodicSet) {
        // Frequency is in Hz
        let freq_register: u8 = match freq {
            PeriodicSet::Off => 0,
            PeriodicSet::SetEvery1Meas => 0,
            PeriodicSet::SetEvery25Meas => 1,
            PeriodicSet::SetEvery75Meas => 2,
            PeriodicSet::SetEvery100Meas => 3,
            PeriodicSet::SetEvery250Meas => 4,
            PeriodicSet::SetEvery500Meas => 5,
            PeriodicSet::SetEvery1000Meas => 6,
            PeriodicSet::SetEvery2000Meas => 7,
        };

        let enable: bool = freq != PeriodicSet::Off;
        // log::info!("Magnetometer: set_periodic_set: data before: {:b}", self.internal_control_registers[2]);
        self.internal_control_registers[2] = (self.internal_control_registers[2] & !(1 << 7)) | ((enable as u8) << 7);
        self.internal_control_registers[2] = (self.internal_control_registers[2] & !(0b111 << 4)) | (freq_register << 4);
        // log::info!("Magnetometer: set_periodic_set: data after: {:b}", self.internal_control_registers[2]);

        self.write_register(ecp5, INTERNAL_CONTROL_2, self.internal_control_registers[2]);
        // log::info!("Magnetometer: set_periodic_set frequency: {} enable: {}", freq, enable);

        self.internal_control_registers[0] = self.internal_control_registers[0] | ((enable as u8) << 5);
        self.write_register(ecp5, INTERNAL_CONTROL_0, self.internal_control_registers[0]);
    }

    pub fn reset_device(&mut self, ecp5: &mut ECP5) {
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
        self.write_register(ecp5, INTERNAL_CONTROL_0, self.internal_control_registers[0] | 1);
        self.read_m_field_w_offsets(ecp5)
    }

    pub fn read_m_field(&mut self, ecp5: &mut ECP5) -> (f32, f32, f32) {
        let mut delay = asm_delay::AsmDelay::new(asm_delay::bitrate::Hertz(
            400000000,
        ));
        let mut data = [0];

        while (data[0] & 1) != 1 {
            self.read_register(ecp5, STATUS, &mut data);
            // log::info!("Magnetometer: Wait for m meas done, status: 0x{:X}", data[0]);
            delay.delay_ms(1 as u32);
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

    pub fn read_m_field_w_offsets(&mut self, ecp5: &mut ECP5) -> (f32, f32, f32) {
        let result = self.read_m_field(ecp5);
        let x_field = result.0 - self.offset[0];
        let y_field = result.1 - self.offset[1];
        let z_field = result.2 - self.offset[2];

        // log::info!("Magnetometer: Field w/ offsets: X {} Y {} Z {}", x_field, y_field, z_field);
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
            delay.delay_ms(1 as u32);
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
