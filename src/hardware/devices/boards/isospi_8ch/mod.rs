use serde::Serialize;
use miniconf::Miniconf;
use crate::hardware::devices::boards::Devices;
use crate::hardware::ecp5::{SLOT, OFFSET_TO_SLOT, OFFSET_TO_SPI, SPI};
use crate::hardware::setup::BusReference;
use crate::hardware::devices::ic::mmc5983ma;
use crate::hardware::devices::ic::mmc5983ma::{Bandwidth, ContinuousMeasurementFrequency, PeriodicSet};

use embedded_hal::blocking::delay::DelayMs;

const sensor_count: usize = 8;

#[derive(Serialize, Clone, Copy)]
pub struct Telemetry {
    pub det_telemetry: [mmc5983ma::Telemetry; sensor_count],
}
impl Telemetry {
    pub fn new() -> Self {
        Self {
            det_telemetry: [mmc5983ma::Telemetry::default(); sensor_count],
        }
    }
}
impl Default for Telemetry {
    fn default() -> Self {
        Self{
            det_telemetry: [mmc5983ma::Telemetry::default(); sensor_count],
        }
    }
}

#[derive(Clone, Copy, Debug, Miniconf)]
pub struct Settings {
    ic_settings: [mmc5983ma::Settings; sensor_count],
    pub telemetry_period : u16,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            ic_settings: [mmc5983ma::Settings::default(); sensor_count],
            telemetry_period: 1,
        }
    }
}

pub struct IsoSPI_8ch {
    pub settings: Settings,
    pub telemetry: Telemetry,
    slot: u16,
    bus: BusReference,
    magnetometers: [mmc5983ma::Mmc5983ma; sensor_count],
    magnetometers_presence: [bool; sensor_count],
}

impl IsoSPI_8ch {
    pub fn new(
        slot: u16,
        bus: BusReference,
    ) -> Self {
        Self{
            settings: Settings::default(),
            telemetry: Telemetry::new(),
            bus,
            slot,
            magnetometers: [mmc5983ma::Mmc5983ma::new(slot); sensor_count],
            magnetometers_presence: [false; sensor_count],
        }
    }

    fn disable_iso_spi_sleep(&mut self){
        self.bus.lock(|bus| {
            let mut data: [u8;2] = [0, 0];
            // Set output
            // bus.ecp5.read_outputs(self.slot, &mut data);
            // log::info!("ECP5 slot {} output  read: 0x{:X}{:X}", self.slot, data[0], data[1]);
            data[1] = 1 << 7;
            bus.ecp5.write_outputs(self.slot, &mut data);
            log::info!("ECP5 slot {} output write: 0x{:X}{:X}", self.slot, data[0], data[1]);

            // Set pin to output
            bus.ecp5.read_oe(self.slot, &mut data);
            log::info!("ECP5 slot {} oe  read: 0x{:X}{:X}", self.slot, data[0], data[1]);
            data[1] = data[1] | 1 << 7;
            bus.ecp5.write_oe(self.slot, &mut data);
            log::info!("ECP5 slot {} oe write: 0x{:X}{:X}", self.slot, data[0], data[1]);
        })
    }

    // Not recommended, requires special wakeup sequence which is not implemented
    fn enable_iso_spi_sleep(&mut self){
        self.bus.lock(|bus| {
            let mut data: [u8;2] = [0, 0];
            // Set output
            // bus.ecp5.read_outputs(self.slot, &mut data);
            // log::info!("ECP5 slot {} output  read: 0x{:X}{:X}", self.slot, data[0], data[1]);
            data[1] = 0 << 7;
            bus.ecp5.write_outputs(self.slot, &mut data);
            log::info!("ECP5 slot {} output write: 0x{:X}{:X}", self.slot, data[0], data[1]);

            // Set pin to output
            bus.ecp5.read_oe(self.slot, &mut data);
            log::info!("ECP5 slot {} oe  read: 0x{:X}{:X}", self.slot, data[0], data[1]);
            data[1] = data[1] | 1 << 7;
            bus.ecp5.write_oe(self.slot, &mut data);
            log::info!("ECP5 slot {} oe write: 0x{:X}{:X}", self.slot, data[0], data[1]);
        })
    }

    // Configure SPI in ECP5
    fn init_interface(&mut self) -> () {
        let offset = self.slot * OFFSET_TO_SLOT + OFFSET_TO_SPI;
        self.bus.lock(|bus| {
            let mut data: [u8;2] = [0, 0];
            bus.ecp5.read_from_ecp5(160, &mut data); //expecting AAAA
            log::info!("ECP5 id1: 0x{:X}{:X}", data[0], data[1]);
            bus.ecp5.read_from_ecp5(161, &mut data); //expecting 5555
            log::info!("ECP5 id2: 0x{:X}{:X}", data[0], data[1]);
            bus.ecp5.read_from_ecp5(162, &mut data); //expecting 0002
            log::info!("ECP5 gateware rev: 0x{:X}{:X}", data[0], data[1]);

            bus.ecp5.read_from_ecp5(offset + SPI::LENGTH, &mut data);
            log::info!("ECP5 SPI length: 0x{:X}{:X}", data[0], data[1]);
            bus.ecp5.write_to_ecp5(offset + SPI::LENGTH, &[0x00, 0x0F]).unwrap();
            bus.ecp5.read_from_ecp5(offset + SPI::LENGTH, &mut data);
            log::info!("ECP5 SPI length: 0x{:X}{:X}", data[0], data[1]);
            bus.ecp5.write_to_ecp5(offset + SPI::CS, &[0x00, 0x01]).unwrap();
            bus.ecp5.write_to_ecp5(offset + SPI::CS_POL, &[0x00, 0x00]).unwrap();
            bus.ecp5.write_to_ecp5(offset + SPI::DIV, &[0x00, 0x90]).unwrap(); // Limited by isoSPI
            bus.ecp5.write_to_ecp5(offset + SPI::OFFLINE, &[0x00, 0x00]).unwrap();
            bus.ecp5.write_to_ecp5(offset + SPI::CLK_POL, &[0x00, 0x00]).unwrap();
            bus.ecp5.write_to_ecp5(offset + SPI::CLK_PHA, &[0x00, 0x00]).unwrap();
            bus.ecp5.write_to_ecp5(offset + SPI::LSB_FST, &[0x00, 0x00]).unwrap();
            bus.ecp5.write_to_ecp5(offset + SPI::HALF_DUP, &[0x00, 0x00]).unwrap();

            // Set encoding cs pins to output
            bus.ecp5.read_oe(self.slot, &mut data);
            log::info!("ECP5 slot {} oe  read: 0x{:X}{:X}", self.slot, data[0], data[1]);
            data[1] = data[1] | (0x07 << 4);
            bus.ecp5.write_oe(self.slot, &mut data);
            log::info!("ECP5 slot {} oe write: 0x{:X}{:X}", self.slot, data[0], data[1]);
        })
    }

    fn encode_cs(&mut self, device_num: u8) -> () {
        let device_num_masked = device_num & 0x07;
        // log::info!("ISO SPI: Encode CS: {}", device_num_masked);
        self.bus.lock(|bus| {
            let mut data: [u8;2] = [0, 0];
            // Set output
            bus.ecp5.read_outputs(self.slot, &mut data);
            // log::info!("ECP5 slot {} output read: 0x{:X}{:X}", self.slot, data[0], data[1]);
            data[1] = data[1] & !(0x07 << 4);
            data[1] = data[1] | (device_num_masked << 4);
            bus.ecp5.write_outputs(self.slot, &mut data);
            // log::info!("ECP5 slot {} output write: 0x{:X}{:X}", self.slot, data[0], data[1]);
        })
    }
}


impl Devices<Settings, Telemetry> for IsoSPI_8ch {
    fn init(&mut self) -> bool {
        let mut delay = asm_delay::AsmDelay::new(asm_delay::bitrate::Hertz(
            400000000,
        ))  ;
        self.init_interface();
        self.disable_iso_spi_sleep();

        self.settings_update(Settings::default());
        
        for i in 0..sensor_count {
            self.encode_cs(i as u8);
            self.bus.lock(|bus| {
                let mut result: u8;
                // log::info!("Magnetometer: bus lock acquired");
                self.magnetometers[i].reset_device(&mut bus.ecp5);
                delay.delay_ms(10 as u32);
                result = self.magnetometers[i].read_product_id(&mut bus.ecp5);
                // log::info!("Magnetometer {}: Read product id: {:X}", i, result);
                
                if result == 0x30 {
                    self.magnetometers_presence[i] = true;
                    self.magnetometers[i].remove_bridge_offset(&mut bus.ecp5);
                    // For now, use defaults
                    // self.magnetometers[i].set_x_inhibit(&mut bus.ecp5, false);
                    // self.magnetometers[i].set_yz_inhibit(&mut bus.ecp5, true);
                    // self.magnetometers[i].set_z_inhibit(&mut bus.ecp5, false);
                }
            });
        };
        true
    }

    fn settings_update(&mut self, new_settings: Settings) -> () {
        log::info!("Magnetometer: Settigns update");
        self.settings = new_settings;

        for i in 0..sensor_count {
            if self.magnetometers_presence[i] && self.settings.ic_settings[i].enable {
                self.encode_cs(i as u8);
                self.bus.lock(|bus| {
                    if self.settings.ic_settings[i].bridge_offset_calculation {
                        self.magnetometers[i].remove_bridge_offset(&mut bus.ecp5);
                    }
                    self.settings.ic_settings[i].bridge_offset_calculation = false;
                    self.magnetometers[i].set_continuous_mode(&mut bus.ecp5, self.settings.ic_settings[i].continuous_measurement_frequency);
                    self.magnetometers[i].set_periodic_set(&mut bus.ecp5, self.settings.ic_settings[i].periodic_set_frequency);
                    self.magnetometers[i].set_meas_bandwidth(&mut bus.ecp5, self.settings.ic_settings[i].bandwidth);
                })
            }
        }
    }

    fn telemetry(&mut self) -> (Telemetry, u16) {
        for i in 0..sensor_count {
            if self.magnetometers_presence[i] && self.settings.ic_settings[i].enable {
                self.encode_cs(i as u8);
                self.bus.lock(|bus| {
                    // log::info!("Magnetometer {}: bus lock acquired", i);
                    
                    let mut result: (f32, f32, f32);
                    if self.settings.ic_settings[i].continuous_measurement_frequency == ContinuousMeasurementFrequency::CM_Off {
                        // self.telemetry.det_telemetry[i].t = self.magnetometers[i].measure_temperature(&mut bus.ecp5);
                        result = self.magnetometers[i].measure_m_field(&mut bus.ecp5);
                    }
                    else {
                        result = self.magnetometers[i].read_m_field_w_offsets(&mut bus.ecp5);
                    }
                    self.telemetry.det_telemetry[i].x = result.0;
                    self.telemetry.det_telemetry[i].y = result.1;
                    self.telemetry.det_telemetry[i].z = result.2;
                });
            }
        }
        (self.telemetry, self.settings.telemetry_period)
    }

    fn check_interrupt(&mut self) -> () {}

    fn poll(&mut self) -> u32 {0}
}
