use serde::Serialize;
use miniconf::Miniconf;
use crate::hardware::devices::boards::Devices;
use crate::hardware::ecp5::{SLOT, OFFSET_TO_SLOT, OFFSET_TO_SPI, SPI};
use crate::hardware::setup::BusReference;
use crate::hardware::devices::ic::mmc5983ma;

use embedded_hal::blocking::delay::DelayMs;

const sensor_count: usize = 8;

#[derive(Serialize, Clone, Copy)]
pub struct Telemetry {
    pub det_telemetry: [mmc5983ma::Telemetry; sensor_count]
}
impl Telemetry {
    pub fn new() -> Self {
        Self {
            det_telemetry: [mmc5983ma::Telemetry::default(); sensor_count]
        }
    }
}
impl Default for Telemetry {
    fn default() -> Self {
        Self{
            det_telemetry: [mmc5983ma::Telemetry::default(); sensor_count]
        }
    }
}

// #[derive(Copy, Clone)]
// pub struct TelemetryBuffer {
//     det: [mmc5983ma::TempTelemetryBuffer; sensor_count]
// }

// impl TelemetryBuffer {
//     pub fn finalize(self, settings: Settings, detectors: &mut [mmc5983ma::Mmc5983ma; sensor_count]) -> Telemetry {
//         let mut telemetry = Telemetry::default();
//         // for (i, det) in self.det.iter().enumerate() {
//         //     if settings.max_settings[i].active {
//         //         telemetry.det_telemetry[i].temp = detectors[i].calculate_pt100(det.temp);
//         //         telemetry.det_telemetry[i].status = det.status;
//         //     }
//         // }
//         telemetry
//     }

//     pub fn new() -> Self {
//         Self {
//             det: [mmc5983ma::TempTelemetryBuffer::default(); sensor_count]
//         }
//     }
// }


#[derive(Clone, Copy, Debug, Miniconf, PartialEq)]
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
        }
    }

    // Enable ISO SPI
    fn enable_iso_spi(&mut self){
        let offset = self.slot * OFFSET_TO_SLOT;

        self.bus.lock(|bus| {
            let mut data: [u8;2] = [0, 0];
            // Set output
            // bus.ecp5.read_from_ecp5(offset + SLOT::OUTPUT, &mut data);
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
            bus.ecp5.write_to_ecp5(offset + SPI::DIV, &[0x00, 0x64]).unwrap();
            bus.ecp5.write_to_ecp5(offset + SPI::OFFLINE, &[0x00, 0x00]).unwrap();
            bus.ecp5.write_to_ecp5(offset + SPI::CLK_POL, &[0x00, 0x01]).unwrap();
            bus.ecp5.write_to_ecp5(offset + SPI::CLK_PHA, &[0x00, 0x01]).unwrap();
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
        let offset = self.slot * OFFSET_TO_SLOT + OFFSET_TO_SPI;
        let device_num_masked = device_num & 0x07;
        // log::info!("ISO SPI: Encode CS: {}", device_num_masked);
        self.bus.lock(|bus| {
            let mut data: [u8;2] = [0, 0];
            // Set output
            bus.ecp5.read_from_ecp5(offset + SLOT::OUTPUT, &mut data);
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
        log::info!("ISO SPI: Begin init");
        self.init_interface();
        log::info!("ISO SPI: SPI setup");
        self.enable_iso_spi();
        log::info!("ISO SPI: Enabled");        
        
        for i in 0..sensor_count {
            self.encode_cs(i as u8);
            self.bus.lock(|bus| {
                let mut result: u8;
                // log::info!("Magnetometer: bus lock acquired");
                self.magnetometers[i].reset(&mut bus.ecp5);
                delay.delay_ms(20 as u32);
                result = self.magnetometers[i].read_product_id(&mut bus.ecp5);
                log::info!("Magnetometer {}: Read product id: {:X}", i, result);
                if result == 0x30 {
                    self.magnetometers[i].settings.active = true;                
                    self.magnetometers[i].set_continuous_mode(&mut bus.ecp5, 10, true);
                    self.magnetometers[i].set_x_inhibit(&mut bus.ecp5, false);
                    self.magnetometers[i].set_y_inhibit(&mut bus.ecp5, false);
                    self.magnetometers[i].set_z_inhibit(&mut bus.ecp5, false);
                }
            });
        };
        true
    }

    fn settings_update(&mut self, new_settings: Settings) -> () {
        return
    }

    fn telemetry(&mut self) -> (Telemetry, u16) {
        log::info!("Magnetometer: telemetry spawn");
        // let mut telemetry = Telemetry::default();
        for i in 0..sensor_count {
            if self.magnetometers[i].settings.active {
                self.encode_cs(i as u8);
                self.bus.lock(|bus| {
                    log::info!("Magnetometer {}: bus lock acquired", i);
                    
                    self.telemetry.det_telemetry[i].temp = self.magnetometers[i].measure_temperature(&mut bus.ecp5);
                // let result = Mmc5983ma::read_m_field(self.slot, &mut bus.ecp5);
                // telemetry.x_field = result.0;
                // telemetry.y_field = result.1;
                // telemetry.z_field = result.2;
                });
            }
        }
        (self.telemetry, self.settings.telemetry_period)
    }

    fn check_interrupt(&mut self) -> () {}

    fn poll(&mut self) -> u32 {0}
}