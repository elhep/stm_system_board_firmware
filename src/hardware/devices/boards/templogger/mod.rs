use serde::Serialize;
use miniconf::Miniconf;
use crate::hardware::devices::boards::Devices;
use crate::hardware::ecp5::{OFFSET_TO_SLOT, OFFSET_TO_SPI, SPI};
use crate::hardware::setup::BusReference;
use crate::hardware::devices::ic::max31865;



#[derive(Serialize)]
pub struct Telemetry {
    pub det_telemetry: [max31865::Telemetry;16]
}
impl Default for Telemetry {
    fn default() -> Self {
        Self{
            det_telemetry: [max31865::Telemetry::default(); 16]
        }
    }
}

#[derive(Copy, Clone)]
pub struct TelemetryBuffer {
    det: [max31865::TempTelemetryBuffer; 16]
}

impl TelemetryBuffer {
    pub fn finalize(self, settings: Settings, detectors: &mut [max31865::Max31865; 16]) -> Telemetry {
        let mut telemetry = Telemetry::default();
        for (i, det) in self.det.iter().enumerate() {
            if settings.max_settings[i].active {
                telemetry.det_telemetry[i].temp = detectors[i].calculate_pt100(det.temp);
                telemetry.det_telemetry[i].status = det.status;
            }
        }
        telemetry
    }

    pub fn new() -> Self {
        Self {
            det: [max31865::TempTelemetryBuffer::default(); 16]
        }
    }
}

#[derive(Clone, Copy, Debug, Miniconf, PartialEq)]
pub struct Settings {
    max_settings: [max31865::Settings; 16],
    pub telemetry_period : u16,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            max_settings: [max31865::Settings::default(); 16],
            telemetry_period: 10,
        }
    }
}

pub struct TempLogger {
    pub settings: Settings,
    pub telemetry: TelemetryBuffer,
    bus: BusReference,
    slot_number: u16,
    detectors: [max31865::Max31865; 16],
}

impl TempLogger {
    pub fn new(
        slot_number: u16,
        bus: BusReference,
    ) -> Self {
        Self{
            settings: Settings::default(),
            telemetry: TelemetryBuffer::new(),
            bus,
            slot_number,
            detectors: [max31865::Max31865::new(slot_number); 16],
        }
    }
}

impl TempLogger {
    fn init_interface(&mut self) -> () {
        self.bus.lock(|bus| {
            bus.ecp5.write_oe(self.slot_number, &[0b0000_0000, 0b1111_0000]);
            bus.ecp5.write_outputs(self.slot_number, &[0b0000_0000, 0b0000_0000]);
            // bus.ecp5.set_spi_cs(my_slot, 0b0000_0000_1010_1000);
            // bus.ecp5.write_spi_cs_at_reg(my_slot, &[0b0000_0000, 0b1010_1000]);
            bus.ecp5.write_to_ecp5(OFFSET_TO_SLOT * self.slot_number + OFFSET_TO_SPI + SPI::LENGTH, &[0x00, 0x07]).unwrap();
            bus.ecp5.write_to_ecp5(OFFSET_TO_SLOT * self.slot_number + OFFSET_TO_SPI + SPI::CS_POL, &mut [0x00, 0x01]).unwrap(); // TODO BUG on PCB, polarity of P and N signal switched
            bus.ecp5.write_to_ecp5(OFFSET_TO_SLOT * self.slot_number + OFFSET_TO_SPI + SPI::CS, &mut [0x00, 0x01]).unwrap();
            bus.ecp5.write_to_ecp5(OFFSET_TO_SLOT * self.slot_number + OFFSET_TO_SPI + SPI::DIV, &mut [0x00, 0xA0]).unwrap();
            bus.ecp5.write_to_ecp5(OFFSET_TO_SLOT * self.slot_number + OFFSET_TO_SPI + SPI::OFFLINE, &mut [0x00, 0x00]).unwrap();
            bus.ecp5.write_to_ecp5(OFFSET_TO_SLOT * self.slot_number + OFFSET_TO_SPI + SPI::CLK_POL, &mut [0x00, 0x00]).unwrap();
            bus.ecp5.write_to_ecp5(OFFSET_TO_SLOT * self.slot_number + OFFSET_TO_SPI + SPI::CLK_PHA, &mut [0x00, 0x01]).unwrap();
            bus.ecp5.write_to_ecp5(OFFSET_TO_SLOT * self.slot_number + OFFSET_TO_SPI + SPI::LSB_FST, &mut [0x00, 0x00]).unwrap();
            bus.ecp5.write_to_ecp5(OFFSET_TO_SLOT * self.slot_number + OFFSET_TO_SPI + SPI::HALF_DUP, &mut [0x00, 0x00]).unwrap();
        })
    }

    fn init_readout(&mut self, mut channel: u8) -> () {
        // let fpga_outputs = [0b0000_0000, channel << 4];
        self.select_cs(channel);
        self.bus.lock(|bus| {
            // bus.ecp5.write_outputs(self.slot_number, &fpga_outputs);
            self.detectors[channel as usize].set_register(&mut bus.ecp5, max31865::REGS::CONFIGURATION, 0b1010_0001)
        });
    }

    fn read_temp(&mut self, mut channel: u8) -> u16 {
        // let fpga_outputs = [0b0000_0000, channel << 4];
        self.select_cs(channel);
        self.bus.lock(|bus| {
            // bus.ecp5.write_outputs(self.slot_number, &fpga_outputs);
            let mut value = [0; 2];
            self.detectors[channel as usize].read_register(&mut bus.ecp5, max31865::REGS::RTDMSB, &mut value);
            let temp = ((value[0] as u16) << 7) | ((value[1] as u16) >> 1);
            temp
        })
    }

    fn select_cs(&mut self, nb: u8) -> () {
        // Check if last transfer is finished
        let mut array : [u8; 2] = [0x00, 0x00];
        let mut address = OFFSET_TO_SLOT * self.slot_number + OFFSET_TO_SPI + SPI::IDLE;

        let mut cs = [0b0000_0000, 0b0000_0000];
        cs[1] |= nb << 4; // Set the chip select for the given channel
        self.bus.lock(|bus| {
            while array[1] != 1 {
                bus.ecp5.read_from_ecp5(address, &mut array).unwrap();
            }
            bus.ecp5.write_outputs(self.slot_number, &cs);
        });
    }
}

impl Devices<Settings, Telemetry> for TempLogger {
    fn init(&mut self) -> bool {
        self.init_interface();
        for i in 0..16 {
            self.select_cs(i as u8);
            self.bus.lock(|bus| {
                self.detectors[i].set_register(&mut bus.ecp5, max31865::REGS::CONFIGURATION, 0b0000_0001); //TODO deactivate all detectors
            });
        };

        true
    }

    fn settings_update(&mut self, new_settings: Settings) -> () {
        // for (i, detector) in self.detectors.iter_mut().enumerate() {
        for i in 0..16 {
            if self.settings.max_settings[i] != new_settings.max_settings[i] {
                self.select_cs(i as u8);
                self.detectors[i].settigns_update(new_settings.max_settings[i].a, new_settings.max_settings[i].b, new_settings.max_settings[i].r_nom, new_settings.max_settings[i].r_ref);
                self.bus.lock(|bus| {                
                    self.detectors[i].update_configuration(&mut bus.ecp5, new_settings.max_settings[i], false, false);
                    if self.settings.max_settings[i].high_threshold != new_settings.max_settings[i].high_threshold {
                        self.detectors[i].set_register(&mut bus.ecp5, max31865::REGS::HTRESHOLDMSB, (new_settings.max_settings[i].high_threshold >> 7) as u8);
                        self.detectors[i].set_register(&mut bus.ecp5, max31865::REGS::HTRESHOLDLSB, (new_settings.max_settings[i].high_threshold & 0xFF) as u8);
                    }
                    if self.settings.max_settings[i].low_threshold != new_settings.max_settings[i].low_threshold {
                        self.detectors[i].set_register(&mut bus.ecp5, max31865::REGS::LTHRESHOLDMSB, (new_settings.max_settings[i].low_threshold >> 7) as u8);
                        self.detectors[i].set_register(&mut bus.ecp5, max31865::REGS::LTHRESHOLDLSB, (new_settings.max_settings[i].low_threshold & 0xFF) as u8);
                    }
                });
                self.settings.max_settings[i] = new_settings.max_settings[i];
            }
        }
        self.settings.telemetry_period = new_settings.telemetry_period;
    }

    fn telemetry(&mut self) -> (Telemetry, u16) {
        // Start Fault Status Check
        for i in 0..16 {
            if self.settings.max_settings[i].active {
                    self.select_cs(i as u8);
                    self.bus.lock(|bus| {   
                        self.detectors[i].update_configuration(&mut bus.ecp5, self.settings.max_settings[i], true, false);
                    });
                }
            }

        // Chceck first active detector if it finished Fault Status Check
        let mut first_active = 16;
        for i in 0..16 {
            if self.settings.max_settings[i].active {
                first_active = i;
                break;
            };
        }

        if first_active != 16 {
            self.select_cs(first_active as u8);
            self.bus.lock(|bus| { 
                    let mut value = [0b100];
                    while value[0] & 0b0000_0100 != 0 {
                        // Wait for fault status check to finish
                        self.detectors[first_active].read_register(&mut bus.ecp5, max31865::REGS::CONFIGURATION, &mut value);
                    }
            });
        }

        
        // Read all Fault Status registers and clear them
        for i in 0..16 {
            if self.settings.max_settings[i].active {
                self.select_cs(i as u8);
                self.bus.lock(|bus| {
                    let mut value = [0; 1];
                    self.detectors[i].read_register(&mut bus.ecp5, max31865::REGS::FALUTSTATUS, &mut value);
                    self.telemetry.det[i].status = value[0];
                    let value = 0b1100_0010 | self.settings.max_settings[i].filter as u8;
                    self.detectors[i].update_configuration(&mut bus.ecp5, self.settings.max_settings[i], false, true);
                });
            } 
        }

        // Read Temperature
        for i in 0..16 {
            if self.settings.max_settings[i].active {
                // self.select_cs(i as u8); // Read temp function has already select_cs
                self.telemetry.det[i].temp = self.read_temp(i as u8);
            }
        }

        return (self.telemetry.finalize(self.settings, &mut self.detectors), self.settings.telemetry_period)
    }

    fn check_interrupt(&mut self) -> () {
        
    }

    fn poll(&mut self) -> u32 {
        0
    }

}