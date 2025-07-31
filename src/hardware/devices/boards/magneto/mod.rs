use embedded_hal::digital::v2::OutputPin;
use heapless::binary_heap;
use serde::Serialize;
use miniconf::Miniconf;
use crate::hardware;
use crate::hardware::devices::boards::Devices;
use crate::hardware::ecp5::{ECP5, OFFSET_TO_SLOT, OFFSET_TO_SPI, SPI};
use crate::hardware::setup::{BusReference, SlotsBus};
use crate::hardware::devices::ic::mcp23s08::{Mcp23s08, REGS};
use crate::net::{settings, telemetry};
use embedded_hal::blocking::delay::DelayUs;


#[derive(Copy, Clone)]
pub struct TelemetryBuffer {
    v_p_limit : bool,
    v_n_limit : bool,
    pwr_limit : bool,
    pwr_limit_avg: bool,
    temp: f32,
    adc: [u16; 4]
}

impl Default for TelemetryBuffer{
    fn default() -> Self {
        Self { 
            v_p_limit : false,
            v_n_limit : false,
            pwr_limit: false, 
            pwr_limit_avg: false, 
            temp: 0.0, 
            adc: [0;4]
        }
    }
}

impl TelemetryBuffer{
    pub fn finalize(self) -> Telemetry{
        Telemetry { 
            v_p_limit: self.v_p_limit,
            v_n_limit: self.v_n_limit,
            pwr_limit: self.pwr_limit, 
            pwr_limit_avg: self.pwr_limit_avg, 
            temp: self.temp, 
            adc: self.convert_adc(),
            amp: (self.adc[0] as f32 * 5.0 / 0xFFFF as f32) * 2.0 - 5.0
        }
    }

    fn convert_adc(self) -> [f32; 4] {
        [
            self.adc[0] as f32 * 5.0 / 0xFFFF as f32,
            self.adc[1] as f32 * 5.0 / 0xFFFF as f32,
            self.adc[2] as f32 * 5.0 / 0xFFFF as f32,
            self.adc[3] as f32 * 5.0 / 0xFFFF as f32,
        ]
    }
}

#[derive(Serialize, Default)]
pub struct Telemetry {
    v_p_limit : bool,
    v_n_limit : bool,
    pwr_limit : bool,
    pwr_limit_avg: bool,
    temp: f32,
    adc: [f32; 4],
    amp: f32
}



#[derive(Clone, Copy, Debug, Miniconf, PartialEq)]
pub struct Settings {
    dac_value   : u16,
    vsel        : [bool; 2],
    ref_sel     : [bool; 2],
    pub telemetry_period : u16,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            dac_value: 32800,
            vsel : [true; 2],
            ref_sel : [true; 2],
            telemetry_period: 1
        }
    }
}

pub struct Magneto{
    pub settings: Settings,
    pub telemetry: TelemetryBuffer,
    bus: BusReference,
    io_expander: Mcp23s08,
    slot_number: u16,
    dac_cs_nb:   u16,
    adc_cs_nb:   u16,
    delay: asm_delay::AsmDelay
}

impl Magneto {
    pub fn new(
        slot_number: u16,
        bus: BusReference
    ) -> Self {
        Self {
            settings: Settings::default(),
            telemetry: TelemetryBuffer::default(),
            bus,
            io_expander: Mcp23s08::new(0, slot_number),
            slot_number,
            dac_cs_nb: 3,
            adc_cs_nb: 7,
            delay: asm_delay::AsmDelay::new(asm_delay::bitrate::Hertz(
                400000000,
            )) 
        }
    }
}

impl Magneto {
    fn init_interface(&mut self) -> () {
        self.bus.lock(|bus| {
            bus.ecp5.write_oe(self.slot_number, &[0b0000_0000, 0b1011_0000]);
            bus.ecp5.write_outputs(self.slot_number, &[0b0000_0000, 0b1010_0000]);
            // bus.ecp5.set_spi_cs(my_slot, 0b0000_0000_1010_1000);
            // bus.ecp5.write_spi_cs_at_reg(my_slot, &[0b0000_0000, 0b1010_1000]);
            bus.ecp5.write_to_ecp5(OFFSET_TO_SLOT * self.slot_number + OFFSET_TO_SPI + SPI::LENGTH, &[0x00, 0x07]).unwrap();
            bus.ecp5.write_to_ecp5(OFFSET_TO_SLOT * self.slot_number + OFFSET_TO_SPI + SPI::CS_POL, &mut [0x00, 0x00]).unwrap();
            bus.ecp5.write_to_ecp5(OFFSET_TO_SLOT * self.slot_number + OFFSET_TO_SPI + SPI::CS, &mut [0x00, 0x00]).unwrap();
            bus.ecp5.write_to_ecp5(OFFSET_TO_SLOT * self.slot_number + OFFSET_TO_SPI + SPI::DIV, &mut [0x00, 0xA0]).unwrap();
            bus.ecp5.write_to_ecp5(OFFSET_TO_SLOT * self.slot_number + OFFSET_TO_SPI + SPI::OFFLINE, &mut [0x00, 0x00]).unwrap();
            bus.ecp5.write_to_ecp5(OFFSET_TO_SLOT * self.slot_number + OFFSET_TO_SPI + SPI::CLK_POL, &mut [0x00, 0x00]).unwrap();
            bus.ecp5.write_to_ecp5(OFFSET_TO_SLOT * self.slot_number + OFFSET_TO_SPI + SPI::CLK_PHA, &mut [0x00, 0x00]).unwrap();
            bus.ecp5.write_to_ecp5(OFFSET_TO_SLOT * self.slot_number + OFFSET_TO_SPI + SPI::LSB_FST, &mut [0x00, 0x00]).unwrap();
            bus.ecp5.write_to_ecp5(OFFSET_TO_SLOT * self.slot_number + OFFSET_TO_SPI + SPI::HALF_DUP, &mut [0x00, 0x00]).unwrap();
        });
    }
    fn write_io_extender(&mut self, reg: u8, value: u8) -> () {
        self.bus.lock(|bus| {
            bus.ecp5.write_outputs(self.slot_number, &[0b0000_0000, 0b1100_1111]);
            self.io_expander.set_register(&mut bus.ecp5, reg,value);
            bus.ecp5.w8_for_spi(self.slot_number);
            bus.ecp5.write_outputs(self.slot_number, &[0b0000_0000, 0b1110_1111]);
        });
    }

    fn read_io_extender(&mut self, reg: u8, value: &mut [u8]) -> () {
        self.bus.lock(|bus| {
            bus.ecp5.write_outputs(self.slot_number, &[0b0000_0000, 0b1100_1111]);
            self.io_expander.read_register(&mut bus.ecp5, reg, value);
            bus.ecp5.write_outputs(self.slot_number, &[0b0000_0000, 0b1110_1111]);
        });
    }

    fn read_adc(&mut self, mut channel: u8) -> [u8; 2] {
        let mut code: [u8; 2] = [0; 2];
        channel &= 0b11;
        self.bus.lock(|bus| {
            bus.ecp5.write_outputs(self.slot_number, &[0b0000_0000, 0b0110_1111]);
            bus.ecp5.read_spi_parallel_write(self.slot_number, &[0b1111_0001 | (channel<<1), 0b1100_0100], &mut code);
            bus.ecp5.w8_for_spi(self.slot_number);
            bus.ecp5.write_outputs(self.slot_number, &[0b0000_0000, 0b1110_1111]);
        });
        code
    }

    fn update_io_extender(&mut self, vsel: [bool; 2], ref_sel: [bool; 2]) -> () {
        let mut value = 0;
        if vsel[0]{
            value |= (1 << 3);
        }
        if vsel[1]{
            value |= (1 << 2);
        }
        if ref_sel[0]{
            value |= (1 << 1);
        }
        if ref_sel[1]{
            value |= 1;
        }
        self.write_io_extender(REGS::OLAT, value);
    }

    fn write_dac(&mut self, value: &[u8]) -> () {
        self.bus.lock(|bus| {
            bus.ecp5.set_spi_cs(self.slot_number, 1);
            bus.ecp5.write_spi(self.slot_number, value);
            bus.ecp5.w8_for_spi(self.slot_number);
            bus.ecp5.set_spi_cs(self.slot_number, 0);
        });
    }

    fn read_temp(&mut self) -> f32 {
        self.bus.lock(|bus| {
            bus.set_servmod(self.slot_number);
            let mut t= 0.0;
            match hardware::devices::ic::lm75a::read_temp(&mut bus.cpcis_i2c, 0b1001000 as u8) {
                Ok(temp) => t = temp,
                Err(_e) => log::error!("Magneto {}: 0", self.slot_number) 
            }
            bus.clear_servmod();

            return t
        })
    }
}

impl Devices<Settings, Telemetry> for Magneto {
    fn init(&mut self) -> bool {
            self.init_interface();
            // IO extender:
            // Set bits 0-3 as outputs  (default 1), bits 4-7 as inputs (default 1)
            self.write_io_extender(REGS::OLAT, 0x0F);   
            self.write_io_extender(REGS::IODIR, 0xF0);
            let mut readout = [0];
            self.read_io_extender(REGS::IODIR, &mut readout);
            log::info!("readout F0: {}", readout[0]);
            self.read_io_extender(REGS::OLAT, &mut readout);
            log::info!("readout 0F: {}", readout[0]);

            //DAC to 0
            let data: [u8; 2] = [((self.settings.dac_value >> 8) as u8), (self.settings.dac_value & 0xFF) as u8];
            self.write_dac(&data);



            // TODO ADC two dummy conversions
            // let mut value: [u8; 2] = [0,0];
            // for i in 1..128{
                // let adc = self.read_adc(i%4);
                // let volt: f32 = ((adc[0] as u16) << 8 | adc[1] as u16) as f32 * 5.0 / 0xFFFF as f32;
                // log::info!("readout adc: {} {}, GEN: {}", adc[0], adc[1], i);
                // log::info!("readout adc: {}V, GEN: {}", volt, i%4);

                // self.delay.delay_us(5 as u32);
                // value[0] += 1;
                // if value[0] == 1{
                    // value[1] += 1;
                // }
                // self.write_dac(&value);
            // }
            true
    }

    fn settings_update(&mut self, new_settings: Settings) -> () {
        if self.settings.dac_value != new_settings.dac_value {
            let data: [u8; 2] = [((new_settings.dac_value >> 8) as u8), (new_settings.dac_value & 0xFF) as u8];
            self.write_dac(&data);
        }
        if (self.settings.vsel != new_settings.vsel) || (self.settings.ref_sel != new_settings.ref_sel){
            self.update_io_extender(new_settings.vsel, new_settings.ref_sel);
        }
        self.settings = new_settings;
    }

    fn telemetry(&mut self) -> (Telemetry, u16) {
        // IO Extender
        let mut ios :[u8; 1] = [0];
        self.read_io_extender(REGS::GPIO, &mut ios);
        self.telemetry.v_p_limit = ios[0] & (1 << 7) != 0;
        self.telemetry.v_n_limit = ios[0] & (1 << 6) != 0;
        self.telemetry.pwr_limit = ios[0] & (1 << 5) != 0;
        self.telemetry.pwr_limit_avg = ios[0] & (1 << 4) != 0;
        self.telemetry.temp = self.read_temp();
        for i in 0..6{
            let adc = self.read_adc(i%4);
            if i >= 2 {
                self.telemetry.adc[i as usize -2] = (adc[0] as u16) << 8 | adc[1] as u16;
            }
            // if i== 2 { // Imeas
                // self.telemetry.amp = (self.telemetry.adc[0] * 2.0) - 5.0;
            // }
            // let volt: f32 = ((adc[0] as u16) << 8 | adc[1] as u16) as f32 * 5.0 / 0xFFFF as f32;
            // log::info!("readout adc: {} {}, GEN: {}", adc[0], adc[1], i);
            // log::info!("readout adc: {}V, GEN: {}", volt, i%4);
            self.delay.delay_us(5 as u32);
        }
        (self.telemetry.finalize(), self.settings.telemetry_period)
    }

    fn check_interrupt(&mut self) -> () {
        
    }

    fn poll(&mut self) -> u32 {
        0
    }
}