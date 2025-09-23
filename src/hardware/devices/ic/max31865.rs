use crate::hardware::ecp5::ECP5;
use miniconf::Miniconf;
use serde::Serialize;
use micromath::F32Ext;

pub mod REGS {
    pub const CONFIGURATION: u8 = 0x00;
    pub const RTDMSB  : u8 = 0x01;
    pub const RTDLSB  : u8 = 0x02;
    pub const HTRESHOLDMSB: u8 = 0x03;
    pub const HTRESHOLDLSB: u8 = 0x04;
    pub const LTHRESHOLDMSB: u8 = 0x05;
    pub const LTHRESHOLDLSB: u8 = 0x06;
    pub const FALUTSTATUS: u8 = 0x07;
}

pub mod STATE {
    pub const IDLE: u8 = 0;
    pub const MEASURE: u8 = 1;
    pub const FAULT_AUTO_CHECK: u8 = 2;
    pub const FAULT_MANUAL_CHECK: u8 = 3;
}

pub mod FILTER {
    pub const Hz60: u8 = 0;
    pub const Hz50: u8 = 1;
}

#[derive(Copy, Clone)]
pub struct TempTelemetryBuffer {
    pub temp: u16,
    pub status: u8
}

impl Default for TempTelemetryBuffer {
    fn default() -> Self {
        Self {
            temp: 0,
            status: 0
        }
    }
}

#[derive(Copy, Clone, Serialize)]
pub struct Telemetry {
    pub temp: f32,
    pub status: u8,
}

impl Default for Telemetry {
    fn default() -> Self {
        Self {
            temp: 0.0,
            status: 0
        }
    }
}

#[derive(Clone, Copy, Debug, Miniconf, PartialEq)]
pub struct Settings {
    pub a   : f32,      //
    pub b       : f32, //
    pub r_nom   : f32, //
    pub r_ref   : f32,  // 3920.0 Ohm or 390.17 Ohm +
    fault_auto : bool,
    pub filter : u8, // 0 - 60Hz, 1 - 50Hz
    pub high_threshold : u16, // High threshold value
    pub low_threshold : u16, // Low threshold value
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            a: 3.9083e-3,
            b: -5.775e-7,
            r_nom: 1000.0,
            r_ref: 3920.0, // 3920.0 Ohm or 390.17 Ohm
            fault_auto: true,
            filter: FILTER::Hz50,
            high_threshold: 0xFFFF, // Default to maximum value
            low_threshold: 0x0000, // Default to minimum value
        }
    }
}

#[derive(Clone, Copy)]
pub struct Max31865 {
    slot: u16,
    z1: f32,
    z2: f32,
    z3: f32,
    z4: f32,
    rref: f32,
    r_nom: f32,
    pub active: bool
}

impl Max31865 {
    pub fn new(
        slot: u16,
    ) -> Self {
        Self {
            slot: slot,
            z1: 0.0,
            z2: 0.0,
            z3: 0.0,
            z4: 0.0,
            rref: 0.0,
            r_nom: 0.0,
            active: true
        }
    }

    pub fn set_register(&mut self, ecp5: &mut ECP5, reg_adr: u8, value: u8) {
        let data : [u8; 2] = [reg_adr | (1 << 7), value];
        ecp5.write_spi(self.slot, &data);
    }

    pub fn read_register(&mut self, ecp5: &mut ECP5, reg_adr: u8, value: &mut [u8]){
        let address = [reg_adr];
        ecp5.read_spi(self.slot, &address, value);
    }

    pub fn update_configuration(&mut self, ecp5: &mut ECP5, settings: Settings, init_fault: bool, reset_fault: bool) {
        let mut config = 0b0000_0000; // Default configuration
        if self.active {
            config |= 0b1000_0000; // Enable Vbias
        }
        if settings.fault_auto {
            config |= 0b0100_0000; // Enable auto conversion mode
        }
        if settings.filter == FILTER::Hz50 {
            config |= 0b0000_0001; // Set filter to 50Hz
        }
        if init_fault {
            config |= 0b0000_0100; // Enable fault detection
        }
        if reset_fault {
            config |= 0b000_0010; // Reset fault status
        }
        self.set_register(ecp5, REGS::CONFIGURATION, config);
    }

    pub fn settigns_update(&mut self, a : f32, b: f32, r_nom: f32, rref: f32) {
        self.z1 = -a;
        self.z2 = a * a - (4.0 * b);
        self.z3 = (4.0 * b) / r_nom as f32;
        self.z4 = 2.0 * b;
        self.rref = rref;
        self.r_nom = r_nom;
    }
    
    pub fn calculate_pt100(&mut self, value: u16) -> f32 {
        let mut temp : f32;
        let mut Rt : f32;

        Rt = self.rref;
        Rt *= value as f32;
        Rt /= 32768.0;
      
        temp = self.z2 + (self.z3 * Rt);
        temp = ((temp.sqrt()) + self.z1) / self.z4;
      
        if (temp >= 0.0) {
            return temp;
        }
      
        // ugh.
        Rt /= self.r_nom;
        Rt *= 100.0; // normalize to 100 ohm
      
        let mut rpoly : f32 = Rt;
      
        temp = -242.02;
        temp += 2.2228 * rpoly;
        rpoly *= Rt; // square
        temp += 2.5859e-3 * rpoly;
        rpoly *= Rt; // ^3
        temp -= 4.8260e-6 * rpoly;
        rpoly *= Rt; // ^4
        temp -= 2.8183e-8 * rpoly;
        rpoly *= Rt; // ^5
        temp += 1.5243e-10 * rpoly;

        return temp;
    }
}