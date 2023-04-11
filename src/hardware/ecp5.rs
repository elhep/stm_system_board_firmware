use embedded_hal::digital::v2::InputPin;
pub use stm32h7xx_hal as hal;
use crate::hardware::ECP5InterfaceReady;
use heapless::String;
use embedded_hal::blocking::delay::DelayMs;

//number of slots in the design
pub const SLOTS_NUM: u8 = 2;
pub const OFFSET_TO_SPI: u8 = 6;
pub const OFFSET_TO_SLOT: u8 = 20;
//
// enum Slot{
//     Output=0,
//     Input=1,
//     Oe=2,
//     Int=3,
//     IntMsk=4,
//     IntCl=5,
//     _end=6
// }

pub mod SLOT{     // SLOT registers map (OFFSET_TO_SLOT * slot_nb + slot_reg)
    pub const OUTPUT:  u8 = 0;
    pub const INPUT:   u8 = 1;
    pub const OE:      u8 = 2;
    pub const INT:     u8 = 3;
    pub const INT_MSK: u8 = 4;
    pub const INT_CL:  u8 = 5;
    pub const _end  :  u8 = 6;
}

pub mod SPI{        // SPI Register map (OFFSET_TO_SLOT * slot_nb + OFFSET_TO_SPI + spi_reg)
    pub const DATA:    u8 = 0;
    pub const LENGTH:  u8 = 1;
    pub const CS:      u8 = 2;
    pub const CS_POL:  u8 = 3;
    pub const DIV:     u8 = 4;
    pub const OFFLINE: u8 = 5;
    pub const CLK_POL: u8 = 6;
    pub const CLK_PHA: u8 = 7;
    pub const LSB_FST: u8 = 8;
    pub const HALF_DUP: u8 = 9;
    pub const END:      u8 = 10;
    pub const READABLE: u8 = 11;
    pub const WRITABLE: u8 = 12;
    pub const IDLE:     u8 = 13;
    pub const _end  :  u8 = 14;
}

pub struct ECP5{
    pub qspi: hal::xspi::Qspi<hal::device::QUADSPI>,
    interface_ready: ECP5InterfaceReady,
}



impl ECP5 {
    pub fn new(qspi : hal::xspi::Qspi<hal::device::QUADSPI>,
               interface_ready: ECP5InterfaceReady
    ) -> Self {
        Self {
            qspi,
            interface_ready
        }
    }

    pub fn init(&self) {
        // TODO Init QSPI: test version: only instruction and data phase with dynamic data length
    }
    fn get_device_reg_addr(&self, slot_number: u8) -> u8 {
        slot_number * 1  //TODO check value with final gateware version
    }

    pub fn write_to_ecp5(&mut self,
                         reg_addr : u8,
                         data: &[u8]
    ) -> Result<(), hal::xspi::QspiError> {
        self.qspi.write(reg_addr, data)
    }

    pub fn read_from_ecp5(&mut self,
                         reg_addr : u8,
                         data: &mut [u8]
    ) -> Result<(), hal::xspi::QspiError> {
        self.qspi.read((1 << 7) | reg_addr, data)
    }


    // // data = [device_reg_address, data0, data1 ... dataX]
    // pub fn write_to_device(&mut self,
    //                        slot_number: u8,
    //                        data: &[u8]
    // ) -> Result<(), hal::xspi::QspiError> {
    //     self.write_to_ecp5(address_ecp5, data)
    // }

    // TODO Check ECP5 active polarization
    fn interface_is_ready(&mut self, slot_number: u8) -> bool {
        match slot_number {
            0 => self.interface_ready.0.is_high().unwrap(),
            1 => self.interface_ready.1.is_high().unwrap(),
            2 => self.interface_ready.2.is_high().unwrap(),
            3 => self.interface_ready.3.is_high().unwrap(),
            4 => self.interface_ready.4.is_high().unwrap(),
            5 => self.interface_ready.5.is_high().unwrap(),
            6 => self.interface_ready.6.is_high().unwrap(),
            7 => self.interface_ready.7.is_high().unwrap(),
            _ => panic!("Slot number >= 8")
        }
    }

    // // dest = [device_reg_address, data0, data1 ... dataX]
    // pub fn read_from_device(&mut self,
    //             slot_number : u8,
    //             dest: &mut [u8],
    // ) -> Result<(), hal::xspi::QspiError> {
    //     self.write_to_device(slot_number, dest).unwrap();
    //
    //     while self.interface_is_ready(slot_number) {}
    //
    //     self.read_from_ecp5(self.get_device_reg_addr(slot_number), dest)
    // }

    pub fn set_spi_cs_pol(&mut self,
                          _slot_number : u8,
                          _pol : u8){
        //TODO wrtie proper write_to_ecp5
    }

    fn spi_machine_write(&mut self, slot_number : u8, data: &[u8; 2]){
        let mut array : [u8; 2] = [0x00, 0x00];

        //wait for idle
        while array[1] != 1 {
            self.read_from_ecp5(OFFSET_TO_SLOT * slot_number + SPI::IDLE, &mut array).unwrap();
        }
        self.write_to_ecp5(OFFSET_TO_SLOT * slot_number + SPI::DATA, data).unwrap();
    }

    fn spi_machine_read(&mut self, slot_number : u8, data: &mut[u8; 2]){
        let mut array : [u8; 2] = [0x00, 0x00];

        while array[1] != 1 {
            self.read_from_ecp5(OFFSET_TO_SLOT * slot_number + SPI::IDLE, &mut array).unwrap();
        }
        self.write_to_ecp5(OFFSET_TO_SLOT * slot_number + SPI::DATA, data).unwrap();

        while array[1] != 1 {
            self.read_from_ecp5(OFFSET_TO_SLOT * slot_number + SPI::IDLE, &mut array).unwrap();
        }
        self.read_from_ecp5(OFFSET_TO_SLOT * slot_number + SPI::DATA, data).unwrap();
    }

    pub fn write_spi(&mut self,
                     slot_number: u8,
                     data: &[u8]){
        let mut pointer = 0;
        let mut idle = [0x00, 0x00];
        while (data.len() - pointer) != 0{
            while idle[1] != 1 {
                self.read_from_ecp5(OFFSET_TO_SLOT * slot_number + SPI::IDLE, &mut idle).unwrap();
            }
            if (data.len() - pointer) == 1{     // default single data transfer is 16 bits, end transfer
                self.write_to_ecp5(OFFSET_TO_SLOT * slot_number + SPI::LENGTH, &[0x00, 0x07]).unwrap();
                self.write_to_ecp5(OFFSET_TO_SLOT * slot_number + SPI::END, &[0x00, 0x01]).unwrap();
                self.spi_machine_write(slot_number, &[0x00, data[pointer]]);
                pointer += 1;
            } else if (data.len() - pointer) == 2 {
                self.write_to_ecp5(OFFSET_TO_SLOT * slot_number + SPI::LENGTH, &[0x00, 0x0F]).unwrap();
                self.write_to_ecp5(OFFSET_TO_SLOT * slot_number + SPI::END, &[0x00, 0x01]).unwrap();
                self.spi_machine_write(slot_number,
                                       &[data[pointer], data[pointer+1]]);
                pointer += 2;
            } else {
                self.write_to_ecp5(OFFSET_TO_SLOT * slot_number + SPI::LENGTH, &[0x00, 0x0F]).unwrap();
                self.write_to_ecp5(OFFSET_TO_SLOT * slot_number + SPI::END, &[0x00, 0x00]).unwrap();
                while (data.len() - pointer) > 2 {
                    self.spi_machine_write(slot_number,
                                           &[data[pointer], data[pointer]]);
                    pointer += 2;
                }
            }
        }
    }

    pub fn read_inputs(&mut self,
                        slot_number: u8,
                        data: &mut [u8]){
        self.read_from_ecp5(OFFSET_TO_SLOT * slot_number + SLOT::INPUT, data);
    }

   pub fn read_spi(&mut self,
                   slot_number: u8,
                   write: &[u8],
                   read: &mut [u8]) {
       let mut array: [u8; 2] = [0x00, 0x00];
       let mut delay = asm_delay::AsmDelay::new(asm_delay::bitrate::Hertz(
            400000000,
        ))  ;
       //16 bits and end (8 address 8 data end transmission
       self.write_to_ecp5(OFFSET_TO_SLOT * slot_number + OFFSET_TO_SPI + SPI::LENGTH, &[0x00, 0x0F]).unwrap();
       self.write_to_ecp5(OFFSET_TO_SLOT * slot_number + OFFSET_TO_SPI + SPI::END, &[0x00, 0x01]).unwrap();

       //wait for idle
       log::info!("READ SPI: Wait for writeable");
       while array[1] != 1 {
           self.read_from_ecp5(OFFSET_TO_SLOT * slot_number + OFFSET_TO_SPI + SPI::WRITABLE, &mut array).unwrap();
       }
       log::info!("READ SPI: Idle detected");

       log::info!("READ SPI: Tranfer 16 bit - 8 real write, 8 read");
       //write
       self.write_to_ecp5(OFFSET_TO_SLOT * slot_number + OFFSET_TO_SPI + SPI::DATA, &[write[0], 0]).unwrap();

       //self.check_registers();

       //wait for IDLE!!! (end of tranfer!!! should wait for readable only without END flag)
       log::info!("READ SPI: Wair for Readable");
       self.read_from_ecp5(OFFSET_TO_SLOT * slot_number + OFFSET_TO_SPI + SPI::IDLE, &mut array).unwrap();
       while array[1] != 1 {
           self.read_from_ecp5(OFFSET_TO_SLOT * slot_number + OFFSET_TO_SPI + SPI::IDLE, &mut array).unwrap();
       }
       log::info!("READ SPI: Readable detected!");
       self.read_from_ecp5(OFFSET_TO_SLOT * slot_number + OFFSET_TO_SPI + SPI::DATA, read).unwrap();
   }

    pub fn check_registers(&mut self){
        let mut array : [u8; 2] = [0x00, 0x00];
        let mut delay = asm_delay::AsmDelay::new(asm_delay::bitrate::Hertz(
            400000000,
        ))  ;
        for slot in 0..3 {
            delay.delay_ms(1000 as u32);
            log::info!("Slot {} :", slot);
            for reg in 0..SLOT::_end{
                self.read_from_ecp5(OFFSET_TO_SLOT * slot + reg, &mut array).unwrap();
                log::info!("   {} Rejestr {} {} {}", OFFSET_TO_SLOT * slot + reg, ECP5::print_reg(reg), array[0], array[1]);
            }
            delay.delay_ms(1000 as u32);
            for reg in 0..SPI::_end{
                // if OFFSET_TO_SLOT * slot + OFFSET_TO_SPI + reg == 26{
                //     continue
                // }
                self.read_from_ecp5(OFFSET_TO_SLOT * slot + OFFSET_TO_SPI + reg, &mut array).unwrap();
                log::info!("   {} Rejestr SPI {} {} {}", OFFSET_TO_SLOT * slot + OFFSET_TO_SPI + reg, ECP5::print_reg_spi(reg), array[0], array[1]);
            }
        }
    }

    fn print_reg(reg: u8) -> String<10>{
        match reg {
            0 => String::from("OUTPUT    "),
            1 => String::from("INPUT     "),
            2 => String::from("OE        "),
            3 => String::from("INT       "),
            4 => String::from("INT_MSK   "),
            5 => String::from("INT_CL    "),
            _ => String::from("ERROR     ")
        }
    }

    fn print_reg_spi(reg: u8) -> String<10>{
        match reg {
            0  => String::from("DATA      "),
            1  => String::from("LENGTH    "),
            2  => String::from("CS        "),
            3  => String::from("CS_POL    "),
            4  => String::from("DIV       "),
            5  => String::from("OFFLINE   "),
            6  => String::from("CLK_POL   "),
            7  => String::from("CLK_PHA   "),
            8  => String::from("LSB_FST   "),
            9  => String::from("HALF_DUP  "),
            10 => String::from("END       "),
            11 => String::from("READABLE  "),
            12 => String::from("WRITABLE  "),
            13 => String::from("IDLE      "),
            _ => String::from("ERROR     ")
        }
    }
}