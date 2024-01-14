use embedded_hal::{blocking::{delay::DelayMs, i2c::{WriteRead, Write, Read}}, digital::v2::OutputPin};
use embedded_hal::prelude::_embedded_hal_blocking_delay_DelayUs;
use stm32h7xx_hal::i2c::Error;
use stm32h7xx_hal::i2c::I2c;
use stm32h7xx_hal::device::I2C1;
use crate::hardware::panic;

use super::ServMod;

// The EEPROM is a variant without address bits, so the 3 LSB of this word are "dont-cares".
const I2C_ADDR: u8 = 0x50;

// The MAC address is stored in the last 6 bytes of the 256 byte address space.
const MAC_POINTER: u8 = 0xFA;

// Po 8 bajtow na kazdy kanal, piersze 4 oznaczaj wartosc slope, kolejne 4 offset (Potrzebna konwersja na f32)
const EEPROM_CHANNEL_1_COEFFICIENTS: u8 = 0x40; // Poczatek danych pierwszego kanalu
const EEPROM_CHANNEL_2_COEFFICIENTS: u8 = 0x48; // Poczatek danych drugiego kanalu
const EEPROM_DATA_LENGTH: u8 = 0x08; // Po osiem bajtow danych na kalibracje dla kazdego kanalu (4 bajty slope, 4 bajty offset)

pub struct SiLPADetector{
    pub channel_1_slope: f32,
    pub channel_1_intercept: f32,
    pub channel_2_slope: f32,
    pub channel_2_intercept: f32
}

impl SiLPADetector{
    pub fn new(ch1_slope : f32, ch1_intercept : f32, ch2_slope : f32, ch2_intercept : f32) -> Self{
        Self {  channel_1_slope: ch1_slope,
                channel_1_intercept: ch1_intercept,
                channel_2_slope: ch2_slope, 
                channel_2_intercept: ch2_intercept}
    }
    
    pub fn set_coefficients<T>(&mut self, slot : u8, i2c: &mut T, servmod: &mut ServMod)
    where 
        T: WriteRead,
    {
        match slot{
            1 => servmod.0.set_low().unwrap(), 
            2 => servmod.1.set_low().unwrap(),
            3 => servmod.2.set_low().unwrap(),
            4 => servmod.3.set_low().unwrap(),
            5 => servmod.4.set_low().unwrap(),
            6 => servmod.5.set_low().unwrap(),
            7 => servmod.6.set_low().unwrap(),
            8 => servmod.7.set_low().unwrap(),
            _ => log::info!("Incorrect Slot Number")
        };
        (self.channel_1_slope, self.channel_1_intercept, self.channel_2_slope, self.channel_2_intercept) = read_detector_coefficients(i2c);
        
        match slot{
            1 => servmod.0.set_high().unwrap(), 
            2 => servmod.1.set_high().unwrap(),
            3 => servmod.2.set_high().unwrap(),
            4 => servmod.3.set_high().unwrap(),
            5 => servmod.4.set_high().unwrap(),
            6 => servmod.5.set_high().unwrap(),
            7 => servmod.6.set_high().unwrap(),
            8 => servmod.7.set_high().unwrap(),
            _ => log::info!("Incorrect Slot Number")
        };
    }
}

pub fn read_eui48<T>(i2c: &mut T, delay: &mut impl DelayMs<u8>) -> [u8; 6]
where
    T: WriteRead,
{
    let mut previous_read: Option<[u8; 6]> = None;
    // On Stabilizer v1.1 and earlier hardware, there is a fault where the I2C bus is not connected
    // to the CPU until the P12V0A rail enables, which can take many seconds, or may never come up
    // at all. During these transient turn-on conditions, we may fail the I2C read operation. To
    // accomodate this, we repeat the I2C read for a set number of attempts with a fixed delay
    // between them. Then, we wait for the bus to stabilize by waiting until the MAC address
    // read-out is identical for two consecutive reads.
    for _ in 0..40 {
        let mut buffer = [0u8; 6];
        if i2c
            .write_read(I2C_ADDR, &[MAC_POINTER], &mut buffer)
            .is_ok()
        {
            if let Some(old_read) = previous_read {
                if old_read == buffer {
                    return buffer;
                }
            }

            previous_read.replace(buffer);
        } else {
            // Remove any pending previous read if we failed the last attempt.
            log::info!("EEPROM failed");
            previous_read.take();
        }

        delay.delay_ms(100);
    }

    panic!("Failed to read MAC address");
}

pub fn test_eeprom<T>(i2c: &mut T, address: u8) -> Result<(), ()>
where
    T: Write + Read + WriteRead,
{
    let mut test_data : [u8; 5] = [0; 5];
    let mut delay = asm_delay::AsmDelay::new(asm_delay::bitrate::Hertz(
        400000000,
    ))  ;
    if i2c
        .write(address, &mut [0x00, 0x01, 0x02, 0x03, 0x04, 0x05])
        .is_ok()
    {
            //i2c.write(address, &mut [0x00]);
            delay.delay_ms(100 as u32);
            match i2c
                .write_read(I2C_ADDR, &[0x00], &mut test_data)
            {
                Ok(()) => {
                    for i in 0..5 {
                        if test_data[i] != ((i as u8) + 1) {
                            log::info!("Test data: {} {} {} {} {}", test_data[0], test_data[1], test_data[2], test_data[3], test_data[4]);
                            panic!("Failed during EEPROM test data {} : {}", i, test_data[i]);
                            return Err(())
                        }
                    }
                },
                Err(e) => {
                    panic!("I2C Read Error")
                }
            }
    } else {
        panic!("Failed to write test data to EEPROM")
    }
    log::info!("Test data: {} {} {} {} {}", test_data[0], test_data[1], test_data[2], test_data[3], test_data[4]);
    Ok(())
}

pub fn read_detector_coefficients<T>(i2c: &mut T) -> (f32, f32, f32, f32)
where
    T: WriteRead,
{
    let channel_1_slope: f32;
    let channel_1_intercept: f32;
    let channel_2_slope: f32;
    let channel_2_intercept: f32;

    let mut received_coefficients : [u8; 16] = [0; 16];  

    match i2c.write_read(I2C_ADDR, &[EEPROM_CHANNEL_1_COEFFICIENTS], &mut received_coefficients){
        Ok(()) => {
            channel_1_slope = ( 
                                ((received_coefficients[0] as u32) << 24) |
                                ((received_coefficients[1] as u32) << 16) |
                                ((received_coefficients[2] as u32) << 8) |
                                ((received_coefficients[3] as u32) << 0)
                            ) as f32;
            channel_1_intercept = ( 
                                ((received_coefficients[4] as u32) << 24) |
                                ((received_coefficients[5] as u32) << 16) |
                                ((received_coefficients[6] as u32) << 8) |
                                ((received_coefficients[7] as u32) << 0)
                            ) as f32;    

            channel_2_slope = ( 
                                ((received_coefficients[8] as u32) << 24) |
                                ((received_coefficients[9] as u32) << 16) |
                                ((received_coefficients[10] as u32) << 8) |
                                ((received_coefficients[11] as u32) << 0)
                            ) as f32;
            channel_2_intercept = ( 
                                ((received_coefficients[12] as u32) << 24) |
                                ((received_coefficients[13] as u32) << 16) |
                                ((received_coefficients[14] as u32) << 8) |
                                ((received_coefficients[15] as u32) << 0)
                            ) as f32;     
        }
        Err(e) => {
            panic!("I2C Error receiving coefficients")
        }
    }


    (channel_1_slope, channel_1_intercept, channel_2_slope, channel_2_intercept)
}

pub fn test_example_coefficients<T>(i2c: &mut T, address: u8, data : &mut [f32]) -> ()
where 
    T: WriteRead + Write,
{
    let mut array_1st_page : [u8; 9] = [0; 9];
    let mut array_2nd_page : [u8; 9] = [0; 9];

    let ch1_slope_u32 : u32 = data[0].to_bits();
    let ch2_slope_u32 : u32 = data[2].to_bits();
    let ch1_intercept_u32 : u32 = data[1].to_bits();
    let ch2_intercept_u32 : u32 = data[3].to_bits();
    log::info!("Dane po konwersji na u32: {:#034b} {:#034b} {:#034b} {:#034b}", ch1_slope_u32, ch2_slope_u32, ch1_intercept_u32, ch2_intercept_u32);

    let mut delay = asm_delay::AsmDelay::new(asm_delay::bitrate::Hertz(
        400000000,
    ));

    for i in 0..7{
        if i < 4 {
            array_1st_page[i + 1] = (ch1_slope_u32 >>  24 - 8*i | 0x00) as u8;
            array_2nd_page[i + 1] = (ch2_slope_u32 >>  24 - 8*i | 0x00) as u8;
        } else {
            array_1st_page[i + 1] = (ch1_intercept_u32 >> 24 - 8*(i - 4) | 0x00) as u8;
            array_2nd_page[i + 1] = (ch2_intercept_u32 >> 24 - 8*(i - 4) | 0x00) as u8;
        }
    }

    array_1st_page[0] = EEPROM_CHANNEL_1_COEFFICIENTS;
    array_2nd_page[0] = EEPROM_CHANNEL_2_COEFFICIENTS;

    // Wpisanie do pamieci testowych współczynników
    let _ = i2c.write(address, &array_1st_page);
    delay.delay_ms(100 as u32);
    let _ = i2c.write(address, &array_2nd_page);
    delay.delay_ms(100 as u32);
    // Odczytanie przykładowych współczynników
    let mut detector_coefficients : [f32; 4] = [0.0; 4];
    (detector_coefficients[0], detector_coefficients[1], detector_coefficients[2], detector_coefficients[3]) = read_detector_coefficients(i2c);

    log::info!("Przykładowe wartości parametrów do testów: {} {} {} {}",    data[0].to_bits(), data[1].to_bits(), data[2].to_bits(), data[3].to_bits());
    log::info!("Odczytane wartości z eepromu:              {} {} {} {}",    detector_coefficients[0].to_bits(), detector_coefficients[1].to_bits(), 
                                                                            detector_coefficients[2].to_bits(), detector_coefficients[3].to_bits());
        
    

}