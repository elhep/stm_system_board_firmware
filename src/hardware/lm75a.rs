use embedded_hal::blocking::i2c::{WriteRead, Write, Read};

// 2 x LM75 on STM_SYS Board address
pub const I2C_ADDR: [u8; 2] = [0x48 , 0x49];

const THYST_REG: u8 = 0x02;
const TOS_REG: u8 = 0x03;

pub mod LM75_TEMPERATURE{
    pub const MIN_TEMPERATURE : f32 = -128.0;
    pub const MAX_TEMPERATURE : f32 = 128.0; //Max temperature plus accuracy
    pub const TRESHOLD_CH1 : f32 = 80.0;
    pub const TRESHOLD_CH2 : f32 = 80.0;
}

pub fn read_temp<T>(i2c: &mut T, dev_addr: u8) -> Result<f32, T::Error>
where
    T: Read,
{
    let mut buffer : [u8; 2] = [0; 2];
    match i2c.read(dev_addr, &mut buffer){
        Ok(()) => {
            let mut x : f32 = 0.0;
            if (buffer[0] & 0b1000_0000) != 0 {
                x = -128.0;
            }
            Ok(x + (buffer[0] & 0b0111_1111) as f32 + 0.5 * (buffer[0] & 0b1000_0000) as f32)
        },
        Err(e) => Err(e),
    }
}

pub fn set_tos<T>(i2c: &mut T, dev_addr: u8, tos: f32) -> ()
where 
    T: Write,
{
    let temp = (tos * 2.0) as u8;

    let mut buffer : [u8; 3] = [0; 3];
    buffer[0] = TOS_REG;
    if (tos < 0.0){
        buffer[1] = (tos as u8);
        buffer[2] = (((128.0 + tos) * 2.0) as u8) << 7;
    } else {
        buffer[1] = 0x00;
        buffer[2] = ((tos * 2.0) as u8) << 7;
    }
    // buffer[1] = temp >> 1;
    // buffer[2] = temp << 7;
    let _ = i2c.write(dev_addr, &buffer);
}

pub fn read_tos<T>(i2c: &mut T, dev_addr : u8) -> f32
where 
    T : WriteRead + Write + Read,
{
    let mut buffer : [u8; 2] = [0; 2];
    let _ = i2c.write_read(dev_addr, &[TOS_REG], &mut buffer);
    // let _ = i2c.read(dev_addr, &mut buffer);
    log::info!("{}, {}", buffer[0], buffer[1]);
    return (buffer[1] * 2) as f32;
}

pub fn set_thyst<T>(i2c: &mut T, dev_addr: u8, thyst: f32) -> ()
where 
    T: WriteRead + Write,
{
    let temp = (thyst * 2.0) as u8;

    let mut buffer : [u8; 3] = [0; 3]; 
    buffer[0] = THYST_REG;
    buffer[1] = temp >> 1;
    buffer[2] = (temp << 7) & 0x80;
    let _ = i2c.write(dev_addr, &buffer);
}

pub fn read_thyst<T>(i2c: &mut T, dev_addr : u8) -> f32
where 
    T : WriteRead + Write + Read,
{
    let mut buffer : [u8; 2] = [0; 2];
    let _ = i2c.write_read(dev_addr, &[THYST_REG], &mut buffer);
    return (buffer[1] * 2) as f32;
}
