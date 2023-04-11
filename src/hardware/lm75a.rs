use embedded_hal::blocking::i2c::{WriteRead, Read};

// 2 x LM75 on STM_SYS Board address
const I2C_ADDR: [u8; 2] = [0x48 , 0x49];

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
