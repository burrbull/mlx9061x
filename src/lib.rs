//#![deny(missing_docs)]
#![deny(warnings)]
#![no_std]

/// Default MLX9061x address
pub const ADDRESS   : u8  = 0x5A;
pub const FREQUENCY : u32 = 100000;


#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Copy, Clone)]
pub enum Register {
    // RAM Access (read only)
    RAW_IR_1 = 0x04,
    RAW_IR_2 = 0x05,
    T_A      = 0x06,
    T_OBJ_1  = 0x07,
    T_OBJ_2  = 0x08,
    
    // EEPROM Access
    T_O_MAX   = 0x20,
    T_O_MIN   = 0x21,
    PWMCTRL   = 0x22,
    T_A_RANGE = 0x23,
    ECC       = 0x24,
    CR1       = 0x25,
    ADDRESS_LSBYTE = 0x2E,
    
    // Read Flags
    FLAGS     = 0xF0,
    
    // Enter SLEEP mode
    SLEEP     = 0xFF,
}

impl Register {
    pub fn addr(&self) -> u8 {
        *self as u8
    }
}

extern crate cast;
extern crate embedded_hal as hal;

use core::mem;

use self::hal::blocking::i2c::{Write, WriteRead};

/// MLX9061x struct
pub struct Mlx9061x<I2C> {
    i2c: I2C,
    address : u8,
}


    
/// Adds one byte to CRC-8 (x8+x2+x1+1)
fn crc8_byte(data : u8, crc : u8) -> u8 {
	let mut crc = crc ^ data;
	for _ in 0..8 { 
		crc = if crc & 0x80 != 0 { crc << 1 ^ 0x07} else { crc << 1 };
	}
	crc
}

impl<I2C, E> Mlx9061x<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
{
    /// Creates a new driver from a I2C peripheral
    pub fn new(i2c: I2C, address : u8) -> Self {
        Mlx9061x { i2c, address }
    }

    /// Inner temperature measurement
    pub fn inner_temp(&mut self) -> Result<f32, E> {
        let temp16 = self.inner_temp_raw()?;
        let celsious = ((temp16 - 13658) as f32) / 50.;       // 273.16 * 50 = 13658
        Ok( celsious )
    }

    /// Temperature sensor 1 measurement
    pub fn object_temp_1(&mut self) -> Result<f32, E> {
        let temp16 = self.object_temp_1_raw()?;
        let celsious = ((temp16 - 13658) as f32) / 50.;
        Ok( celsious )
    }

    /// Temperature sensor 2 measurement
    pub fn object_temp_2(&mut self) -> Result<f32, E> {
        let temp16 = self.object_temp_2_raw()?;
        let celsious = ((temp16 - 13658) as f32) / 50.;
        Ok( celsious )
    }

    /// Raw inner temperature measurement
    #[inline]
    pub fn inner_temp_raw(&mut self) -> Result<u16, E> {
        let temp = self.read_temp(Register::T_A)?;
        Ok( temp )
    }

    /// Raw temperature sensor 1 measurement
    #[inline]
    pub fn object_temp_1_raw(&mut self) -> Result<u16, E> {
        let temp = self.read_temp(Register::T_OBJ_1)?;
        Ok( temp )
    }

    /// Raw temperature sensor 2 measurement
    #[inline]
    pub fn object_temp_2_raw(&mut self) -> Result<u16, E> {
        let temp = self.read_temp(Register::T_OBJ_2)?;
        Ok( temp )
    }

    fn read_temp(&mut self, reg: Register) -> Result<u16, E> {
        let mut crc = 0u8;
        /*crc = crc8_byte(self.address, crc);
        crc = crc8_byte(reg.addr(), crc);
        crc = crc8_byte(self.address | 1, crc);*/
        crc = crc8_byte(self.address << 1, crc);
        crc = crc8_byte(reg.addr(), crc);
        crc = crc8_byte((self.address << 1) | 1, crc);
        
   //     let buff = self.read_registers::<[u8; 3]>(reg)?;
        let mut buff: [u8; 3] = unsafe { mem::uninitialized() };
        self.i2c.write_read(self.address, &[reg.addr()], &mut buff)?;
    
        // Read CRC add compare with PEC
        for b in buff.iter() {
            crc = crc8_byte(*b, crc);
        }
        /*
        if crc != 0 {
            return Err(nb::Error::Other(Error::BadChksum));
        }*/
      
        let temp16 = (buff[0] as u16) | (((buff[1]) as u16) << 8);
        /*
        if (temp16 & 0x8000) != 0 {
            return Err(nb::Error::Other(Error::MeasErr));
        }*/
        
        Ok(temp16)
    }
/*    
    #[allow(unused)]
    fn read_registers<B>(&mut self, reg: Register) -> Result<B, E>
    where
        B: Unsize<[u8]>,
    {
        let mut buffer: B = unsafe { mem::uninitialized() };

        {
            let buffer: &mut [u8] = &mut buffer;
            self.i2c
                .write_read(self.address, &[reg.addr()], buffer)?;
        }
        Ok(buffer)
    }

    #[allow(unused)]
    fn read_register(&mut self, reg: Register) -> Result<u8, E> {
        self.read_registers::<[u8; 1]>(reg).map(|b| b[0])
    }

    #[allow(unused)]
    fn write_register(&mut self, reg: Register, byte: u8) -> Result<(), E> {
        self.i2c.write(self.address, &[reg.addr(), byte])
    }*/
}
