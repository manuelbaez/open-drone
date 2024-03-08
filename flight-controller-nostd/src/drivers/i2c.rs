use embedded_hal::i2c::I2c;
use esp32_hal::{
    clock::Clocks,
    i2c::I2C,
    peripheral::Peripheral,
    peripherals::{Peripherals, I2C0},
    prelude::*,
    IO,
};

pub fn get_i2c_driver<'a, T>(peripherals: &mut Peripherals, clocks: &Clocks) -> I2C<'a, I2C0> {
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let sda = unsafe { io.pins.gpio21.clone_unchecked() };
    let scl = unsafe { io.pins.gpio22.clone_unchecked() };
    let i2c = unsafe { peripherals.I2C0.clone_unchecked() };
    // let config = I2cConfig::new().baudrate(400.kHz().into());

    I2C::new(i2c, sda, scl, 400_u32.kHz(), clocks)
}
