use esp32_hal::{
    clock::Clocks,
    i2c::I2C,
    peripheral::Peripheral,
    peripherals::{Peripherals, I2C0},
    prelude::*,
    IO,
};

pub fn get_i2c_driver<'a, T>(peripherals: &mut Peripherals, clocks: &Clocks) -> I2C<'a, I2C0> {
    let gpio = unsafe { peripherals.GPIO.clone_unchecked() };
    let io_mux = unsafe { peripherals.IO_MUX.clone_unchecked() };

    let io = IO::new(gpio, io_mux);
    let sda = io.pins.gpio21;
    let scl = io.pins.gpio22;
    let i2c = unsafe { peripherals.I2C0.clone_unchecked() };
    // let config = I2cConfig::new().baudrate(400.kHz().into());

    I2C::new(i2c, sda, scl, 400_u32.kHz(), clocks)
}
