use embedded_hal::i2c::I2c;
use esp_idf_svc::hal::prelude::*;
use esp_idf_svc::hal::{
    i2c::{I2cConfig, I2cDriver, I2cError},
    peripheral::Peripheral,
    peripherals::Peripherals,
};

pub trait I2cGenericDriver = I2c<Error = I2cError>;

pub fn get_i2c_driver(peripherals: &mut Peripherals) -> I2cDriver<'static> {
    let sda = unsafe { peripherals.pins.gpio21.clone_unchecked() };
    let scl = unsafe { peripherals.pins.gpio22.clone_unchecked() };
    let i2c = unsafe { peripherals.i2c0.clone_unchecked() };
    let config = I2cConfig::new().baudrate(400.kHz().into());

    I2cDriver::new(i2c, sda, scl, &config).unwrap()
}
