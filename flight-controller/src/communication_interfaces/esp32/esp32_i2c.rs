use esp_idf_svc::hal::prelude::*;
use esp_idf_svc::hal::{
    i2c::{I2cConfig, I2cDriver},
    peripherals::Peripherals,
};

pub struct Esp32I2CAdapter;

impl Esp32I2CAdapter {
    pub fn get_i2c_driver() -> I2cDriver<'static> {
        let peripherals = Peripherals::take().unwrap();

        let sda = peripherals.pins.gpio21;
        let scl = peripherals.pins.gpio22;
        let i2c = peripherals.i2c0;
        let config = I2cConfig::new().baudrate(400.kHz().into());

        I2cDriver::new(i2c, sda, scl, &config).unwrap()
    }
}
