use crate::communication_interfaces::i2c_adapter::I2CAdapter;
use crate::util::error::AppError;
use esp_idf_svc::hal::delay::BLOCK;
use esp_idf_svc::hal::prelude::*;
use esp_idf_svc::hal::{
    i2c::{I2cConfig, I2cDriver},
    peripherals::Peripherals,
};

pub struct Esp32I2CAdapter<'a> {
    driver: I2cDriver<'a>,
}

impl<'a> Esp32I2CAdapter<'a> {
    pub fn new() -> Self {
        let peripherals = Peripherals::take().unwrap();

        let sda = peripherals.pins.gpio21;
        let scl = peripherals.pins.gpio22;
        let i2c = peripherals.i2c0;
        let config = I2cConfig::new().baudrate(400.kHz().into());
        Esp32I2CAdapter {
            driver: I2cDriver::new(i2c, sda, scl, &config).unwrap(),
        }
    }
}

impl<'a> I2CAdapter for Esp32I2CAdapter<'a> {
    fn write_to_device(&mut self, address: u8, bytes: &[u8]) -> Result<(), AppError> {
        let result = self.driver.write(address, &bytes, BLOCK);
        if result.is_err() {
            return Err(AppError {
                message: format!("Could not write to {}", address),
            });
        }
        Ok(result.unwrap())
    }

    fn read_from_device<const BUFFER_SIZE: usize>(
        &mut self,
        address: u8,
    ) -> Result<[u8; BUFFER_SIZE], AppError> {
        let mut buffer: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];
        let result = self.driver.read(address, &mut buffer, BLOCK);
        if result.is_err() {
            return Err(AppError {
                message: format!("Could not read from {}", address),
            });
        }
        result.unwrap();
        Ok(buffer)
        // self.driver.write(address, bytes, BLOCK)
    }
}
