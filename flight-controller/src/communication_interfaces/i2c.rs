use super::esp32::esp32_i2c::Esp32I2CAdapter;
use embedded_hal::i2c::I2c;
use embedded_hal_0_2::blocking::i2c::{Write, WriteRead};
use esp_idf_svc::hal::{i2c::I2cError, peripherals::Peripherals};

pub trait I2cGenericDriver =
    I2c<Error = I2cError> + Write<Error = I2cError> + WriteRead<Error = I2cError>;

pub struct I2CAdapterFactory;
impl I2CAdapterFactory {
    // #[cfg(board = "esp32")]
    pub fn get_i2c_adapter(peripherals: Peripherals) -> impl I2cGenericDriver {
        Esp32I2CAdapter::get_i2c_driver(peripherals)
    }
}
