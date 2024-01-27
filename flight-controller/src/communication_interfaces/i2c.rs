use super::{esp32::esp32_i2c::Esp32I2CAdapter, i2c_adapter::I2CAdapter};

pub struct I2CAdapterFactory;
impl I2CAdapterFactory {
    // #[cfg(board = "esp32")]
    pub fn get_i2c_adapter() -> impl I2CAdapter {
        Esp32I2CAdapter::new()
    }
}
