use std::sync::Mutex;

use embedded_storage::{ReadStorage, Storage};
use esp_storage::FlashStorage;
use once_cell::sync::Lazy;

use crate::util::math::vectors::{AccelerationVector3D, RotationVector3D};

const FLASH_ADDR: u32 = 0x9000;

pub static APP_CONFIG_STORE: Lazy<Mutex<ConfigStorage>> = Lazy::new(|| Mutex::new(ConfigStorage::new()));

#[derive(Default)]
pub struct AppStoredConfig {
    pub gyro_calibration: RotationVector3D,
    pub accelerometer_calibration: AccelerationVector3D,
}

pub struct ConfigStorage {
    flash: FlashStorage,
}

impl ConfigStorage {
    pub fn new() -> Self {
        ConfigStorage {
            flash: FlashStorage::new(),
        }
    }
    pub fn store_to_flash(&mut self, config: AppStoredConfig) {
        let data_buffer = unsafe {
            core::slice::from_raw_parts(
                &config as *const _ as *mut u8,
                core::mem::size_of::<AppStoredConfig>(),
            )
        };
        self.flash.write(FLASH_ADDR, data_buffer).unwrap();
    }

    pub fn load_from_flash(&mut self) -> AppStoredConfig {
        let config = AppStoredConfig::default();
        let data_buffer = unsafe {
            core::slice::from_raw_parts_mut(
                &config as *const _ as *mut u8,
                core::mem::size_of::<AppStoredConfig>(),
            )
        };
        self.flash.read(FLASH_ADDR, data_buffer).unwrap();
        config
    }
}
