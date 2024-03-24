use embedded_storage::{ReadStorage, Storage};
use esp_storage::{FlashStorage, FlashStorageError};

use crate::util::{
    error::AppError,
    math::vectors::{AccelerationVector3D, RotationVector3D},
};

const FLASH_ADDR: u32 = 0x9000;

// pub static APP_CONFIG_STORE: Lazy<Mutex<ConfigStorage>> = Lazy::new(|| Mutex::new(ConfigStorage::new()));

#[derive(Default, Debug)]
pub struct AppStoredConfig {
    pub initialized: bool,
    pub gyro_calibration: RotationVector3D,
    pub accelerometer_calibration: AccelerationVector3D,
}

pub struct ConfigStorage {
    flash: FlashStorage,
}

impl ConfigStorage {
    pub fn new() -> Self {
        let flash = FlashStorage::new();
        esp_println::println!("Flash Capacity {}", flash.capacity());
        ConfigStorage { flash }
    }
    pub fn store_to_flash(
        &mut self,
        config: AppStoredConfig,
    ) -> Result<(), AppError<Option<FlashStorageError>>> {
        let data_buffer = unsafe {
            core::slice::from_raw_parts(
                &config as *const _ as *mut u8,
                core::mem::size_of::<AppStoredConfig>(),
            )
        };
        let result = self.flash.write(FLASH_ADDR, data_buffer);
        if result.is_err() {
            Err(AppError {
                message: &"Failed to store the config",
                error: result.err(),
            })
        } else {
            Ok(())
        }
    }

    pub fn load_from_flash(
        &mut self,
    ) -> Result<AppStoredConfig, AppError<Option<FlashStorageError>>> {
        let config = AppStoredConfig::default();
        let data_buffer = unsafe {
            core::slice::from_raw_parts_mut(
                &config as *const _ as *mut u8,
                core::mem::size_of::<AppStoredConfig>(),
            )
        };
        let result = self.flash.read(FLASH_ADDR, data_buffer);
        if result.is_err() {
            Err(AppError {
                message: &"Failed to store the config",
                error: result.err(),
            })
        } else if config.initialized {
            Ok(config)
        } else {
            Ok(AppStoredConfig::default())
        }
    }
}
