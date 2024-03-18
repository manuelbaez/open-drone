use std::sync::Mutex;

use crate::util::{
    error::AppError,
    math::vectors::{AccelerationVector3D, RotationVector3D},
};
use esp_idf_svc::{
    nvs::{EspDefaultNvsPartition, EspNvs, EspNvsPartition, NvsDefault},
    sys::EspError,
};
use once_cell::sync::Lazy;

const BLOB_NAME: &str = "config";

pub static APP_CONFIG_STORE: Lazy<Mutex<ConfigStorage>> = Lazy::new(|| Mutex::new(ConfigStorage::new()));

#[derive(Default)]
pub struct AppStoredConfig {
    pub gyro_calibration: RotationVector3D,
    pub accelerometer_calibration: AccelerationVector3D,
}

pub struct ConfigStorage {
    flash: EspNvs<NvsDefault>,
}

impl ConfigStorage {
    pub fn new() -> Self {
        let nvs_default_partition: EspNvsPartition<NvsDefault> =
            EspDefaultNvsPartition::take().unwrap();
        let namespace = "app_config";
        let flash = EspNvs::new(nvs_default_partition, namespace, true)
            .expect("Could not get nvs partition");
        ConfigStorage { flash }
    }
    pub fn store_to_flash(
        &mut self,
        config: AppStoredConfig,
    ) -> Result<(), AppError<Option<EspError>>> {
        let data_buffer = unsafe {
            core::slice::from_raw_parts(
                &config as *const _ as *mut u8,
                core::mem::size_of::<AppStoredConfig>(),
            )
        };
        let result = self.flash.set_blob(BLOB_NAME, data_buffer);
        if result.is_err() {
            Err(AppError {
                message: &"Failed to store the config",
                error: result.err(),
            })
        } else {
            Ok(())
        }
    }

    pub fn load_from_flash(&mut self) -> Result<AppStoredConfig, AppError<Option<EspError>>> {
        let config = AppStoredConfig::default();
        let data_buffer = unsafe {
            core::slice::from_raw_parts_mut(
                &config as *const _ as *mut u8,
                core::mem::size_of::<AppStoredConfig>(),
            )
        };
        let result = self.flash.get_blob(BLOB_NAME, data_buffer);
        if result.is_err() {
            Err(AppError {
                message: &"Failed to store the config",
                error: result.err(),
            })
        } else {
            Ok(config)
        }
    }
}
