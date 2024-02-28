#![feature(trait_alias)]
#![feature(ptr_metadata)]

mod communication_interfaces;
mod control;
mod drivers;
mod output;
mod shared_core_values;
mod threads;
mod util;
pub mod config {
    pub mod constants;
}

use crate::communication_interfaces::controller::RemoteControl;
use crate::communication_interfaces::ibus::IBusController;
use crate::communication_interfaces::wifi_control::WifiRemoteControl;
use crate::communication_interfaces::{i2c::*, ControllerTypes};
use crate::config::constants::{CONTROLLER_TYPE, WIFI_CONTROLLER_CHANNEL};
use crate::shared_core_values::INPUT_SHARED;
use crate::threads::{flight_thread, telemetry_thread};

use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::sys::{vTaskDelete, xTaskCreatePinnedToCore};
// use esp_idf_svc::sys::{
//     esp_pm_config_t, esp_pm_configure, esp_pm_lock_acquire, esp_pm_lock_handle_t,
//     esp_pm_lock_type_t_ESP_PM_CPU_FREQ_MAX, vTaskDelete, xTaskCreatePinnedToCore,
// };
use once_cell::sync::Lazy;
use std::ops::DerefMut;
use std::sync::Mutex;

pub static SHARED_PERIPHERALS: Lazy<Mutex<Peripherals>> =
    Lazy::new(|| Mutex::new(Peripherals::take().unwrap()));

unsafe extern "C" fn flight_thread_task(_params: *mut core::ffi::c_void) {
    flight_thread();
    vTaskDelete(std::ptr::null_mut());
}

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let _telemetry = std::thread::Builder::new().stack_size(4096).spawn(|| {
        telemetry_thread();
    });

    log::info!("Running");

    unsafe {
        xTaskCreatePinnedToCore(
            Some(flight_thread_task),
            std::ptr::null_mut(),
            4096,
            std::ptr::null_mut(),
            1,
            std::ptr::null_mut(),
            1,
        )
    };

    match CONTROLLER_TYPE {
        ControllerTypes::Wifi => {
            let controller = WifiRemoteControl::new(WIFI_CONTROLLER_CHANNEL);
            controller.init();
            controller.start_changes_monitor(&INPUT_SHARED)
        }
        ControllerTypes::Ibus => {
            let mut peripherals_lock = SHARED_PERIPHERALS.lock().unwrap();
            let controller = IBusController::new(peripherals_lock.deref_mut());
            drop(peripherals_lock);
            controller.start_changes_monitor(&INPUT_SHARED)
        }
    };
}
