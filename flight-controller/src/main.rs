#![feature(trait_alias)]
#![feature(ptr_metadata)]
#![feature(const_float_bits_conv)]

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
use crate::communication_interfaces::i2c::*;
#[cfg(feature = "ibus-controller")]
use crate::communication_interfaces::ibus::IBusController;
use crate::shared_core_values::SHARED_CONTROLLER_INPUT;
use crate::threads::{flight_thread, telemetry_thread};
#[cfg(any(feature = "wifi-controller", feature = "wifi-tuning"))]
use {
    crate::communication_interfaces::wifi_control::WifiRemoteControl,
    crate::config::constants::WIFI_CONTROLLER_CHANNEL,
};

use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::sys::{vTaskDelete, xTaskCreatePinnedToCore};
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

    #[cfg(feature = "wifi")]
    {
        let controller = WifiRemoteControl::new(WIFI_CONTROLLER_CHANNEL);
        controller.init();
        #[cfg(feature = "wifi-controller")]
        controller.start_input_changes_monitor(&SHARED_CONTROLLER_INPUT);
        #[cfg(feature = "wifi-tuning")]
        controller.start_tuing_monitor();
    }

    #[cfg(all(not(feature = "wifi-controller"), feature = "ibus-controller"))]
    {
        let mut peripherals_lock = SHARED_PERIPHERALS.lock().unwrap();
        let controller = IBusController::new(peripherals_lock.deref_mut());
        drop(peripherals_lock);
        controller.start_input_changes_monitor(&SHARED_CONTROLLER_INPUT)
    }
}
