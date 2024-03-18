#![feature(trait_alias)]
#![feature(ptr_metadata)]
#![feature(const_float_bits_conv)]
#![feature(generic_const_exprs)]

mod communication_interfaces;
mod config;
mod control;
mod drivers;
mod output;
mod shared_core_values;
mod threads;
mod util;

use crate::communication_interfaces::controller::{RemoteControl, RemoteTelemetry};
use crate::communication_interfaces::i2c::*;
use crate::shared_core_values::{SHARED_CONTROLLER_INPUT, SHARED_TELEMETRY};
use crate::threads::{flight_thread, measurements_thread, telemetry_thread};
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::sys::xTaskCreatePinnedToCore;
use once_cell::sync::Lazy;
use std::ops::DerefMut;
use std::sync::Mutex;
use threads::imu_measurements_task_start;

#[cfg(feature = "ibus-controller")]
use {
    crate::communication_interfaces::ibus::controller::IBusController,
    crate::communication_interfaces::ibus::telemetry::IBusTelemetry,
};
#[cfg(feature = "wifi")]
use {
    crate::communication_interfaces::wifi_control::WifiRemoteControl,
    crate::config::constants::WIFI_CONTROLLER_CHANNEL,
};

pub static SHARED_PERIPHERALS: Lazy<Mutex<Peripherals>> =
    Lazy::new(|| Mutex::new(Peripherals::take().unwrap()));

unsafe extern "C" fn imu_measurements_task(_params: *mut core::ffi::c_void) {
    imu_measurements_task_start();
}

unsafe extern "C" fn flight_thread_task(_params: *mut core::ffi::c_void) {
    flight_thread();
}

unsafe extern "C" fn telemetry_thread_task(_params: *mut core::ffi::c_void) {
    telemetry_thread();
}

unsafe extern "C" fn measurements_thread_task(_params: *mut core::ffi::c_void) {
    measurements_thread();
}

unsafe extern "C" fn comms_thread_task(_params: *mut core::ffi::c_void) {
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
        let _telemetry: Result<std::thread::JoinHandle<()>, std::io::Error> =
            std::thread::Builder::new().stack_size(4096).spawn(|| {
                let mut peripherals_lock = SHARED_PERIPHERALS.lock().unwrap();
                let telemetry_controller = IBusTelemetry::new(peripherals_lock.deref_mut());
                drop(peripherals_lock);
                telemetry_controller.start_telemetry_tx_loop(&SHARED_TELEMETRY);
            });
        let mut peripherals_lock = SHARED_PERIPHERALS.lock().unwrap();
        let controller = IBusController::new(peripherals_lock.deref_mut());
        drop(peripherals_lock);
        controller.start_input_changes_monitor(&SHARED_CONTROLLER_INPUT)
    }
}

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

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

    unsafe {
        xTaskCreatePinnedToCore(
            Some(imu_measurements_task),
            std::ptr::null_mut(),
            4096,
            std::ptr::null_mut(),
            1,
            std::ptr::null_mut(),
            1,
        )
    };

    unsafe {
        xTaskCreatePinnedToCore(
            Some(telemetry_thread_task),
            std::ptr::null_mut(),
            3072,
            std::ptr::null_mut(),
            3,
            std::ptr::null_mut(),
            0,
        )
    };

    unsafe {
        xTaskCreatePinnedToCore(
            Some(measurements_thread_task),
            std::ptr::null_mut(),
            3072,
            std::ptr::null_mut(),
            2,
            std::ptr::null_mut(),
            0,
        )
    };

    unsafe {
        xTaskCreatePinnedToCore(
            Some(comms_thread_task),
            std::ptr::null_mut(),
            4096,
            std::ptr::null_mut(),
            1,
            std::ptr::null_mut(),
            0,
        )
    };
}
