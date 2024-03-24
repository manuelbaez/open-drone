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
mod tasks;
mod util;

use crate::communication_interfaces::i2c::*;
use crate::tasks::{flight_task_start, measurements_task_start, telemetry_task_start};
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::sys::xTaskCreatePinnedToCore;
use once_cell::sync::Lazy;
use std::sync::Mutex;
use tasks::{comms_task_start, imu_measurements_task_start};

pub static SHARED_PERIPHERALS: Lazy<Mutex<Peripherals>> =
    Lazy::new(|| Mutex::new(Peripherals::take().unwrap()));

unsafe extern "C" fn imu_measurements_task(_params: *mut core::ffi::c_void) {
    imu_measurements_task_start();
}

unsafe extern "C" fn flight_thread_task(_params: *mut core::ffi::c_void) {
    flight_task_start();
}

unsafe extern "C" fn telemetry_thread_task(_params: *mut core::ffi::c_void) {
    telemetry_task_start();
}

unsafe extern "C" fn measurements_thread_task(_params: *mut core::ffi::c_void) {
    measurements_task_start();
}

unsafe extern "C" fn comms_thread_task(_params: *mut core::ffi::c_void) {
    comms_task_start();
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
