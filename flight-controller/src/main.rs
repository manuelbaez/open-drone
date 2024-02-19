#![feature(trait_alias)]
#![feature(ptr_metadata)]

mod communication_interfaces;
mod control;
mod output;
mod util;

use std::ffi::{c_void, CString};

use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::sys::{esp_pm_config_t, esp_pm_configure, vTaskDelete, xTaskCreatePinnedToCore};

use crate::communication_interfaces::wifi::WifiSniffer;
use crate::control::control_loops::init_flight_stabilizer_thread;

use crate::communication_interfaces::i2c::*;
use crate::output::motors_state_manager::MotorsStateManager;

unsafe extern "C" fn comms_thread_task(params: *mut core::ffi::c_void) {
    WifiSniffer::init_promiscous(13);
    vTaskDelete(std::ptr::null_mut());
}

unsafe extern "C" fn flight_control_thread_task(params: *mut core::ffi::c_void) {
    let mut peripherals: Peripherals = Peripherals::take().unwrap();

    let mut motors_states = MotorsStateManager::new();
    motors_states.initialize_motor_controllers(&mut peripherals);

    let i2c_driver = get_i2c_driver(&mut peripherals);

    init_flight_stabilizer_thread(i2c_driver, motors_states);
    vTaskDelete(std::ptr::null_mut());
}
fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Running");

    unsafe {
        let config = esp_pm_config_t {
            max_freq_mhz: 240,
            min_freq_mhz: 240,
            light_sleep_enable: false,
        };
        let config_pointer: *const c_void = &config as *const _ as *const c_void;
        esp_pm_configure(config_pointer);
        log::info!("Set core frequency");
    }

    unsafe {
        xTaskCreatePinnedToCore(
            Some(comms_thread_task),
            CString::new("Comms Task").unwrap().as_ptr(),
            4096,
            std::ptr::null_mut(),
            10,
            std::ptr::null_mut(),
            0,
        )
    };

    unsafe {
        xTaskCreatePinnedToCore(
            Some(flight_control_thread_task),
            CString::new("Flight Task").unwrap().as_ptr(),
            4096,
            std::ptr::null_mut(),
            10,
            std::ptr::null_mut(),
            1,
        )
    };
    // wifi_driver.set_configuration(&Configuration::Client(ClientConfiguration{}))
}
