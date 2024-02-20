#![feature(trait_alias)]
#![feature(ptr_metadata)]

mod communication_interfaces;
mod control;
mod output;
mod util;

use std::ffi::{c_void, CString};
use std::mem;
use std::sync::{Arc, RwLock};

use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::sys::{esp_pm_config_t, esp_pm_configure, vTaskDelete, xTaskCreatePinnedToCore};

use crate::communication_interfaces::wifi::{
    ControllerInput, WifiController, CONTROLLER_INPUT_DATA,
};
use crate::control::control_loops::start_flight_stabilizer;

use crate::communication_interfaces::i2c::*;

unsafe extern "C" fn comms_thread_task(params: *mut core::ffi::c_void) {
    let controller_input_ptr = params as *const _ as *const Arc<RwLock<ControllerInput>>;
    let controller_input = controller_input_ptr.read();
    // loop {
    //     let mut input_data = input_data_lock.write().unwrap();
    //     // log::info!("Kill {}", input_data.kill_motors);
    //     input_data.kill_motors = true;
    //     drop(input_data);
    //     FreeRtos::delay_ms(500);
    // }

    WifiController::init_monitor(13, controller_input.clone());
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

    let controller_input_shared = ControllerInput {
        yaw: 0,
        pitch: 0,
        roll: 0,
        throttle: 0,
        kill_motors: false,
        start: false,
        calibrate:false,
    };
    let control_input_values = Arc::new(RwLock::new(controller_input_shared));

    unsafe {
        xTaskCreatePinnedToCore(
            Some(comms_thread_task),
            CString::new("Comms Task").unwrap().as_ptr(),
            4096,
            &control_input_values.clone() as *const _ as *mut c_void,
            10,
            std::ptr::null_mut(),
            1,
        )
    };

    start_flight_stabilizer(control_input_values.clone());
    // loop {
    //     let data = control_input_values.read().unwrap();
    //     println!("Data {}", data.start);
    //     FreeRtos::delay_ms(1000);
    // }

    // unsafe {
    //     xTaskCreatePinnedToCore(
    //         Some(flight_control_thread_task),
    //         CString::new("Flight Task").unwrap().as_ptr(),
    //         4096,
    //         std::ptr::null_mut(),
    //         10,
    //         std::ptr::null_mut(),
    //         1,
    //     )
    // };
    // wifi_driver.set_configuration(&Configuration::Client(ClientConfiguration{}))
}
