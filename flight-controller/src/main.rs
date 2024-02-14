#![feature(trait_alias)]

mod communication_interfaces;
mod control;
mod output;
mod util;

use std::ffi::c_void;

use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::sys::{esp_pm_config_t, esp_pm_configure};

use crate::communication_interfaces::wifi::WifiSniffer;
use crate::control::control_loops::init_flight_stabilizer_thread;

use crate::communication_interfaces::i2c::*;
use crate::output::motors_state_manager::MotorsStateManager;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Running");

    let mut peripherals: Peripherals = Peripherals::take().unwrap();

    let mut motors_states = MotorsStateManager::new();
    motors_states.initialize_motor_controllers(&mut peripherals);

    let i2c_driver = get_i2c_driver(&mut peripherals);

    // init_flight_stabilizer_thread(i2c_driver, motors_states);

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

    WifiSniffer::init_promiscous(4);

    loop {
        FreeRtos::delay_ms(1000);
    }
    // wifi_driver.set_configuration(&Configuration::Client(ClientConfiguration{}))
}
