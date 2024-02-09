#![feature(trait_alias)]

mod communication_interfaces;
mod control;
mod output;
mod util;

use esp_idf_svc::hal::peripherals::Peripherals;

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

    init_flight_stabilizer_thread(i2c_driver, motors_states);
}
