#![feature(trait_alias)]

mod communication_interfaces;
mod control;
mod output;
mod util;

use esp_idf_svc::hal::{
    delay::FreeRtos,
    // ledc::{LedcDriver, LedcTimerDriver, Resolution},
    peripherals::Peripherals,
};
// use esp_idf_svc::systime::EspSystemTime;
use mpu6050::*;
// use rayon::prelude::*;
use std::sync::{Arc, RwLock};
use std::time::SystemTime;

use crate::control::control_loops::init_flight_stabilizer_thread;
use crate::control::integrator::Integrator;
use crate::control::kalman_filter::KalmanFilter;
use crate::output::motors_state_manager::MotorsStateManager;
use crate::{
    communication_interfaces::i2c::*,
    // output::motor_controller::{MotorConfig, MotorController},
};

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Running");
    // rayon::ThreadPoolBuilder::new().num_threads(4).stack_size(4096).build_global().unwrap();

    let mut peripherals: Peripherals = Peripherals::take().unwrap();

    let mut motors_states = MotorsStateManager::new();
    motors_states.initialize_motor_controllers(&mut peripherals);
    motors_states.initialize_esc();

    // let _ = std::thread::Builder::new().stack_size(4096).spawn(move || {

    //     motors_states.set_motor_power(vec![10.0_f32, 10.0_f32, 10.0_f32, 10.0_f32]);
    //     FreeRtos::delay_ms(12000);
    //     //     motors_states.set_motor_power(vec![50.0_f32, 50.0_f32, 50.0_f32, 50.0_f32]);
    //     //     FreeRtos::delay_ms(1500);
    //     motors_states.set_motor_power(vec![3.0_f32, 3.0_f32, 3.0_f32, 3.0_f32]);
    //     FreeRtos::delay_ms(2000);
    //     motors_states.set_motor_power(vec![0.0_f32, 0.0_f32, 0.0_f32, 0.0_f32]);
    // });

    // // let peripherals_arc = Arc::new(peripherals);

    let i2c_driver = get_i2c_driver(&mut peripherals);

    init_flight_stabilizer_thread(i2c_driver , motors_states);

}
