#![feature(trait_alias)]

mod communication_interfaces;
mod control;
mod inertial_measurement;
mod output;
mod util;

// use core::time::Duration;
// use esp_idf_svc::hal::ledc::config::TimerConfig;
// use esp_idf_svc::hal::peripheral::Peripheral;
// use esp_idf_svc::hal::prelude::*;
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

use crate::control::integrator::Integrator;
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

    // let _ = std::thread::Builder::new().stack_size(4096).spawn(move || {
    //     motors_states.initialize_esc();

    //     motors_states.set_motor_power(vec![30.0_f32, 30.0_f32, 30.0_f32, 30.0_f32]);
    //     FreeRtos::delay_ms(12000);
    //     motors_states.set_motor_power(vec![50.0_f32, 50.0_f32, 50.0_f32, 50.0_f32]);
    //     FreeRtos::delay_ms(1500);
    //     motors_states.set_motor_power(vec![3.0_f32, 3.0_f32, 3.0_f32, 3.0_f32]);
    //     FreeRtos::delay_ms(2000);
    //     motors_states.set_motor_power(vec![0.0_f32, 0.0_f32, 0.0_f32, 0.0_f32]);
    // });

    // // let peripherals_arc = Arc::new(peripherals);

    let i2c_driver = get_i2c_driver(&mut peripherals);

    let mut mpu = Mpu6050::new(i2c_driver);

    mpu.init(&mut FreeRtos).unwrap();

    // let gyro_angles_integrated = Arc::new(RwLock::new([
    //     Integrator::new(),
    //     Integrator::new(),
    //     Integrator::new(),
    // ])); // roll-pitch-yaw

    let gyro_angles_kalman = Arc::new(RwLock::new([
        [0.0_f64, 0.0_f64, 0.0_f64],
        [0.0_f64, 0.0_f64, 0.0_f64],
        [0.0_f64, 0.0_f64, 0.0_f64],
    ])); // roll-pitch-yaw

    let mpu_lock = Arc::new(RwLock::new(mpu));
    {
        let mpu_lock = mpu_lock.clone();
        let gyro_angles_kalman = gyro_angles_kalman.clone();

        let _ = std::thread::Builder::new().stack_size(4096).spawn(move || {
            let shared_mpu = mpu_lock;

            let system_time = SystemTime::now();

            //kalman filter variables
            let mut previous_time_us = 0_u128;
            const GYRO_DRIFT_DEG: f64 = 3.0_f64;
            const ACCEL_UNCERTAINTY_DEG: f64 = 3.0_f64;
            // let mut kamal_gain = 0.0_f64;
            let mut kalmal_pitch_prediction: f64 = 0.0_f64;
            let mut kalman_pitch_uncertainty = 0.0_f64;

            loop {
                let mut driver = shared_mpu.write().unwrap();
                let gyro_angles = driver.get_gyro().unwrap();
                let accelerometer_roll_pitch = driver.get_acc_angles().unwrap();

                let current_time_us: u128 = system_time.elapsed().unwrap().as_micros();

                let pitch_rate_deg = gyro_angles[1] as f64 * 180.0_f64 / std::f64::consts::PI;

                let accel_pitch_deg =
                    accelerometer_roll_pitch[1] as f64 * 180.0_f64 / std::f64::consts::PI;

                let time_since_last_reading_seconds =
                    (current_time_us - previous_time_us) as f64 / 1_000_000.0_f64;
                previous_time_us = current_time_us;

                // Apply pitch kalman filter

                kalmal_pitch_prediction = kalmal_pitch_prediction
                    + time_since_last_reading_seconds * pitch_rate_deg as f64;

                kalman_pitch_uncertainty = kalman_pitch_uncertainty
                    + time_since_last_reading_seconds.powf(2.0) * GYRO_DRIFT_DEG.powf(2.0);

                let kamal_gain = kalman_pitch_uncertainty
                    / (kalman_pitch_uncertainty + ACCEL_UNCERTAINTY_DEG.powf(2.0));

                kalmal_pitch_prediction = kalmal_pitch_prediction
                    + kamal_gain * (accel_pitch_deg as f64 - kalmal_pitch_prediction);

                kalman_pitch_uncertainty = (1.0_f64 - kamal_gain) * kalman_pitch_uncertainty;

                let mut angles = gyro_angles_kalman.write().unwrap();
                angles[1][0] = kalmal_pitch_prediction;
                angles[1][1] = kalman_pitch_uncertainty;
                angles[1][2] = kamal_gain;

                drop(driver);
                drop(angles);
            }
        });
    }

    {
        let gyro_angles_kalman = gyro_angles_kalman.clone();

        let _ = std::thread::Builder::new()
            .stack_size(4096)
            .spawn(move || loop {
                let angles = gyro_angles_kalman.read().unwrap();
                print!("Curent rotation {:?}\n", angles);
                drop(angles);
                FreeRtos::delay_ms(500);
            });
    }

    // for _ in [1..5] {
    //     //
    // }
    // let gyro_value = mpu.get_gyro().unwrap();

    // let time_b = system_time.elapsed().unwrap().as_micros();

    // print!("Cuurent time {} {} \n", time_b - time_a,  gyro_value[0]);
    loop {
        FreeRtos::delay_ms(1000);
    }
}
