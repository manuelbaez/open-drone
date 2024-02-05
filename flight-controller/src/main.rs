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
use crate::control::kalman_filter::{self, KalmanFilter};
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
        [0.0_f64, 0.0_f64],
        [0.0_f64, 0.0_f64],
        [0.0_f64, 0.0_f64],
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

            let mut pitch_kalman_filter: KalmanFilter =
                KalmanFilter::new(GYRO_DRIFT_DEG, ACCEL_UNCERTAINTY_DEG);
            let mut roll_kalman_filter: KalmanFilter =
                KalmanFilter::new(GYRO_DRIFT_DEG, ACCEL_UNCERTAINTY_DEG);

            loop {
                let mut driver = shared_mpu.write().unwrap();
                let gyro_angles = driver.get_gyro().unwrap();
                let accelerometer_roll_pitch = driver.get_acc_angles().unwrap();

                let current_time_us: u128 = system_time.elapsed().unwrap().as_micros();

                let pitch_rate_deg = gyro_angles[1] as f64 * 180.0_f64 / std::f64::consts::PI;
                let roll_rate_deg = gyro_angles[0] as f64 * 180.0_f64 / std::f64::consts::PI;

                let accel_pitch_deg =
                    accelerometer_roll_pitch[1] as f64 * 180.0_f64 / std::f64::consts::PI;
                let accel_roll_deg =
                    accelerometer_roll_pitch[0] as f64 * 180.0_f64 / std::f64::consts::PI;

                let time_since_last_reading_seconds =
                    (current_time_us - previous_time_us) as f64 / 1_000_000.0_f64;
                previous_time_us = current_time_us;

                pitch_kalman_filter.apply_flilter_update(
                    pitch_rate_deg,
                    accel_pitch_deg,
                    time_since_last_reading_seconds,
                );

                roll_kalman_filter.apply_flilter_update(
                    roll_rate_deg,
                    accel_roll_deg,
                    time_since_last_reading_seconds,
                );


                let mut angles = gyro_angles_kalman.write().unwrap();

                angles[0][0] = roll_kalman_filter.get_current_state();
                angles[0][1] = roll_kalman_filter.get_current_uncertainty();

                angles[1][0] = pitch_kalman_filter.get_current_state();
                angles[1][1] = pitch_kalman_filter.get_current_uncertainty();


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

    // loop {
    //     FreeRtos::delay_ms(1000);
    // }
}
