#![feature(trait_alias)]

mod communication_interfaces;
mod inertial_measurement;
mod output;
mod util;
use embedded_hal::delay::DelayNs;
use esp_idf_svc::hal::ledc::config::TimerConfig;
use esp_idf_svc::hal::peripheral::Peripheral;
use esp_idf_svc::hal::prelude::*;
use esp_idf_svc::hal::{
    delay::FreeRtos,
    ledc::{LedcDriver, LedcTimerDriver, Resolution},
    peripherals::Peripherals,
};
use mpu6050::*;
use rayon::prelude::*;
use std::sync::{Arc, RwLock};

use crate::output::motors_state_manager::MotorsStateManager;
use crate::{
    communication_interfaces::i2c::I2CAdapterFactory,
    output::motor_controller::{MotorConfig, MotorController},
};

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Running");
    // rayon::ThreadPoolBuilder::new().num_threads(4).stack_size(4096).build_global().unwrap();

    let peripherals: Peripherals = Peripherals::take().unwrap();

    let mut motors_states = MotorsStateManager::new();
    motors_states.initialize_motor_controllers(peripherals);
    motors_states.initialize_esc();
    motors_states.set_motor_values(vec![1.0_f32, 1.0_f32, 1.0_f32, 1.0_f32]);
    FreeRtos::delay_ms(2000);
    motors_states.set_motor_values(vec![100.0_f32, 100.0_f32, 100.0_f32, 100.0_f32]);
    FreeRtos::delay_ms(1500);
    motors_states.set_motor_values(vec![0.0_f32, 0.0_f32, 0.0_f32, 0.0_f32]);

    // // let peripherals_arc = Arc::new(peripherals);

    // let i2c_driver = I2CAdapterFactory::get_i2c_adapter(peripherals);

    // let mut mpu = Mpu6050::new(i2c_driver);

    // mpu.init(&mut FreeRtos).unwrap();
    // let mpu_lock = Arc::new(RwLock::new(mpu));
    // {
    //     let mpu_lock = mpu_lock.clone();
    //     let _ = std::thread::Builder::new().stack_size(4096).spawn(move || {
    //         let shared_mpu = mpu_lock;
    //         loop {
    //             let mut driver = shared_mpu.write().unwrap();
    //             let acc = driver.get_acc_angles().unwrap();
    //             println!("r/p: {:?}", acc);
    //             drop(driver);
    //             FreeRtos::delay_ms(5);
    //         }
    //     });
    // }

    // {
    //     let mpu_lock = mpu_lock.clone();
    //     let _ = std::thread::Builder::new().stack_size(4096).spawn(move || {
    //         let shared_mpu = mpu_lock;
    //         loop {
    //             let mut driver = shared_mpu.write().unwrap();
    //             let gyro = driver.get_gyro().unwrap();
    //             println!("gyro: {:?}", gyro);
    //             drop(driver);
    //             FreeRtos::delay_ms(5);
    //         }
    //     });
    // }

    // drivers[0].calibrate_driver();

    // drivers.iter_mut().for_each(|driver| {
    //     driver.set_motor_speed(10.0_f32);
    // });
    // FreeRtos::delay_ms(5000);
    // drivers.iter_mut().for_each(|driver| {
    //     driver.set_motor_speed(50.0_f32);
    // });
    // FreeRtos::delay_ms(1000);
    // drivers.iter_mut().for_each(|driver| {
    //     driver.set_motor_speed(70.0_f32);
    // });
    // FreeRtos::delay_ms(1000);
    // drivers.iter_mut().for_each(|driver| {
    //     driver.set_motor_speed(0.0_f32);
    // });
    // motor_2_controller.calibrate_driver();
    // motor_2_controller.set_motor_speed(20.0_f32);
    // let config = TimerConfig::default()
    //     .frequency(4.kHz().into())
    //     .resolution(Resolution::Bits14);

    // // let timer = TimerDriver::new(peripherals.ledc.timer0, &config);
    // FreeRtos::delay_ms(5000);

    // let timer_driver = LedcTimerDriver::new(peripherals.ledc.timer0, &config).unwrap();
    // let mut driver = LedcDriver::new(
    //     peripherals.ledc.channel0,
    //     timer_driver,
    //     peripherals.pins.gpio14,
    // )
    // .unwrap();

    // log::info!("Set 250us");
    // let _ = driver.set_duty(16000).unwrap();
    // FreeRtos::delay_ms(10000);

    // log::info!("Set 125us");
    // let _ = driver.set_duty(8192).unwrap();

    // FreeRtos::delay_ms(10000);
    // log::info!("Set low 140us");
    // let _ = driver.set_duty(9175).unwrap();
    // FreeRtos::delay_ms(5000);

    // log::info!("Set low full power");
    // let _ = driver.set_duty(16000).unwrap();

    // FreeRtos::delay_ms(1000);
    // log::info!("Set off");
    // let _ = driver.set_duty(8192).unwrap();

    // let timer_driver = LedcTimerDriver::new(peripherals.ledc.timer1, &config).unwrap();
    // let mut driver = LedcDriver::new(
    //     peripherals.ledc.channel1,
    //     timer_driver,
    //     peripherals.pins.gpio27,
    // )
    // .unwrap();

    // log::info!("Set 2ms");
    // let _ = driver.set_duty(1638).unwrap();
    // FreeRtos::delay_ms(10000);

    // log::info!("Set 1ms");
    // let _ = driver.set_duty(820).unwrap();

    // FreeRtos::delay_ms(10000);
    // log::info!("Set low");
    // let _ = driver.set_duty(1065).unwrap();

    // FreeRtos::delay_ms(5000);
    // log::info!("Set 1.0ms");
    // let _ = driver.set_duty(820).unwrap();

    loop {
        FreeRtos::delay_ms(1000);
    }
}
