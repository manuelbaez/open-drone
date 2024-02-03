#![feature(trait_alias)]

mod communication_interfaces;
mod inertial_measurement;
mod output;
mod util;
use embedded_hal::delay::DelayNs;
use esp_idf_svc::hal::ledc::config::TimerConfig;
use esp_idf_svc::hal::prelude::*;
use esp_idf_svc::hal::{
    delay::FreeRtos,
    ledc::{LedcDriver, LedcTimerDriver, Resolution},
    peripherals::Peripherals,
};
use mpu6050::*;
use std::sync::{Arc, RwLock};

use crate::{
    communication_interfaces::i2c::I2CAdapterFactory,
    output::motor_controller::{MotorConfig, MotorsController},
};

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Running");

    let peripherals: Peripherals = Peripherals::take().unwrap();
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

    let motors_controller = MotorsController::new(
        vec![MotorConfig {
            pin: peripherals.pins.gpio14,
            channel: peripherals.ledc.channel0,
            timer: peripherals.ledc.timer0,
        }],
    );

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
