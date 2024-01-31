#![feature(trait_alias)]

mod communication_interfaces;
mod inertial_measurement;
mod util;
use std::sync::{Arc, RwLock};

use esp_idf_svc::hal::delay::FreeRtos;
use mpu6050::*;

use crate::communication_interfaces::i2c::I2CAdapterFactory;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Running");

    let i2c_driver = I2CAdapterFactory::get_i2c_adapter();

    let mut mpu = Mpu6050::new(i2c_driver);

    mpu.init(&mut FreeRtos).unwrap();
    let mpu_lock = Arc::new(RwLock::new(mpu));
    {
        let mpu_lock = mpu_lock.clone();
        let _ = std::thread::Builder::new().stack_size(4096).spawn(move || {
            let shared_mpu = mpu_lock;
            loop {
                let mut driver = shared_mpu.write().unwrap();
                let acc = driver.get_acc_angles().unwrap();
                println!("r/p: {:?}", acc);
                drop(driver);
                FreeRtos::delay_ms(5);
            }
        });
    }

    {
        let mpu_lock = mpu_lock.clone();
        let _ = std::thread::Builder::new().stack_size(4096).spawn(move || {
            let shared_mpu = mpu_lock;
            loop {
                let mut driver = shared_mpu.write().unwrap();
                let gyro = driver.get_gyro().unwrap();
                println!("gyro: {:?}", gyro);
                drop(driver);
                FreeRtos::delay_ms(5);
            }
        });
    }

}
