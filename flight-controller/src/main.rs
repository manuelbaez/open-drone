// use std::borrow::BorrowMut;
// use std::sync::Mutex;
// use esp_idf_svc::hal::delay::FreeRtos;
// use esp_idf_svc::hal::i2c::I2cDriver;

mod communication_interfaces;
mod inertial_measurement;
mod util;
use crate::{
    communication_interfaces::{i2c::I2CAdapterFactory, i2c_adapter::I2CAdapter},
    inertial_measurement::accelerometer::{get_accelerometer_xyz_values, init_accelerometer},
};

fn get_accelerometer_data_task(i2c_adapter: &mut impl I2CAdapter) {
    let mut values: [f32; 3];
    loop {
        values = get_accelerometer_xyz_values(i2c_adapter);
        log::info!("Read x{} y{} z{}", values[0], values[1], values[2]);
        // FreeRtos::delay_ms(1);
    }
}

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    log::info!("Running");
    let mut i2c_adapter = I2CAdapterFactory::get_i2c_adapter();
    // let mut i2c_driver = init_i2c_driver();
    let mut thread_list = Vec::with_capacity(2);

    init_accelerometer(&mut i2c_adapter);

    // let driver_mutex: Mutex<_> = Mutex::new(i2c_adapter);

    let acceleromenter_thread = std::thread::Builder::new().stack_size(4096).spawn(move || {
        // let mut driver = driver_mutex.lock().unwrap();
        get_accelerometer_data_task(&mut i2c_adapter);
    });

    thread_list.push(acceleromenter_thread);

    for handle in thread_list {
        handle.expect("Failed thread");
    }
}
