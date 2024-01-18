use esp_idf_svc::hal::delay::FreeRtos;

mod comms;
mod inertial_measurement;
use crate::comms::i2c::init_i2c_driver;
use crate::inertial_measurement::accelerometer::{get_accelerometer_values, init_accelerometer};

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    log::info!("Running");
    let mut i2c_driver = init_i2c_driver();
    init_accelerometer(&mut i2c_driver);
    loop {
        let values = get_accelerometer_values(&mut i2c_driver);
        log::info!("Read x{} y{} z{}", values[0], values[1], values[2]);
        FreeRtos::delay_ms(1000);
    }
}
