use esp_idf_svc::hal::{
    ledc::{config::TimerConfig, LedcDriver, LedcTimerDriver, Resolution},
    peripherals::Peripherals,
};

use esp_idf_svc::hal::prelude::*;

pub struct MotorsController;

impl MotorsController {
    pub fn new(peripherals: Peripherals) {
        let timer_driver = LedcTimerDriver::new(
            peripherals.ledc.timer0,
            &TimerConfig::default()
                .frequency(400.Hz())
                .resolution(Resolution::Bits14),
        )
        .unwrap();
        let mut driver = LedcDriver::new(
            peripherals.ledc.channel0,
            timer_driver,
            peripherals.pins.gpio14,
        )
        .unwrap();

        let _ = driver.set_duty(16000).unwrap();
    }
}
