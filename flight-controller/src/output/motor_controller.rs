use std::borrow::BorrowMut;

use esp_idf_svc::hal::ledc::{LedcChannel, LedcTimer};
use esp_idf_svc::hal::peripheral::Peripheral;
use esp_idf_svc::hal::prelude::*;
use esp_idf_svc::hal::{
    delay::FreeRtos,
    gpio::IOPin,
    ledc::{config::TimerConfig, LedcDriver, LedcTimerDriver, Resolution},
    peripherals::Peripherals,
};
use rayon::prelude::*;

const MIN_MOTOR_DUTY: u32 = 8192;
const MAX_MOTOR_DUTY: u32 = 16000;

pub struct MotorConfig<T, S, P>
where
    T: Peripheral + 'static,
    S: Peripheral + 'static,
    P: Peripheral + 'static,
{
    pub pin: T,
    pub timer: S,
    pub channel: P,
}

pub struct MotorsController {
    motor_drivers: Vec<LedcDriver<'static>>,
}

impl MotorsController {
    pub fn new<C: LedcChannel, T: LedcTimer>(
        motor_configs: Vec<MotorConfig<impl IOPin, impl Peripheral<P = T>, impl Peripheral<P = C>>>,
    ) -> Self {
        let config = TimerConfig::default()
            .frequency(4.kHz().into())
            .resolution(Resolution::Bits14);

        let drivers = motor_configs
            .into_iter()
            .map(|mut motor_config| {
                let timer_driver = LedcTimerDriver::new(motor_config.timer, &config).unwrap();
                LedcDriver::new(motor_config.channel, timer_driver, motor_config.pin).unwrap()
            })
            .collect();

        // let config = TimerConfig::default()
        //     .frequency(4.kHz().into())
        //     .resolution(Resolution::Bits14);

        // let timer_driver = LedcTimerDriver::new(peripherals.ledc.timer0, &config).unwrap();
        // let mut driver = LedcDriver::new(
        //     peripherals.ledc.channel0,
        //     timer_driver,
        //     peripherals.pins.gpio14,
        // )
        // .unwrap();

        // let drivers: Vec<LedcDriver> = vec![driver];
        MotorsController {
            motor_drivers: drivers,
        }
    }

    pub fn calibrate_drivers(&mut self) {
        self.motor_drivers.par_iter_mut().for_each(|driver| {
            log::info!("Set Max Duty");
            let _ = driver.set_duty(MAX_MOTOR_DUTY).unwrap();
            FreeRtos::delay_ms(10000);
            log::info!("Set min duty");
            let _ = driver.set_duty(MIN_MOTOR_DUTY).unwrap();
        });
    }

    pub fn set_motor_speed(&mut self, motor_number: u8, speed: u32) {
        self.motor_drivers[0].set_duty(speed);
    }
}
