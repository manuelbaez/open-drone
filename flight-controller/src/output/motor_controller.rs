use esp_idf_svc::hal::ledc::{LedcChannel, LedcTimer};
use esp_idf_svc::hal::peripheral::Peripheral;
use esp_idf_svc::hal::prelude::*;
use esp_idf_svc::hal::{
    delay::FreeRtos,
    gpio::IOPin,
    ledc::{config::TimerConfig, LedcDriver, LedcTimerDriver, Resolution},
};

use crate::config::constants::{ESC_PWM_FREQUENCY_HZ, MAX_MOTOR_DUTY, MIN_MOTOR_DUTY};

pub struct MotorConfig<TPin, TTimer, TChannel>
where
    TPin: Peripheral + 'static,
    TTimer: Peripheral + 'static,
    TChannel: Peripheral + 'static,
{
    pub pin: TPin,
    pub timer: TTimer,
    pub channel: TChannel,
}

pub struct MotorController {
    motor_driver: LedcDriver<'static>,
}

impl MotorController {
    pub fn new<C: LedcChannel, T: LedcTimer>(
        motor_config: MotorConfig<impl IOPin, impl Peripheral<P = T>, impl Peripheral<P = C>>,
    ) -> Self {
        let config = TimerConfig::default()
            .frequency(ESC_PWM_FREQUENCY_HZ.Hz().into())
            .resolution(Resolution::Bits14);

        let timer_driver = LedcTimerDriver::new(motor_config.timer, &config).unwrap();
        let mut driver =
            LedcDriver::new(motor_config.channel, timer_driver, motor_config.pin).unwrap();
        let _ = driver.set_duty(MIN_MOTOR_DUTY).unwrap();

        MotorController {
            motor_driver: driver,
        }
    }

    pub fn calibrate_esc(&mut self) {
        log::info!("Set Max Duty");
        let _ = self.motor_driver.set_duty(MAX_MOTOR_DUTY).unwrap();
        FreeRtos::delay_ms(5000);
        log::info!("Set min duty");
        let _ = self.motor_driver.set_duty(MIN_MOTOR_DUTY).unwrap();
        FreeRtos::delay_ms(8000);
    }

    pub fn set_motor_speed(&mut self, speed: f32) {
        let duty = (((MAX_MOTOR_DUTY - MIN_MOTOR_DUTY) as f32) * speed / 100.0_f32
            + MIN_MOTOR_DUTY as f32) as u32;
        // log::info!("Set duty {}-{}", speed, duty);
        let _ = self.motor_driver.set_duty(duty).unwrap();
    }
}
