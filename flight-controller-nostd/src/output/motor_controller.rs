use esp32_hal::{
    gpio::{GpioPin, GpioProperties, Output, OutputPin, PushPull},
    ledc::{
        channel::{self, config::PinConfig, ChannelHW, ChannelIFace},
        timer::Timer,
        HighSpeed, LEDC,
    },
    peripheral::Peripheral,
};

use crate::config::constants::{MAX_MOTOR_DUTY, MIN_MOTOR_DUTY};

pub struct MotorController<'a, O, const PIN_NUM: u8>
where
    O: OutputPin,
{
    pwm_channel: channel::Channel<'a, HighSpeed, O>,
}

impl<'a, O, const PIN_NUM: u8> MotorController<'a, O, PIN_NUM>
where
    O: OutputPin,
    GpioPin<Output<PushPull>, PIN_NUM>: GpioProperties + OutputPin + Peripheral<P = O>,
{
    pub fn new(
        ledc_driver: &'a LEDC<'a>,
        timer: &'a Timer<'a, HighSpeed>,
        pin: GpioPin<Output<PushPull>, PIN_NUM>,
    ) -> Self {
        let mut pwm_channel =
            ledc_driver.get_channel::<HighSpeed, _>(channel::Number::Channel0, pin);

        let channel_config = channel::config::Config {
            timer,
            pin_config: PinConfig::PushPull,
            duty_pct: 0,
        };
        pwm_channel.configure(channel_config).unwrap();
        pwm_channel.set_duty_hw(MIN_MOTOR_DUTY);

        MotorController { pwm_channel }
    }
    // #[allow(dead_code)]
    // pub fn calibrate_esc(&mut self) {
    //     let _ = self.motor_driver.set_duty(0_u32).unwrap();
    //     Timer::after(Duration::from_millis(1000)).await;
    //     log::info!("Set Max Duty");
    //     let _ = self.motor_driver.set_duty(MAX_MOTOR_DUTY).unwrap();
    //     Timer::after(Duration::from_millis(5000)).await;

    //     log::info!("Set min duty");
    //     let _ = self.motor_driver.set_duty(MIN_MOTOR_DUTY).unwrap();
    //     FreeRtos::delay_ms(8000);
    //     log::info!("Calibrated!!");
    // }

    pub fn set_motor_speed(&mut self, speed: f32) {
        let duty: u32 = (((MAX_MOTOR_DUTY - MIN_MOTOR_DUTY) as f32) * speed / 100.0_f32
            + MIN_MOTOR_DUTY as f32) as u32;
        self.pwm_channel.set_duty_hw(duty);
    }
}
