use esp32_hal::{
    gpio::{self, GpioPin},
    ledc::{timer::Timer, HighSpeed, LEDC},
    peripheral::Peripheral,
    IO,
};

use crate::config::constants::{ESC_PWM_FREQUENCY_HZ, MAX_MOTOR_DUTY, MIN_MOTOR_DUTY};

use super::motor_controller::MotorController;

pub struct QuadcopterMotorsStateManager<'a> {
    motor_1_controller: MotorController<'a, GpioPin<gpio::Output<gpio::PushPull>, 13>, 13>,
    motor_2_controller: MotorController<'a, GpioPin<gpio::Output<gpio::PushPull>, 12>, 12>,
    motor_3_controller: MotorController<'a, GpioPin<gpio::Output<gpio::PushPull>, 14>, 14>,
    motor_4_controller: MotorController<'a, GpioPin<gpio::Output<gpio::PushPull>, 27>, 27>,
}

impl<'a> QuadcopterMotorsStateManager<'a> {
    pub fn new(io: &mut IO, ledc_driver: &'a LEDC<'a>, timer: &'a Timer<'a, HighSpeed>) -> Self {
        let motor_1_controller = MotorController::new(
            ledc_driver,
            timer,
            unsafe { io.pins.gpio13.clone_unchecked() }.into_push_pull_output(),
        );
        let motor_2_controller = MotorController::new(
            ledc_driver,
            timer,
            unsafe { io.pins.gpio12.clone_unchecked() }.into_push_pull_output(),
        );
        let motor_3_controller = MotorController::new(
            ledc_driver,
            timer,
            unsafe { io.pins.gpio14.clone_unchecked() }.into_push_pull_output(),
        );
        let motor_4_controller = MotorController::new(
            ledc_driver,
            timer,
            unsafe { io.pins.gpio27.clone_unchecked() }.into_push_pull_output(),
        );
        QuadcopterMotorsStateManager {
            motor_1_controller,
            motor_2_controller,
            motor_3_controller,
            motor_4_controller,
        }
    }

    pub fn set_motor_power(&mut self, values: [f32; 4]) {
        self.motor_1_controller.set_motor_speed(values[0]);
        self.motor_2_controller.set_motor_speed(values[1]);
        self.motor_3_controller.set_motor_speed(values[2]);
        self.motor_4_controller.set_motor_speed(values[3]);
    }

    pub fn kill_motors(&mut self) {
        self.set_motor_power([0.0, 0.0, 0.0, 0.0]);
        esp_println::println!("Killed motors");
    }
}
