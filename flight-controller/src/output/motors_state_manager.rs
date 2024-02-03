use esp_idf_svc::hal::{
    peripheral::Peripheral,
    peripherals::{self, Peripherals},
};

use super::motor_controller::{MotorConfig, MotorController};

pub struct MotorsStateManager {
    controllers: Vec<MotorController>,
}

impl MotorsStateManager {
    pub fn new() -> Self {
        MotorsStateManager {
            controllers: Vec::with_capacity(4),
        }
    }

    pub fn initialize_motor_controllers(&mut self, mut peripherals: Peripherals) {
        let mut motor_1_controller = MotorController::new(MotorConfig {
            pin: peripherals.pins.gpio14,
            channel: peripherals.ledc.channel0,
            timer: unsafe { peripherals.ledc.timer0.clone_unchecked() },
        });
        self.controllers.push(motor_1_controller);

        let mut motor_2_controller = MotorController::new(MotorConfig {
            pin: peripherals.pins.gpio13,
            channel: peripherals.ledc.channel1,
            timer: unsafe { peripherals.ledc.timer0.clone_unchecked() },
        });
        self.controllers.push(motor_2_controller);

        let mut motor_3_controller = MotorController::new(MotorConfig {
            pin: peripherals.pins.gpio12,
            channel: peripherals.ledc.channel2,
            timer: unsafe { peripherals.ledc.timer0.clone_unchecked() },
        });
        self.controllers.push(motor_3_controller);

        let mut motor_4_controller = MotorController::new(MotorConfig {
            pin: peripherals.pins.gpio27,
            channel: peripherals.ledc.channel3,
            timer: unsafe { peripherals.ledc.timer0.clone_unchecked() },
        });
        self.controllers.push(motor_4_controller);
    }

    pub fn initialize_esc(&mut self) {
        self.controllers[0].calibrate_esc();
    }

    pub fn set_motor_values(&mut self, values: Vec<f32>) {
        for (index, value) in values.into_iter().enumerate() {
            self.controllers[index].set_motor_speed(value);
        }
    }
}
