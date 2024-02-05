use esp_idf_svc::hal::{peripheral::Peripheral, peripherals::Peripherals};

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

    pub fn initialize_motor_controllers(&mut self, peripherals: &mut Peripherals) {
        let motor_1_controller = MotorController::new(MotorConfig {
            pin: unsafe { peripherals.pins.gpio14.clone_unchecked() },
            channel: unsafe { peripherals.ledc.channel0.clone_unchecked() },
            timer: unsafe { peripherals.ledc.timer0.clone_unchecked() },
        });
        self.controllers.push(motor_1_controller);

        let motor_2_controller = MotorController::new(MotorConfig {
            pin: unsafe { peripherals.pins.gpio13.clone_unchecked() },
            channel: unsafe { peripherals.ledc.channel1.clone_unchecked() },
            timer: unsafe { peripherals.ledc.timer0.clone_unchecked() },
        });
        self.controllers.push(motor_2_controller);

        let motor_3_controller = MotorController::new(MotorConfig {
            pin: unsafe { peripherals.pins.gpio12.clone_unchecked() },
            channel: unsafe { peripherals.ledc.channel2.clone_unchecked() },
            timer: unsafe { peripherals.ledc.timer0.clone_unchecked() },
        });
        self.controllers.push(motor_3_controller);

        let motor_4_controller = MotorController::new(MotorConfig {
            pin: unsafe { peripherals.pins.gpio27.clone_unchecked() },
            channel: unsafe { peripherals.ledc.channel3.clone_unchecked() },
            timer: unsafe { peripherals.ledc.timer0.clone_unchecked() },
        });
        self.controllers.push(motor_4_controller);
    }

    pub fn initialize_esc(&mut self) {
        self.controllers[0].calibrate_esc();
    }

    pub fn set_motor_power(&mut self, values: Vec<f32>) {
        for (index, value) in values.into_iter().enumerate() {
            self.controllers[index].set_motor_speed(value);
        }
    }
}
