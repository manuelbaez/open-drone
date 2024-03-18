use esp_idf_svc::hal::{peripheral::Peripheral, peripherals::Peripherals};

use super::motor_controller::{MotorConfig, MotorController};

pub struct QuadcopterMotorsStateManager {
    controllers: [MotorController; 4],
}

impl QuadcopterMotorsStateManager {
    pub fn new(peripherals: &mut Peripherals) -> Self {
        let controllers = [
            MotorController::new(MotorConfig {
                pin: unsafe { peripherals.pins.gpio13.clone_unchecked() },
                channel: unsafe { peripherals.ledc.channel0.clone_unchecked() },
                timer: unsafe { peripherals.ledc.timer0.clone_unchecked() },
            }),
            MotorController::new(MotorConfig {
                pin: unsafe { peripherals.pins.gpio12.clone_unchecked() },
                channel: unsafe { peripherals.ledc.channel1.clone_unchecked() },
                timer: unsafe { peripherals.ledc.timer0.clone_unchecked() },
            }),
            MotorController::new(MotorConfig {
                pin: unsafe { peripherals.pins.gpio14.clone_unchecked() },
                channel: unsafe { peripherals.ledc.channel2.clone_unchecked() },
                timer: unsafe { peripherals.ledc.timer0.clone_unchecked() },
            }),
            MotorController::new(MotorConfig {
                pin: unsafe { peripherals.pins.gpio27.clone_unchecked() },
                channel: unsafe { peripherals.ledc.channel3.clone_unchecked() },
                timer: unsafe { peripherals.ledc.timer0.clone_unchecked() },
            }),
        ];

        QuadcopterMotorsStateManager { controllers }
    }

    pub fn set_motor_power(&mut self, values: [f32; 4]) {
        self.controllers[0].set_motor_speed(values[0]);
        self.controllers[1].set_motor_speed(values[1]);
        self.controllers[2].set_motor_speed(values[2]);
        self.controllers[3].set_motor_speed(values[3]);
    }

    pub fn kill_motors(&mut self) {
        self.set_motor_power([0.0, 0.0, 0.0, 0.0]);
        log::info!("Killed motors");
    }
}
