use esp32_hal::{
    gpio::{GpioPin, GpioProperties, Output, OutputPin, PushPull},
    ledc::{timer::Timer, HighSpeed, LEDC},
    peripheral::Peripheral,
};

use super::motor_controller::MotorController;

pub struct QuadcopterMotorsStateManager<
    'a,
    O1,
    O2,
    O3,
    O4,
    const MOTOR_1_PIN: u8,
    const MOTOR_2_PIN: u8,
    const MOTOR_3_PIN: u8,
    const MOTOR_4_PIN: u8,
> where
    O1: OutputPin,
    O2: OutputPin,
    O3: OutputPin,
    O4: OutputPin,
{
    motor_1_controller: MotorController<'a, O1, MOTOR_1_PIN>,
    motor_2_controller: MotorController<'a, O2, MOTOR_2_PIN>,
    motor_3_controller: MotorController<'a, O3, MOTOR_3_PIN>,
    motor_4_controller: MotorController<'a, O4, MOTOR_4_PIN>,
}

impl<
        'a,
        O1,
        O2,
        O3,
        O4,
        const MOTOR_1_PIN: u8,
        const MOTOR_2_PIN: u8,
        const MOTOR_3_PIN: u8,
        const MOTOR_4_PIN: u8,
    >
    QuadcopterMotorsStateManager<
        'a,
        O1,
        O2,
        O3,
        O4,
        MOTOR_1_PIN,
        MOTOR_2_PIN,
        MOTOR_3_PIN,
        MOTOR_4_PIN,
    >
where
    O1: OutputPin,
    O2: OutputPin,
    O3: OutputPin,
    O4: OutputPin,
    GpioPin<Output<PushPull>, MOTOR_1_PIN>: GpioProperties + OutputPin + Peripheral<P = O1>,
    GpioPin<Output<PushPull>, MOTOR_2_PIN>: GpioProperties + OutputPin + Peripheral<P = O2>,
    GpioPin<Output<PushPull>, MOTOR_3_PIN>: GpioProperties + OutputPin + Peripheral<P = O3>,
    GpioPin<Output<PushPull>, MOTOR_4_PIN>: GpioProperties + OutputPin + Peripheral<P = O4>,
{
    pub fn new_with_pins(
        ledc_driver: &'a LEDC<'a>,
        timer: &'a Timer<'a, HighSpeed>,
        motor_1_pin: GpioPin<Output<PushPull>, MOTOR_1_PIN>,
        motor_2_pin: GpioPin<Output<PushPull>, MOTOR_2_PIN>,
        motor_3_pin: GpioPin<Output<PushPull>, MOTOR_3_PIN>,
        motor_4_pin: GpioPin<Output<PushPull>, MOTOR_4_PIN>,
    ) -> Self {
        let motor_1_controller = MotorController::new(ledc_driver, timer, motor_1_pin);
        let motor_2_controller = MotorController::new(ledc_driver, timer, motor_2_pin);
        let motor_3_controller = MotorController::new(ledc_driver, timer, motor_3_pin);
        let motor_4_controller = MotorController::new(ledc_driver, timer, motor_4_pin);

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
