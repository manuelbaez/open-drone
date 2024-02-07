use nalgebra::Vector4;

use super::pid::PID;

pub struct RotationRateControllerInput {
    pub throttle: f32,
    pub desired_roll_rate: f32,
    pub desired_pitch_rate: f32,
    pub desired_yaw_rate: f32,
    pub measured_roll_rate: f32,
    pub measured_pitch_rate: f32,
    pub measured_yaw_rate: f32,
    pub iteration_time: f32,
}

pub struct RotationRateFlightController {
    roll_pid: PID,
    pitch_pid: PID,
    yaw_pid: PID,
    motor_min_power: f32,
    motor_max_power: f32,
}

impl RotationRateFlightController {
    pub fn new(motor_min_power: f32, motor_max_power: f32) -> Self {
        RotationRateFlightController {
            roll_pid: PID::new(0.0006, 0.003, 0.001),
            pitch_pid: PID::new(0.0006, 0.003, 0.001),
            yaw_pid: PID::new(0.0001, 0.001, 0.0),
            motor_min_power,
            motor_max_power,
        }
    }

    pub fn get_next_output(&mut self, input: RotationRateControllerInput) -> [f32; 4] {
        let roll_output = self.roll_pid.update(
            input.desired_roll_rate,
            input.measured_roll_rate,
            input.iteration_time,
        );
        let pitch_output = self.pitch_pid.update(
            input.desired_pitch_rate,
            input.measured_pitch_rate,
            input.iteration_time,
        );
        let yaw_output = self.yaw_pid.update(
            input.desired_yaw_rate,
            input.measured_yaw_rate,
            input.iteration_time,
        );

        let roll_motor_input = self.map_roll_to_motor_input(roll_output);
        let pitch_motor_input = self.map_pitch_to_motor_input(pitch_output);
        let yaw_motor_input = self.map_yaw_to_motor_input(yaw_output);

        let throtlle_motor_input = Vector4::new(
            input.throttle,
            input.throttle,
            input.throttle,
            input.throttle,
        );

        let motor_input =
            throtlle_motor_input + roll_motor_input + pitch_motor_input + yaw_motor_input;

        self.validate_and_map_motor_output(motor_input)
    }

    fn map_roll_to_motor_input(&self, roll_value: f32) -> Vector4<f32> {
        Vector4::new(roll_value, -roll_value, roll_value, -roll_value)
    }

    fn map_pitch_to_motor_input(&self, pitch_value: f32) -> Vector4<f32> {
        Vector4::new(pitch_value, pitch_value, -pitch_value, -pitch_value)
    }

    fn map_yaw_to_motor_input(&self, yaw_value: f32) -> Vector4<f32> {
        Vector4::new(-yaw_value, yaw_value, yaw_value, -yaw_value)
    }

    fn validate_and_map_motor_output(&self, motor_inputs: Vector4<f32>) -> [f32; 4] {
        let max_throttle_vector = Vector4::from([
            self.motor_max_power,
            self.motor_max_power,
            self.motor_max_power,
            self.motor_max_power,
        ]);

        let exccess = (motor_inputs - max_throttle_vector).map(|value| {
            if value > 0.0 {
                return value;
            } else {
                return 0.0;
            }
        });

        let capped_throttle = (motor_inputs - exccess).map(|value| {
            if value > self.motor_min_power {
                return value;
            } else {
                return self.motor_min_power;
            }
        });

        [
            capped_throttle[0],
            capped_throttle[1],
            capped_throttle[2],
            capped_throttle[3],
        ]
    }
}
