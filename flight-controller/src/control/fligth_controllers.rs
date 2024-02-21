use nalgebra::Vector4;

use super::{
    inertial_measurement::vectors::{RotationVector2D, RotationVector3D},
    kalman_filter::KalmanFilter,
    pid::PID,
};

pub struct RotationRateControllerInput {
    pub throttle: f32,
    pub desired_rotation_rate: RotationVector3D,
    pub measured_rotation_rate: RotationVector3D,
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
            roll_pid: PID::new(0.03, 0.05, 0.005),
            pitch_pid: PID::new(0.03, 0.05, 0.005),
            yaw_pid: PID::new(0.01, 0.001, 0.0),
            motor_min_power,
            motor_max_power,
        }
    }

    pub fn get_next_output(&mut self, input: RotationRateControllerInput) -> [f32; 4] {
        let roll_output = self.roll_pid.get_update(
            input.desired_rotation_rate.roll,
            input.measured_rotation_rate.roll,
            input.iteration_time,
        );
        let pitch_output = self.pitch_pid.get_update(
            input.desired_rotation_rate.pitch,
            input.measured_rotation_rate.pitch,
            input.iteration_time,
        );
        let yaw_output = self.yaw_pid.get_update(
            input.desired_rotation_rate.yaw,
            input.measured_rotation_rate.yaw,
            input.iteration_time,
        );

        let roll_motor_input = self.map_roll_to_motor_input(roll_output);
        let pitch_motor_input = self.map_pitch_to_motor_input(pitch_output);
        let yaw_motor_input = self.map_yaw_to_motor_input(yaw_output);

        let throtlle_motor_input = Vector4::from([input.throttle; 4]);

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
        let max_throttle_vector = Vector4::from([self.motor_max_power; 4]);

        let excess = (motor_inputs - max_throttle_vector).map(|value| {
            if value > 0.0 {
                return value;
            } else {
                return 0.0;
            }
        });

        let capped_throttle = (motor_inputs - excess).map(|value| {
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

    pub fn reset(&mut self) {
        self.pitch_pid.reset();
        self.roll_pid.reset();
        self.yaw_pid.reset();
    }
}

pub struct AngleModeControllerInput {
    pub desired_rotation: RotationVector2D,
    pub measured_rotation_rate: RotationVector2D,
    pub measured_rotation: RotationVector2D,
    pub iteration_time: f32,
}

pub struct AngleModeFlightController {
    roll_pid: PID,
    pitch_pid: PID,
    roll_kalman_filter: KalmanFilter,
    pitch_kalman_filter: KalmanFilter,
    max_rotation_rate: f32,
}

impl AngleModeFlightController {
    pub fn new(
        max_rotation_rate: f32,
        gyro_drift_deg_sec: f32,
        accelerometer_uncertainty_deg: f32,
    ) -> Self {
        let pitch_kalman_filter: KalmanFilter =
            KalmanFilter::new(gyro_drift_deg_sec, accelerometer_uncertainty_deg);
        let roll_kalman_filter: KalmanFilter =
            KalmanFilter::new(gyro_drift_deg_sec, accelerometer_uncertainty_deg);

        AngleModeFlightController {
            roll_pid: PID::new(2.0, 0.0, 0.0),
            pitch_pid: PID::new(2.0, 0.0, 0.0),
            roll_kalman_filter,
            pitch_kalman_filter,
            max_rotation_rate,
        }
    }

    pub fn get_next_output(&mut self, input: AngleModeControllerInput) -> RotationVector2D {
        let estimated_roll = self.roll_kalman_filter.update_next_prediction(
            input.measured_rotation_rate.roll,
            input.measured_rotation.roll,
            input.iteration_time,
        );

        let estimated_pitch = self.pitch_kalman_filter.update_next_prediction(
            input.measured_rotation_rate.pitch,
            input.measured_rotation.pitch,
            input.iteration_time,
        );

        let mut roll_output = self.roll_pid.get_update(
            input.desired_rotation.roll,
            estimated_roll,
            input.iteration_time,
        );
        let mut pitch_output = self.pitch_pid.get_update(
            input.desired_rotation.pitch,
            estimated_pitch,
            input.iteration_time,
        );

        if roll_output > self.max_rotation_rate {
            roll_output = self.max_rotation_rate;
        }

        if roll_output < (-self.max_rotation_rate) {
            roll_output = -self.max_rotation_rate;
        }

        if pitch_output > self.max_rotation_rate {
            pitch_output = self.max_rotation_rate;
        }

        if pitch_output < (-self.max_rotation_rate) {
            pitch_output = -self.max_rotation_rate;
        }

        RotationVector2D {
            pitch: pitch_output,
            roll: roll_output,
        }
    }
}
