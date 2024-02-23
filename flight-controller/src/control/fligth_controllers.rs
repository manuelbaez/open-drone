use nalgebra::Vector4;

use super::{
    inertial_measurement::vectors::{RotationVector2D, RotationVector3D},
    kalman_filter::KalmanFilter,
    pid::PID,
};

pub struct RotationRateControllerInput {
    pub desired_rotation_rate: RotationVector3D,
    pub measured_rotation_rate: RotationVector3D,
    pub iteration_time: f32,
}

pub struct RotationRateFlightController {
    roll_pid: PID,
    pitch_pid: PID,
    yaw_pid: PID,
}

impl RotationRateFlightController {
    pub fn new() -> Self {
        RotationRateFlightController {
            roll_pid: PID::new(0.1, 0.0, 0.01),
            pitch_pid: PID::new(0.1, 0.0, 0.01),
            yaw_pid: PID::new(0.1, 0.0, 0.0),
        }
    }

    ///Updates the flight pids and returns their current state, this should
    ///ideally return something in the range of +-100 as that's the maximum
    ///accepted by the motors managers.
    pub fn get_next_output(&mut self, input: RotationRateControllerInput) -> RotationVector3D {
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

        RotationVector3D {
            pitch: pitch_output,
            roll: roll_output,
            yaw: yaw_output,
        }
    }

    pub fn reset(&mut self) {
        self.pitch_pid.reset();
        self.roll_pid.reset();
        self.yaw_pid.reset();
    }
}

#[derive(Debug, Clone, Default)]
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

    pub fn get_current_kalman_predicted_state(&self) -> RotationVector2D {
        RotationVector2D {
            roll: self.roll_kalman_filter.get_current_state(),
            pitch: self.pitch_kalman_filter.get_current_state(),
        }
    }
}
