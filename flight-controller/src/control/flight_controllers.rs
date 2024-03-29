use shared_definitions::controller::PIDTuneInput;

use crate::util::math::vectors::{RotationVector2D, RotationVector3D};

use super::{kalman_filter::KalmanFilter, pid::PID};

pub struct RotationRateControllerInput {
    pub desired_rotation_rate: RotationVector3D,
    pub measured_rotation_rate: RotationVector3D,
    pub iteration_time: f32,
}

#[derive(Clone)]
pub struct RotationRateFlightController {
    roll_pid: PID,
    pitch_pid: PID,
    yaw_pid: PID,
}

impl RotationRateFlightController {
    pub fn new() -> Self {
        RotationRateFlightController {
            roll_pid: PID::new(0.3, 0.0, 0.07, 20.0),
            pitch_pid: PID::new(0.3, 0.0, 0.07, 20.0),
            yaw_pid: PID::new(0.3, 0.0, 0.0, 20.0),
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
    
    #[allow(dead_code)]
    pub fn set_pid_tune(&mut self, tune_values: PIDTuneInput) {
        self.roll_pid.set_tune_values(tune_values.roll);
        self.pitch_pid.set_tune_values(tune_values.pitch);
        self.yaw_pid.set_tune_values(tune_values.yaw);
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
    output_gain: f32,
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
            output_gain: 2.0,
            roll_kalman_filter,
            pitch_kalman_filter,
            max_rotation_rate,
        }
    }

    pub fn get_next_output(&mut self, input: AngleModeControllerInput) -> RotationVector2D {
        let estimated_roll = self.roll_kalman_filter.get_next_state_prediction(
            input.measured_rotation_rate.roll,
            input.measured_rotation.roll,
            input.iteration_time,
        );

        let estimated_pitch = self.pitch_kalman_filter.get_next_state_prediction(
            input.measured_rotation_rate.pitch,
            input.measured_rotation.pitch,
            input.iteration_time,
        );

        let mut roll_output = self.output_gain * (input.desired_rotation.roll - estimated_roll);

        let mut pitch_output = self.output_gain * (input.desired_rotation.pitch - estimated_pitch);

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

    #[allow(dead_code)]
    pub fn get_current_kalman_predicted_state(&self) -> RotationVector2D {
        RotationVector2D {
            roll: self.roll_kalman_filter.get_current_state_prediction(),
            pitch: self.pitch_kalman_filter.get_current_state_prediction(),
        }
    }
}
