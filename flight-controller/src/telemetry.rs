use crate::util::vectors::{RotationVector2D, RotationVector3D};

#[derive(Default, Debug)]
pub struct TelemetryDataValues {
    pub loop_exec_time_us: u128,
    pub rate_controller_output: RotationVector3D,
    pub angle_controller_output: RotationVector2D,
    pub kalman_predicted_state: RotationVector2D,
    pub rotation_rate: RotationVector3D,
    pub accelerometer_rotation: RotationVector2D,
    pub motors_power: [f32; 4],
    pub throttle: f32,
}
