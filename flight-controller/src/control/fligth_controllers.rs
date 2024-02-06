use nalgebra::Vector4;

use super::pid::PID;

pub struct RotationRateFlightController {
    roll_pid: PID,
    pitch_pid: PID,
    yaw_pid: PID,
    motor_min_power: f32,
}

impl RotationRateFlightController {
    fn new(motor_min_power: f32) -> Self {
        RotationRateFlightController {
            roll_pid: PID::new(1.0, 1.0, 1.0),
            pitch_pid: PID::new(1.0, 1.0, 1.0),
            yaw_pid: PID::new(1.0, 1.0, 1.0),
            motor_min_power,
        }
    }

    fn map_yaw_to_motor_input(yaw_value: f32) -> Vector4<f32> {
        Vector4::new(0.0, 0.0, 0.0, 0.0)
    }

    fn map_roll_to_motor_input(yaw_value: f32) -> Vector4<f32> {
        Vector4::new(0.0, 0.0, 0.0, 0.0)
    }
    
    fn map_pitch_to_motor_input(yaw_value: f32) -> Vector4<f32> {
        Vector4::new(0.0, 0.0, 0.0, 0.0)
    }
}
