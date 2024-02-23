use std::{thread::sleep, time::Duration};

use super::vectors::{AccelerationVector3D, RotationVector2D, RotationVector3D};

pub trait Accelerometer {
    fn get_acceleration_vector(&mut self) -> AccelerationVector3D;
    fn get_roll_pitch_angles(&mut self) -> RotationVector2D;
}

pub trait Gyroscope {
    fn get_rotation_rates_uncalibrated(&mut self) -> RotationVector3D;
    fn get_rotation_rates(&mut self) -> RotationVector3D;
    fn calculate_drift_average(&mut self) -> RotationVector3D {
        let mut rotation_rate_accumulator = RotationVector3D::default();
        let mut count = 0.0_f32;
        while count < 1000.0 {
            let rotation_rate = self.get_rotation_rates_uncalibrated();
            rotation_rate_accumulator += rotation_rate;
            count += 1.0;
            sleep(Duration::from_millis(2));
        }
        rotation_rate_accumulator / count
    }

    fn set_drift_calibration(&mut self, claibration: RotationVector3D);
}
