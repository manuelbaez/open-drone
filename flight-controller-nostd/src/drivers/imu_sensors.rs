use crate::util::math::vectors::{AccelerationVector3D, RotationVector3D};

pub trait Accelerometer {
    fn get_acceleration_vector_uncalibrated(&mut self) -> AccelerationVector3D;
    fn get_acceleration_vector(&mut self) -> AccelerationVector3D;
    ///To run this the accelerometer must be in a completely horizontal surface
    fn calculate_deviation_average(&mut self) -> AccelerationVector3D {
        let mut acceleration_accumulator = AccelerationVector3D::default();
        let mut count = 0.0_f32;
        while count < 4000.0 {
            let acceleration = self.get_acceleration_vector_uncalibrated();
            acceleration_accumulator += acceleration;
            count += 1.0;
        }
        let mut result = acceleration_accumulator / count;
        result.z = 0.0;
        result
    }

    fn set_deviation_calibration(&mut self, claibration: AccelerationVector3D);
}

pub trait Gyroscope {
    fn get_rotation_rates_uncalibrated(&mut self) -> RotationVector3D;
    fn get_rotation_rates(&mut self) -> RotationVector3D;
    ///To run this the gyro must be completely still
    fn calculate_drift_average(&mut self) -> RotationVector3D {
        let mut rotation_rate_accumulator = RotationVector3D::default();
        let mut count = 0.0_f32;
        while count < 4000.0 {
            let rotation_rate = self.get_rotation_rates_uncalibrated();
            rotation_rate_accumulator += rotation_rate;
            count += 1.0;
            // sleep(Duration::from_millis(2));
        }
        rotation_rate_accumulator / count
    }

    fn set_drift_calibration(&mut self, claibration: RotationVector3D);
}

pub trait CombinedGyroscopeAccelerometer {
    fn get_combined_gyro_accel_output(&mut self) -> (RotationVector3D, AccelerationVector3D);
}
