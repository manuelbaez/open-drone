use super::vectors::{AccelerationVector3D, RotationVector2D, RotationVector3D};

pub trait Accelerometer {
    fn get_acceleration_vector(&mut self) -> AccelerationVector3D;
    fn get_roll_pitch_angles(&mut self) -> RotationVector2D;
}

pub trait Gyroscope {
    fn get_rotation_rates(&mut self) -> RotationVector3D;
}
