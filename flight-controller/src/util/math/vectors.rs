use std::ops::{Add, AddAssign, Div, Sub};

use libm::{atan2f, sqrtf};

#[derive(Debug, Default, Clone, Copy)]
pub struct AccelerationVector3D {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl AccelerationVector3D {
    pub fn calculate_orientation_angles(&self) -> RotationVector2D {
        let roll = atan2f(self.y, sqrtf(self.x.powf(2.0) + self.z.powf(2.0)));
        let pitch = atan2f(self.x, sqrtf(self.y.powf(2.0) + self.z.powf(2.0)));

        RotationVector2D {
            roll: roll.to_degrees(),
            pitch: pitch.to_degrees(),
        }
    }
}

impl Sub<AccelerationVector3D> for AccelerationVector3D {
    type Output = AccelerationVector3D;

    fn sub(self, rhs: AccelerationVector3D) -> Self::Output {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl AddAssign<AccelerationVector3D> for AccelerationVector3D {
    fn add_assign(&mut self, rhs: AccelerationVector3D) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl Div<f32> for AccelerationVector3D {
    type Output = AccelerationVector3D;

    fn div(self, rhs: f32) -> Self::Output {
        Self {
            x: self.x / rhs,
            y: self.y / rhs,
            z: self.z / rhs,
        }
    }
}

#[derive(Debug, Default, Copy)]
pub struct RotationVector3D {
    pub pitch: f32,
    pub roll: f32,
    pub yaw: f32,
}

impl Add<RotationVector3D> for RotationVector3D {
    type Output = RotationVector3D;

    fn add(self, rhs: RotationVector3D) -> Self::Output {
        Self {
            pitch: self.pitch + rhs.pitch,
            roll: self.roll + rhs.roll,
            yaw: self.yaw + rhs.yaw,
        }
    }
}

impl Sub<RotationVector3D> for RotationVector3D {
    type Output = RotationVector3D;

    fn sub(self, rhs: RotationVector3D) -> Self::Output {
        Self {
            pitch: self.pitch - rhs.pitch,
            roll: self.roll - rhs.roll,
            yaw: self.yaw - rhs.yaw,
        }
    }
}

impl AddAssign<RotationVector3D> for RotationVector3D {
    fn add_assign(&mut self, rhs: RotationVector3D) {
        self.pitch += rhs.pitch;
        self.roll += rhs.roll;
        self.yaw += rhs.yaw;
    }
}

impl Div<f32> for RotationVector3D {
    type Output = RotationVector3D;

    fn div(self, rhs: f32) -> Self::Output {
        Self {
            pitch: self.pitch / rhs,
            roll: self.roll / rhs,
            yaw: self.yaw / rhs,
        }
    }
}

impl RotationVector3D {
    pub fn from(vector_2d: &RotationVector2D) -> Self {
        RotationVector3D {
            roll: vector_2d.roll.clone(),
            pitch: vector_2d.pitch.clone(),
            yaw: 0.0_f32,
        }
    }
}

impl Clone for RotationVector3D {
    fn clone(&self) -> Self {
        Self {
            roll: self.roll.clone(),
            pitch: self.pitch.clone(),
            yaw: self.yaw.clone(),
        }
    }
}

#[derive(Debug, Default)]
pub struct RotationVector2D {
    pub roll: f32,
    pub pitch: f32,
}

impl RotationVector2D {
    pub fn from(vector_3d: &RotationVector3D) -> Self {
        RotationVector2D {
            roll: vector_3d.roll.clone(),
            pitch: vector_3d.pitch.clone(),
        }
    }
}

impl Clone for RotationVector2D {
    fn clone(&self) -> Self {
        Self {
            roll: self.roll.clone(),
            pitch: self.pitch.clone(),
        }
    }
}
