use std::ops::{Add, AddAssign, Div, Sub};

#[derive(Debug, Default, Clone)]
pub struct AccelerationVector3D {
    pub x: f32,
    pub y: f32,
    pub z: f32,
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
#[derive(Debug, Default)]
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
    pub fn from(vector_3d: &RotationVector2D) -> Self {
        RotationVector3D {
            roll: vector_3d.roll.clone(),
            pitch: vector_3d.pitch.clone(),
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
