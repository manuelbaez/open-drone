pub struct AccelerationVector3D {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Debug)]
pub struct RotationVector3D {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
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
