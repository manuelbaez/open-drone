use std::sync::atomic::{AtomicBool, AtomicI16, AtomicI32, AtomicU8, Ordering};

use crate::util::vectors::RotationVector3D;
pub struct AtomicControllerInput {
    pub roll: AtomicI16,
    pub pitch: AtomicI16,
    pub yaw: AtomicI16,
    pub throttle: AtomicU8,
    pub kill_motors: AtomicBool,
    pub start: AtomicBool,
    pub calibrate_esc: AtomicBool,
    pub calibrate_sensors: AtomicBool,
}

impl AtomicControllerInput {
    pub const fn new() -> Self {
        AtomicControllerInput {
            roll: AtomicI16::new(0),
            pitch: AtomicI16::new(0),
            yaw: AtomicI16::new(0),
            throttle: AtomicU8::new(0),
            kill_motors: AtomicBool::new(false),
            start: AtomicBool::new(false),
            calibrate_esc: AtomicBool::new(false),
            calibrate_sensors: AtomicBool::new(false),
        }
    }
}

pub struct AtomicRotationVector3D {
    roll: AtomicI16,
    pitch: AtomicI16,
    yaw: AtomicI16,
}

impl AtomicRotationVector3D {
    pub const fn new() -> Self {
        Self {
            roll: AtomicI16::new(0),
            pitch: AtomicI16::new(0),
            yaw: AtomicI16::new(0),
        }
    }
    pub fn store(&self, vector: RotationVector3D) {
        self.roll
            .store((vector.roll * 100.0) as i16, Ordering::Release);
        self.pitch
            .store((vector.pitch * 100.0) as i16, Ordering::Release);
        self.yaw
            .store((vector.yaw * 100.0) as i16, Ordering::Release);
    }
    pub fn read(&self) -> RotationVector3D {
        RotationVector3D {
            roll: self.roll.load(Ordering::Relaxed) as f32 / 100.0,
            pitch: self.pitch.load(Ordering::Relaxed) as f32 / 100.0,
            yaw: self.yaw.load(Ordering::Relaxed) as f32 / 100.0,
        }
    }
}

pub struct AtomicTelemetry {
    pub loop_exec_time_us: AtomicI32,
    pub rotation_rate: AtomicRotationVector3D,
    pub motor_1_power: AtomicU8,
    pub motor_2_power: AtomicU8,
    pub motor_3_power: AtomicU8,
    pub motor_4_power: AtomicU8,
    pub throttle: AtomicU8,
}

impl AtomicTelemetry {
    pub const fn new() -> Self {
        AtomicTelemetry {
            loop_exec_time_us: AtomicI32::new(0),
            rotation_rate: AtomicRotationVector3D::new(),
            motor_1_power: AtomicU8::new(0),
            motor_2_power: AtomicU8::new(0),
            motor_3_power: AtomicU8::new(0),
            motor_4_power: AtomicU8::new(0),
            throttle: AtomicU8::new(0),
        }
    }
}

pub static INPUT_SHARED: AtomicControllerInput = AtomicControllerInput::new();
pub static TELEMETRY_SHARED: AtomicTelemetry = AtomicTelemetry::new();
