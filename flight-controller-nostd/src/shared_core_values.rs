use core::sync::atomic::{AtomicBool, AtomicI16, AtomicI32, AtomicU32, AtomicU8, Ordering};

#[cfg(feature = "wifi-tuning")]
use shared_definitions::controller::PIDTuneConfig;

use crate::util::math::vectors::RotationVector3D;

pub struct AtomicF32(AtomicU32);
impl AtomicF32 {
    pub const fn new(val: f32) -> Self {
        Self(AtomicU32::new(val.to_bits()))
    }
    #[allow(dead_code)]
    pub fn load(&self, order: Ordering) -> f32 {
        f32::from_bits(self.0.load(order))
    }
    #[allow(dead_code)]
    pub fn store(&self, val: f32, order: Ordering) {
        self.0.store(val.to_bits(), order)
    }
}

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
            .store((vector.roll * 100.0) as i16, Ordering::Relaxed);
        self.pitch
            .store((vector.pitch * 100.0) as i16, Ordering::Relaxed);
        self.yaw
            .store((vector.yaw * 100.0) as i16, Ordering::Relaxed);
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
    pub battery_voltage: AtomicF32,
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
            battery_voltage: AtomicF32::new(12.0),
        }
    }
}

#[cfg(feature = "wifi-tuning")]
pub struct AtomicPidTuning {
    pub proportional: AtomicF32,
    pub integral: AtomicF32,
    pub derivative: AtomicF32,
    pub max_integral: AtomicF32,
}

#[cfg(feature = "wifi-tuning")]
impl AtomicPidTuning {
    const fn default() -> Self {
        Self {
            proportional: AtomicF32::new(0.3),
            integral: AtomicF32::new(0.0),
            derivative: AtomicF32::new(0.07),
            max_integral: AtomicF32::new(20.0),
        }
    }

    pub fn map_to_pid_input(&self) -> PIDTuneConfig {
        PIDTuneConfig {
            proportional_multiplier: self.proportional.load(Ordering::Relaxed),
            integral_multiplier: self.integral.load(Ordering::Relaxed),
            derivative_multiplier: self.derivative.load(Ordering::Relaxed),
            max_accumulated_error: self.max_integral.load(Ordering::Relaxed),
        }
    }
}

#[cfg(feature = "wifi-tuning")]
pub struct AtomicPidTuningInput {
    pub roll: AtomicPidTuning,
    pub pitch: AtomicPidTuning,
    pub yaw: AtomicPidTuning,
}

#[cfg(feature = "wifi-tuning")]
impl AtomicPidTuningInput {
    pub const fn new() -> Self {
        Self {
            roll: AtomicPidTuning::default(),
            pitch: AtomicPidTuning::default(),
            yaw: AtomicPidTuning::default(),
        }
    }
}

pub static SHARED_CONTROLLER_INPUT: AtomicControllerInput = AtomicControllerInput::new();
pub static SHARED_TELEMETRY: AtomicTelemetry = AtomicTelemetry::new();
#[cfg(feature = "wifi-tuning")]
pub static SHARED_TUNING: AtomicPidTuningInput = AtomicPidTuningInput::new();
