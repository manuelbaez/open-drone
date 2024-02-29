#[derive(Debug, Clone, Copy, Default)]
#[repr(C, packed)]
pub struct ControllerInput {
    pub roll: i16,
    pub pitch: i16,
    pub yaw: i16,
    pub throttle: u8,
    pub kill_motors: bool,
    pub start: bool,
    pub calibrate_esc: bool,
    pub calibrate_sensors: bool,
}

#[derive(Debug, Clone, Copy, Default)]
#[repr(C, packed)]
pub struct PIDTuneConfig {
    pub proportional_multiplier: f32,
    pub integral_multiplier: f32,
    pub derivative_multiplier: f32,
    pub max_accumulated_error: f32,
}

#[derive(Debug, Clone, Copy, Default)]
#[repr(C, packed)]
pub struct PIDTuneInput {
    pub roll: PIDTuneConfig,
    pub pitch: PIDTuneConfig,
    pub yaw: PIDTuneConfig,
}
