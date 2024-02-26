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
