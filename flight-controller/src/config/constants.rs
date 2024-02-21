use crate::communication_interfaces::ControllerTypes;

// Gyroscope Drift Calibrations
pub const GYRO_PITCH_CALIBRATION_DEG: f32 = -1.9987461681754335;
pub const GYRO_ROLL_CALIBRATION_DEG: f32 = -0.03329770963243634;
pub const GYRO_YAW_CALIBRATION_DEG: f32 = -0.06639503742574737;

//Accelerometer Calibration
pub const ACCEL_X_DEVIATION: f32 = 0.033;
pub const ACCEL_Y_DEVIATION: f32 = -0.035;
pub const ACCEL_Z_DEVIATION: f32 = 0.034;

// For kalmal filters purposes
pub const GYRO_DRIFT_DEG: f32 = 3.0_f32;
pub const ACCEL_UNCERTAINTY_DEG: f32 = 3.0_f32;

//Drone limits
pub const MAX_ROTATION_RATE: f32 = 75.0_f32;
pub const MIN_POWER: f32 = 15.0_f32;
pub const MAX_POWER: f32 = 100.0_f32; // for the stabilizer
pub const MAX_THROTTLE: f32 = 60.0_f32; // For the controller
pub const MAX_INCLINATION: f32 = 45.0_f32;

//Motors Controller based on pwm frequency
pub const MIN_MOTOR_DUTY: u32 = 8192;
pub const MAX_MOTOR_DUTY: u32 = 16000;
pub const ESC_PWM_FREQUENCY_HZ: u32 = 4000;

// Wifi controller config
pub const PAIRING_BSSID_ADDRESS: [u8; 6] = [0x50, 0xba, 0x61, 0x2a, 0x4a, 0x5e];
pub const TRANSMITTER_ADDRESS: [u8; 6] = [0xf2, 0xda, 0xd7, 0x60, 0x7e, 0xa9];

pub const CONTROLLER_TYPE: ControllerTypes = ControllerTypes::Wifi;
