use bitfield_struct::bitfield;

pub const DEFAULT_SLAVE_ADDR: u8 = 0x68;

pub struct MPURegisters;
impl MPURegisters {
    pub const CONFIG: u8 = 0x1A;
    pub const GYRO_CONFIG: u8 = 0x1B;
    pub const ACCEL_CONFIG: u8 = 0x1C;
    pub const ACCEL_MEASURE_START: u8 = 0x3B;
    pub const GYRO_MEASURE_START: u8 = 0x43;
    pub const POWER_MANAGEMENT: u8 = 0x6B;
}

pub struct MpuGyroSensitivityRanges {
    pub fs_sel: u8,
    pub range: u16,
    pub sensitvity: f32,
}

#[allow(dead_code)]
impl MpuGyroSensitivityRanges {
    //Gyro
    pub const GYRO_RANGE_250: MpuGyroSensitivityRanges = Self {
        fs_sel: 0x0,
        range: 250,
        sensitvity: 131.0,
    };
    pub const GYRO_RANGE_500: MpuGyroSensitivityRanges = Self {
        fs_sel: 0x1,
        range: 500,
        sensitvity: 65.5,
    };
    pub const GYRO_RANGE_1000: MpuGyroSensitivityRanges = Self {
        fs_sel: 0x2,
        range: 1000,
        sensitvity: 32.8,
    };
    pub const GYRO_RANGE_2000: MpuGyroSensitivityRanges = Self {
        fs_sel: 0x3,
        range: 2000,
        sensitvity: 16.4,
    };
}

pub struct MpuAccelSensitivityRanges {
    pub afs_sel: u8,
    pub range: u8,
    pub sensitvity: u16,
}

#[allow(dead_code)]
impl MpuAccelSensitivityRanges {
    //Accelerometer
    pub const ACCEL_RANGE_2G: MpuAccelSensitivityRanges = Self {
        afs_sel: 0x0,
        range: 2,
        sensitvity: 16384,
    };
    pub const ACCEL_RANGE_4G: MpuAccelSensitivityRanges = Self {
        afs_sel: 0x1,
        range: 4,
        sensitvity: 8192,
    };
    pub const ACCEL_RANGE_8G: MpuAccelSensitivityRanges = Self {
        afs_sel: 0x2,
        range: 8,
        sensitvity: 4096,
    };
    pub const ACCEL_RANGE_16G: MpuAccelSensitivityRanges = Self {
        afs_sel: 0x3,
        range: 16,
        sensitvity: 2048,
    };
}

#[repr(u8)]
#[allow(dead_code)]
#[derive(Clone)]
pub enum LowPassFrequencyValues {
    None = 0x0,
    Freq10Hz = 0x5,
    Freq21Hz = 0x4,
}

#[bitfield(u8)]
pub struct AccelGyroConfigRegister {
    #[bits(3)]
    pub pad: u8,
    #[bits(2)]
    pub fs_sel: u8,
    pub z_self_test: bool,
    pub y_self_test: bool,
    pub x_self_test: bool,
}

///See docs for register 107
#[bitfield(u8)]
pub struct MpuPowerManagementRegister {
    #[bits(3)]
    pub clock_sel: u8, // Set to zero for internal oscillator
    pub temp_sensor_disable: bool,
    pub padding_bit: bool, // Set to zero
    pub cycle: bool,
    pub sleep: bool,
    pub device_reset: bool,
}

#[derive(Default)]
#[repr(C, packed)]
pub struct Mpu6050AccelRegOut {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}
#[derive(Default)]
#[repr(C, packed)]
pub struct Mpu6050GyroRegOut {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}
