use crate::communication_interfaces::i2c_adapter::I2CAdapter;

const ADXL345_ADDR: u8 = 0x53;
const ADXL345_POWER_CTL: u8 = 0x2d;
// const ADXL345_TO_READ: u8 = 6;
const ADXL345_DATAX0: u8 = 0x32;
const ACCLEL_1G_VALUE: f32 = 256_f32;
const ACCELEROMETER_AXIS_G_GAIN_XYZ: [f32; 3] = [
    9.81_f32 / ACCLEL_1G_VALUE,
    9.81_f32 / ACCLEL_1G_VALUE,
    9.81_f32 / ACCLEL_1G_VALUE,
];

pub fn init_accelerometer(i2c_adapter: &mut impl I2CAdapter) {
    i2c_adapter
        .write_to_device(ADXL345_ADDR, &[ADXL345_POWER_CTL, 0])
        .expect("Failed to set config parameters");

    i2c_adapter
        .write_to_device(ADXL345_ADDR, &[ADXL345_POWER_CTL, 16])
        .expect("Failed to set config parameters");

    i2c_adapter
        .write_to_device(ADXL345_ADDR, &[ADXL345_POWER_CTL, 8])
        .expect("Failed to set config parameters");

    log::info!("Accelerometer turned on");
}

pub fn get_accelerometer_xyz_values(i2c_adapter: &mut impl I2CAdapter) -> [f32; 3] {
    i2c_adapter
        .write_to_device(ADXL345_ADDR, &[ADXL345_DATAX0])
        .expect("Error requesting read");

    let buffer: [u8; 6] = i2c_adapter
        .read_from_device(ADXL345_ADDR)
        .expect("Error reading");

    let x = (u16::from(buffer[1]) << 8 | u16::from(buffer[0])) as i16 as f32
        * ACCELEROMETER_AXIS_G_GAIN_XYZ[0];

    let y = ((u16::from(buffer[3]) << 8 | u16::from(buffer[2])) as i16) as f32
        * ACCELEROMETER_AXIS_G_GAIN_XYZ[1];

    let z = ((u16::from(buffer[5]) << 8 | u16::from(buffer[4])) as i16) as f32
        * ACCELEROMETER_AXIS_G_GAIN_XYZ[2];

    [x, y, z]
}
