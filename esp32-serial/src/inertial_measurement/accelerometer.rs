use esp_idf_svc::hal::{delay::BLOCK, i2c::*};

const ADXL345_ADDR: u8 = 0x53;
const ADXL345_POWER_CTL: u8 = 0x2d;
// const ADXL345_TO_READ: u8 = 6;
const ADXL345_DATAX0: u8 = 0x32;
const ACCLEL_1G_VALUE: f32 = 256_f32;
const ACCELEROMETER_AXIS_G_GAIN_XYZ: [f32; 3] = [
    1.0_f32 / ACCLEL_1G_VALUE,
    1.0_f32 / ACCLEL_1G_VALUE,
    1.0_f32 / ACCLEL_1G_VALUE,
];

pub fn init_accelerometer(i2c_driver: &mut I2cDriver<'_>) {
    i2c_driver
        .write(ADXL345_ADDR, &[ADXL345_POWER_CTL, 0], BLOCK)
        .expect("Failed to set config parameters");

    i2c_driver
        .write(ADXL345_ADDR, &[ADXL345_POWER_CTL, 16], BLOCK)
        .expect("Failed to set config parameters");

    i2c_driver
        .write(ADXL345_ADDR, &[ADXL345_POWER_CTL, 8], BLOCK)
        .expect("Failed to set config parameters");

    log::info!("Accelerometer turned on");
}

pub fn get_accelerometer_values(i2c_driver: &mut I2cDriver<'_>) -> Vec<f32> {
    let mut buffer: [u8; 6] = [0; 6];

    i2c_driver
        .write(ADXL345_ADDR, &[ADXL345_DATAX0], BLOCK)
        .expect("Error writing");

    i2c_driver
        .read(ADXL345_ADDR, &mut buffer, BLOCK)
        .expect("Error reading");

    let x = (u16::from(buffer[1]) << 8 | u16::from(buffer[0])) as i16 as f32
        * ACCELEROMETER_AXIS_G_GAIN_XYZ[0];

    let y = ((u16::from(buffer[3]) << 8 | u16::from(buffer[2])) as i16) as f32
        * ACCELEROMETER_AXIS_G_GAIN_XYZ[1];

    let z = ((u16::from(buffer[5]) << 8 | u16::from(buffer[4])) as i16) as f32
        * ACCELEROMETER_AXIS_G_GAIN_XYZ[2];

    vec![x, y, z]
}
