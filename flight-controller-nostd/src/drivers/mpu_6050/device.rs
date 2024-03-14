use super::registers::{
    AccelGyroConfigRegister, LowPassFrequencyValues, MPURegisters, Mpu6050AccelRegOut,
    Mpu6050GyroRegOut, MpuAccelSensitivityRanges, MpuGyroSensitivityRanges,
    MpuPowerManagementRegister, DEFAULT_SLAVE_ADDR,
};
use crate::{
    drivers::imu_sensors::{Accelerometer, CombinedGyroscopeAccelerometer, Gyroscope},
    util::math::vectors::{AccelerationVector3D, RotationVector3D},
};
use embassy_time::{Duration, Timer};
use esp_hal::{
    i2c::{self, I2C},
    prelude::*,
};

#[derive(Default)]
#[repr(C, packed)]
struct Mpu6050CombinedOutReg {
    accelerometer: Mpu6050AccelRegOut,
    temperature: i16,
    gyroscope: Mpu6050GyroRegOut,
}

pub struct MPU6050Sensor<'a, I>
where
    I: i2c::Instance,
{
    i2c_driver: &'a mut I2C<'a, I>,
    mpu_addr: u8,
    accel_sensitivity: MpuAccelSensitivityRanges,
    gyro_sensitivity: MpuGyroSensitivityRanges,
    gyro_drift_calibration: RotationVector3D,
    accelerometer_calibration: AccelerationVector3D,
    low_pass_filter_freq: LowPassFrequencyValues,
}

impl<'a, I> MPU6050Sensor<'a, I>
where
    I: i2c::Instance,
{
    pub fn new(
        i2c_driver: &'a mut I2C<'a, I>,
        gyro_drift_calibration: RotationVector3D,
        accelerometer_calibration: AccelerationVector3D,
    ) -> Self {
        MPU6050Sensor {
            i2c_driver,
            gyro_drift_calibration,
            accelerometer_calibration,
            mpu_addr: DEFAULT_SLAVE_ADDR,
            accel_sensitivity: MpuAccelSensitivityRanges::ACCEL_RANGE_2G,
            gyro_sensitivity: MpuGyroSensitivityRanges::GYRO_RANGE_500,
            low_pass_filter_freq: LowPassFrequencyValues::None,
        }
    }

    pub async fn init(&mut self) {
        Timer::after_millis(100).await;
        self.reset_device();
        Timer::after_millis(100).await;
        self.update_gyro_config_register();
        Timer::after_millis(100).await;
        self.update_accel_config_register();
        Timer::after_millis(100).await;
        self.set_power_management_register_default();
        Timer::after_millis(500).await;
        self.update_dlpf_filter_register();
        Timer::after_millis(100).await;
    }

    pub fn enable_low_pass_filter(&mut self, low_pass_freq: LowPassFrequencyValues) {
        esp_println::println!(
            "Set mpu low pass filter {:02x?}",
            low_pass_freq.clone() as u8
        );
        self.low_pass_filter_freq = low_pass_freq;
    }

    #[allow(dead_code)]
    pub fn set_gyro_sensitivity(&mut self, sensitivity: MpuGyroSensitivityRanges) {
        self.gyro_sensitivity = sensitivity;
        self.update_gyro_config_register();
    }

    #[allow(dead_code)]
    pub fn set_accel_sensitivity(&mut self, sensitivity: MpuGyroSensitivityRanges) {
        self.gyro_sensitivity = sensitivity;
        self.update_accel_config_register();
    }

    fn reset_device(&mut self) {
        let register_value = MpuPowerManagementRegister::new()
            .with_device_reset(true)
            .with_sleep(true)
            .into_bits();
        self.i2c_driver
            .write(self.mpu_addr, &[MPURegisters::GYRO_CONFIG, register_value])
            .ok();
    }

    fn update_gyro_config_register(&mut self) {
        let register_bitmap = AccelGyroConfigRegister::new()
            .with_fs_sel(self.gyro_sensitivity.fs_sel)
            .into_bits();
        self.i2c_driver
            .write(self.mpu_addr, &[MPURegisters::GYRO_CONFIG, register_bitmap])
            .unwrap();
    }

    fn update_accel_config_register(&mut self) {
        let register_bitmap = AccelGyroConfigRegister::new()
            .with_fs_sel(self.accel_sensitivity.afs_sel)
            .into_bits();
        self.i2c_driver
            .write(
                self.mpu_addr,
                &[MPURegisters::ACCEL_CONFIG, register_bitmap],
            )
            .unwrap();
    }

    pub fn update_dlpf_filter_register(&mut self) {
        let reg_value = match self.low_pass_filter_freq {
            LowPassFrequencyValues::Freq10Hz => 0x5_u8,
            LowPassFrequencyValues::Freq21Hz => 0x4_u8,
            LowPassFrequencyValues::None => 0x0_u8,
        };
        let buffer = [MPURegisters::CONFIG, reg_value];
        self.i2c_driver.write(self.mpu_addr, &buffer).unwrap(); //Set to 10hz
        esp_println::println!("Set mpu low pass filter {:02x?}", buffer);
    }

    fn get_accel_data(&mut self) -> Mpu6050AccelRegOut {
        let mut accel_output = Mpu6050AccelRegOut::default();
        let mut buf: [u8; 6] = [0; 6];

        self.i2c_driver
            .write_read(
                self.mpu_addr,
                &[MPURegisters::ACCEL_MEASURE_START],
                &mut buf,
            )
            .ok();

        accel_output.x = (buf[0] as i16) << 8 | buf[1] as i16;
        accel_output.y = (buf[2] as i16) << 8 | buf[3] as i16;
        accel_output.z = (buf[4] as i16) << 8 | buf[5] as i16;

        accel_output
    }

    fn get_gyro_data(&mut self) -> Mpu6050GyroRegOut {
        let mut gyro_output = Mpu6050GyroRegOut::default();
        let mut buf: [u8; 6] = [0; 6];

        self.i2c_driver
            .write_read(self.mpu_addr, &[MPURegisters::GYRO_MEASURE_START], &mut buf)
            .ok();

        gyro_output.x = (buf[0] as i16) << 8 | buf[1] as i16;
        gyro_output.y = (buf[2] as i16) << 8 | buf[3] as i16;
        gyro_output.z = (buf[4] as i16) << 8 | buf[5] as i16;

        gyro_output
    }

    fn map_gyro_out_to_vector(&self, gyro_data: Mpu6050GyroRegOut) -> RotationVector3D {
        let roll = gyro_data.x as f32 / self.gyro_sensitivity.sensitvity; // Roll is rotation along the y axis
        let pitch = -gyro_data.y as f32 / self.gyro_sensitivity.sensitvity; // Pitch is rotation along the x axis
        let yaw = gyro_data.z as f32 / self.gyro_sensitivity.sensitvity; // Yaw is rotation along the z axis
                                                                         // See right hand rule to understand how positive and negative rotation rates are measured.
        RotationVector3D { pitch, roll, yaw }
    }

    fn map_accel_out_to_vector(&self, accel_data: Mpu6050AccelRegOut) -> AccelerationVector3D {
        let x = accel_data.x as f32 / self.accel_sensitivity.sensitvity as f32;
        let y = accel_data.y as f32 / self.accel_sensitivity.sensitvity as f32;
        let z = accel_data.z as f32 / self.accel_sensitivity.sensitvity as f32;

        AccelerationVector3D { x, y, z }
    }

    fn set_power_management_register_default(&mut self) {
        let register_value = MpuPowerManagementRegister::new().into_bits();
        self.i2c_driver
            .write(
                self.mpu_addr,
                &[MPURegisters::POWER_MANAGEMENT, register_value],
            )
            .ok();
    }
}

impl<'a, I> Accelerometer for MPU6050Sensor<'a, I>
where
    I: i2c::Instance,
{
    fn get_acceleration_vector_uncalibrated(&mut self) -> AccelerationVector3D {
        let acc_values = self.get_accel_data();
        self.map_accel_out_to_vector(acc_values)
    }

    fn get_acceleration_vector(&mut self) -> AccelerationVector3D {
        self.get_acceleration_vector_uncalibrated() - self.accelerometer_calibration.clone()
    }

    fn set_deviation_calibration(&mut self, claibration: AccelerationVector3D) {
        self.accelerometer_calibration = claibration;
    }
}

impl<'a, I> Gyroscope for MPU6050Sensor<'a, I>
where
    I: i2c::Instance,
{
    fn get_rotation_rates_uncalibrated(&mut self) -> RotationVector3D {
        let gyro_data = self.get_gyro_data();
        self.map_gyro_out_to_vector(gyro_data)
    }

    fn get_rotation_rates(&mut self) -> RotationVector3D {
        self.get_rotation_rates_uncalibrated() - self.gyro_drift_calibration.clone()
    }

    fn set_drift_calibration(&mut self, claibration: RotationVector3D) {
        self.gyro_drift_calibration = claibration;
    }
}

impl<'a, I> CombinedGyroscopeAccelerometer for MPU6050Sensor<'a, I>
where
    I: i2c::Instance,
{
    fn get_combined_gyro_accel_output(&mut self) -> (RotationVector3D, AccelerationVector3D) {
        let mut gyro_output = Mpu6050GyroRegOut::default();
        let mut accel_output = Mpu6050AccelRegOut::default();

        let mut buf: [u8; 14] = [0; 14];
        self.i2c_driver
            .write_read(
                self.mpu_addr,
                &[MPURegisters::ACCEL_MEASURE_START],
                &mut buf,
            )
            .ok();

        accel_output.x = (buf[0] as i16) << 8 | buf[1] as i16;
        accel_output.y = (buf[2] as i16) << 8 | buf[3] as i16;
        accel_output.z = (buf[4] as i16) << 8 | buf[5] as i16;
        let accel_vector =
            self.map_accel_out_to_vector(accel_output) - self.accelerometer_calibration.clone();

        gyro_output.x = (buf[8] as i16) << 8 | buf[9] as i16;
        gyro_output.y = (buf[10] as i16) << 8 | buf[11] as i16;
        gyro_output.z = (buf[12] as i16) << 8 | buf[13] as i16;
        let gyro_vector =
            self.map_gyro_out_to_vector(gyro_output) - self.gyro_drift_calibration.clone();

        (gyro_vector, accel_vector)
    }
}
