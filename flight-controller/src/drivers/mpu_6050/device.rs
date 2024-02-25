use embedded_hal::i2c::I2c;
use esp_idf_svc::hal::{delay::FreeRtos, i2c::I2cError};
use libm::{atan2f, sqrtf};

use crate::{
    drivers::imu_sensors::{Accelerometer, CombinedGyroscopeAccelerometer, Gyroscope},
    util::vectors::{AccelerationVector3D, RotationVector2D, RotationVector3D},
    I2cGenericDriver,
};

use super::registers::{
    AccelGyroConfigRegister, LowPassFrequencyValues, MPURegisters, Mpu6050AccelRegOut,
    Mpu6050GyroRegOut, MpuAccelSensitivityRanges, MpuGyroSensitivityRanges,
    MpuPowerManagementRegister, DEFAULT_SLAVE_ADDR,
};

#[derive(Default)]
#[repr(C, packed)]
struct Mpu6050CombinedOutReg {
    accelerometer: Mpu6050AccelRegOut,
    temperature: i16,
    gyroscope: Mpu6050GyroRegOut,
}

pub struct MPU6050Sensor<I>
where
    I: I2c<Error = I2cError>,
{
    i2c_driver: I,
    mpu_addr: u8,
    accel_sensitivity: MpuAccelSensitivityRanges,
    gyro_sensitivity: MpuGyroSensitivityRanges,
    gyro_drift_calibration: RotationVector3D,
    accelerometer_calibration: AccelerationVector3D,
}

impl<I> MPU6050Sensor<I>
where
    I: I2c<Error = I2cError>,
{
    pub fn new(
        i2c_driver: I,
        gyro_drift_calibration: RotationVector3D,
        accelerometer_calibration: AccelerationVector3D,
    ) -> Self {
        FreeRtos::delay_ms(100);
        MPU6050Sensor {
            i2c_driver,
            gyro_drift_calibration,
            accelerometer_calibration,
            mpu_addr: DEFAULT_SLAVE_ADDR,
            accel_sensitivity: MpuAccelSensitivityRanges::ACCEL_RANGE_2G,
            gyro_sensitivity: MpuGyroSensitivityRanges::GYRO_RANGE_500,
        }
    }

    pub fn init(&mut self) {
        FreeRtos::delay_ms(250);
        self.update_gyro_config_register();
        FreeRtos::delay_ms(5);
        self.update_accel_config_register();
        FreeRtos::delay_ms(5);
        self.set_power_management_register_default();
        FreeRtos::delay_ms(250);
    }

    pub fn enable_low_pass_filter(&mut self, low_pass_freq: LowPassFrequencyValues) {
        self.i2c_driver
            .write(self.mpu_addr, &[MPURegisters::CONFIG, low_pass_freq as u8])
            .unwrap(); //Set to 10hz
        FreeRtos::delay_ms(100);
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

    fn get_accel_data(&mut self) -> Mpu6050AccelRegOut {
        let mut accel_output = Mpu6050AccelRegOut::default();
        let mut buf: [u8; 6] = [0; 6];

        self.i2c_driver
            .write_read(
                self.mpu_addr,
                &[MPURegisters::ACCEL_MEASURE_START],
                &mut buf,
            )
            .unwrap();

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
            .unwrap();

        gyro_output.x = (buf[0] as i16) << 8 | buf[1] as i16;
        gyro_output.y = (buf[2] as i16) << 8 | buf[3] as i16;
        gyro_output.z = (buf[4] as i16) << 8 | buf[5] as i16;

        gyro_output
    }

    fn map_gyro_out_to_vector(&self, gyro_data: Mpu6050GyroRegOut) -> RotationVector3D {
        let pitch = gyro_data.x as f32 / self.gyro_sensitivity.sensitvity; // Pitch is rotation along the x axis
        let roll = gyro_data.y as f32 / self.gyro_sensitivity.sensitvity; // Roll is rotation along the y axis
        let yaw = gyro_data.z as f32 / self.gyro_sensitivity.sensitvity; // Yaw is rotation along the z axis
                                                                         // See right hand rule to understand how positive and negative rotation rates are measured.
        RotationVector3D { pitch, roll, yaw }
    }

    fn map_accel_out_to_vector(&self, accel_data: Mpu6050AccelRegOut) -> AccelerationVector3D {
        let x = (accel_data.x as f32 / self.accel_sensitivity.sensitvity as f32)
            - self.accelerometer_calibration.x;
        let y = (accel_data.y as f32 / self.accel_sensitivity.sensitvity as f32)
            - self.accelerometer_calibration.y;
        let z = (accel_data.z as f32 / self.accel_sensitivity.sensitvity as f32)
            - self.accelerometer_calibration.z;

        AccelerationVector3D { x, y, z }
    }

    fn set_power_management_register_default(&mut self) {
        let register_value = MpuPowerManagementRegister::new().into_bits();
        self.i2c_driver
            .write(
                self.mpu_addr,
                &[MPURegisters::POWER_MANAGEMENT, register_value],
            )
            .unwrap();
    }
}

impl<I> Accelerometer for MPU6050Sensor<I>
where
    I: I2cGenericDriver,
{
    fn get_acceleration_vector(&mut self) -> AccelerationVector3D {
        let acc_values = self.get_accel_data();
        self.map_accel_out_to_vector(acc_values)
    }

    fn get_roll_pitch_angles(
        &mut self,
        acceleration_vector: AccelerationVector3D,
    ) -> RotationVector2D {
        let roll = atan2f(
            -acceleration_vector.x,
            sqrtf(acceleration_vector.y.powf(2.0) + acceleration_vector.z.powf(2.0)),
        );

        let pitch = atan2f(
            acceleration_vector.y,
            sqrtf(acceleration_vector.x.powf(2.0) + acceleration_vector.z.powf(2.0)),
        );

        RotationVector2D {
            roll: roll.to_degrees(),
            pitch: pitch.to_degrees(),
        }
    }
}

impl<I> Gyroscope for MPU6050Sensor<I>
where
    I: I2cGenericDriver,
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

impl<I> CombinedGyroscopeAccelerometer for MPU6050Sensor<I>
where
    I: I2cGenericDriver,
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
            .unwrap();

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
