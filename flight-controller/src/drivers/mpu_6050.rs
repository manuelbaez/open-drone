use esp_idf_svc::hal::delay::FreeRtos;
use libm::{atan2f, sqrtf};
use mpu6050::{device::GyroRange, Mpu6050};

use crate::{
    util::vectors::{AccelerationVector3D, RotationVector2D, RotationVector3D},
    I2cGenericDriver,
};

use super::imu_sensors::{Accelerometer, Gyroscope};

struct MPURegisters;
impl MPURegisters {
    pub const GYRO_LOW_PASS_FILTER: u8 = 0x1A;
}

#[repr(u8)]
#[allow(dead_code)]
pub enum LowPassFrequencyValues {
    Freq10Hz = 0x5,
    Freq21Hz = 0x4,
}

pub struct MPU6050Sensor<I> {
    driver: Mpu6050<I>,
    gyro_drift_calibration: RotationVector3D,
    accelerometer_calibration: AccelerationVector3D,
}

impl<I> MPU6050Sensor<I>
where
    I: I2cGenericDriver,
{
    pub fn new(
        i2c_driver: I,
        gyro_rate_calibration: RotationVector3D,
        accelerometer_calibration: AccelerationVector3D,
    ) -> Self {
        let mut mpu = Mpu6050::new(i2c_driver);
        mpu.init(&mut FreeRtos).unwrap();
        mpu.set_gyro_range(GyroRange::D500).unwrap();
        FreeRtos::delay_ms(100);
        MPU6050Sensor {
            driver: mpu,
            gyro_drift_calibration: gyro_rate_calibration,
            accelerometer_calibration,
        }
    }

    pub fn enable_low_pass_filter(&mut self, low_pass_freq: LowPassFrequencyValues) {
        self.driver
            .write_byte(MPURegisters::GYRO_LOW_PASS_FILTER, low_pass_freq as u8)
            .unwrap(); //Set to 10hz
        FreeRtos::delay_ms(100);
    }
}

impl<I> Accelerometer for MPU6050Sensor<I>
where
    I: I2cGenericDriver,
{
    fn get_acceleration_vector(&mut self) -> AccelerationVector3D {
        let acc_values = self.driver.get_acc().unwrap();
        // Z is negative here because apparently the gyro/accel is upside down and the roll angle is inverted
        AccelerationVector3D {
            x: acc_values[0] - self.accelerometer_calibration.x,
            y: acc_values[1] - self.accelerometer_calibration.y,
            z: acc_values[2] - self.accelerometer_calibration.z,
        }
    }

    fn get_roll_pitch_angles(&mut self) -> RotationVector2D {
        let acceleration_vector = self.get_acceleration_vector();
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
        let gyro_rates_xyz = self.driver.get_gyro().unwrap();

        let pitch = gyro_rates_xyz[0].to_degrees(); // Pitch is rotation along the x axis
        let roll = gyro_rates_xyz[1].to_degrees(); // Roll is rotation along the y axis
        let yaw = gyro_rates_xyz[2].to_degrees(); // Yaw is rotation along the z axis
                                                  // See right hand rule to understand how positive and negative rotation rates are measured.
        RotationVector3D { pitch, roll, yaw }
    }

    fn get_rotation_rates(&mut self) -> RotationVector3D {
        self.get_rotation_rates_uncalibrated() - self.gyro_drift_calibration.clone()
    }

    fn set_drift_calibration(&mut self, claibration: RotationVector3D) {
        self.gyro_drift_calibration = claibration;
    }
}
