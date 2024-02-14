use esp_idf_svc::hal::delay::FreeRtos;
use libm::{atan2f, sqrtf};
use mpu6050::Mpu6050;

use crate::I2cGenericDriver;

use super::{
    imu_sensors::{Accelerometer, Gyroscope},
    vectors::{AccelerationVector3D, RotationVector2D, RotationVector3D},
};

const GYRO_HIGHPASS_FILTER: f32 = 0.1;

pub struct MPU6050Sensor<I> {
    driver: Mpu6050<I>,
    gyro_rate_calibration: RotationVector3D,
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

        MPU6050Sensor {
            driver: mpu,
            gyro_rate_calibration,
            accelerometer_calibration,
        }
    }
}

impl<I> Accelerometer for MPU6050Sensor<I>
where
    I: I2cGenericDriver,
{
    fn get_acceleration_vector(&mut self) -> AccelerationVector3D {
        let acc_values = self.driver.get_acc().unwrap();
        AccelerationVector3D {
            x: acc_values[0] - self.accelerometer_calibration.x,
            y: acc_values[1] - self.accelerometer_calibration.y,
            z: acc_values[2] - self.accelerometer_calibration.z,
        }
    }

    fn get_roll_pitch_angles(&mut self) -> RotationVector2D {
        let acceleration_vector = self.get_acceleration_vector();
        let roll = atan2f(
            acceleration_vector.y,
            sqrtf(acceleration_vector.x.powf(2.0) + acceleration_vector.z.powf(2.0)),
        );

        let pitch = atan2f(
            -acceleration_vector.x,
            sqrtf(acceleration_vector.y.powf(2.0) + acceleration_vector.z.powf(2.0)),
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
    fn get_rotation_rates(&mut self) -> RotationVector3D {
        let gyro_rates = self.driver.get_gyro().unwrap();

        let roll = gyro_rates[0].to_degrees() - self.gyro_rate_calibration.roll;
        let pitch = gyro_rates[1].to_degrees() - self.gyro_rate_calibration.pitch;
        let yaw = gyro_rates[2].to_degrees() - self.gyro_rate_calibration.yaw;

        RotationVector3D {
            roll: if roll.abs() > GYRO_HIGHPASS_FILTER {
                roll
            } else {
                0.0
            },
            pitch: if pitch.abs() > GYRO_HIGHPASS_FILTER {
                pitch
            } else {
                0.0
            },
            yaw: if yaw.abs() > GYRO_HIGHPASS_FILTER {
                yaw
            } else {
                0.0
            },
        }
    }
}
