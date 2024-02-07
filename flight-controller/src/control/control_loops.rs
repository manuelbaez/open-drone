use std::{
    sync::{Arc, RwLock},
    time::SystemTime,
};

use esp_idf_svc::hal::{delay::FreeRtos, i2c::I2cDriver};

use crate::{
    control::{
        fligth_controllers::{RotationRateControllerInput, RotationRateFlightController},
        inertial_measurement::{
            imu_sensor_traits::{Accelerometer, Gyroscope},
            mpu_6050::MPU6050Sensor,
            vectors::{AccelerationVector3D, RotationVector3D},
        },
        kalman_filter::KalmanFilter,
    },
    output::motors_state_manager::MotorsStateManager,
};

const GYRO_PITCH_CALIBRATION_DEG: f32 = -1.9987461681754335;
const GYRO_ROLL_CALIBRATION_DEG: f32 = -0.03329770963243634;
const GYRO_YAW_CALIBRATION_DEG: f32 = -0.06639503742574737;

pub fn init_flight_stabilizer_thread(
    i2c_driver: I2cDriver<'static>,
    mut motors_manager: MotorsStateManager,
) {
    let gyro_angles_kalman = Arc::new(RwLock::new([
        [0.0_f32, 0.0_f32],
        [0.0_f32, 0.0_f32],
        [0.0_f32, 0.0_f32],
    ]));

    let motor_power = Arc::new(RwLock::new([0.0_f32, 0.0_f32, 0.0_f32, 0.0_f32]));

    let acceleromenter_calibration = AccelerationVector3D {
        x: 0.033,
        y: -0.035,
        z: 0.034,
    };

    let gyro_calibration = RotationVector3D {
        roll: GYRO_ROLL_CALIBRATION_DEG,
        pitch: GYRO_PITCH_CALIBRATION_DEG,
        yaw: GYRO_YAW_CALIBRATION_DEG,
    };

    let mut imu = MPU6050Sensor::new(i2c_driver, gyro_calibration, acceleromenter_calibration);

    {
        let gyro_angles_kalman = gyro_angles_kalman.clone();
        let motor_power = motor_power.clone();

        let _ = std::thread::Builder::new().stack_size(4096).spawn(move || {
            let system_time = SystemTime::now();

            //kalman filter variables
            let mut previous_time_us = 0_u128;
            const GYRO_DRIFT_DEG: f32 = 3.0_f32;
            const ACCEL_UNCERTAINTY_DEG: f32 = 3.0_f32;
            const ACCEL_ROLL_CALIBRATION_DEG: f32 = -2.01;
            const ACCEL_PITCH_CALIBRATION_DEG: f32 = -2.54;

            let mut pitch_kalman_filter: KalmanFilter =
                KalmanFilter::new(GYRO_DRIFT_DEG, ACCEL_UNCERTAINTY_DEG);
            let mut roll_kalman_filter: KalmanFilter =
                KalmanFilter::new(GYRO_DRIFT_DEG, ACCEL_UNCERTAINTY_DEG);

            let mut rotation_mode_flight_controller = RotationRateFlightController::new(10.0, 20.0);

            loop {
                let current_time_us: u128 = system_time.elapsed().unwrap().as_micros();

                let time_since_last_reading_seconds =
                    (current_time_us - previous_time_us) as f32 / 1_000_000.0_f32;
                previous_time_us = current_time_us;

                let rotation_rates = imu.get_rotation_rates();
                let acceleration_angles = imu.get_roll_pitch_angles();

                pitch_kalman_filter.apply_flilter_update(
                    rotation_rates.pitch,
                    acceleration_angles.pitch,
                    time_since_last_reading_seconds,
                );

                roll_kalman_filter.apply_flilter_update(
                    rotation_rates.roll,
                    acceleration_angles.roll,
                    time_since_last_reading_seconds,
                );

                let controller_input = RotationRateControllerInput {
                    throttle: 15.0,
                    desired_roll_rate: 0.0,
                    desired_pitch_rate: 0.0,
                    desired_yaw_rate: 0.0,
                    measured_roll_rate: rotation_rates.roll,
                    measured_pitch_rate: rotation_rates.pitch,
                    measured_yaw_rate: rotation_rates.yaw,
                    iteration_time: time_since_last_reading_seconds,
                };

                let motor_output =
                    rotation_mode_flight_controller.get_next_output(controller_input);

                // log::info!(
                //     "Yaw {} Motors power: {}-{}-{}-{}",
                //     rotation_rates.yaw,
                //     motor_output[0],
                //     motor_output[1],
                //     motor_output[2],
                //     motor_output[3]
                // );

                motors_manager.set_motor_power(motor_output);

                let mut power = motor_power.write().unwrap();
                power[0] = motor_output[0];
                power[1] = motor_output[1];
                power[2] = motor_output[2];
                power[3] = motor_output[3];

                let mut angles = gyro_angles_kalman.write().unwrap();

                angles[0][0] =
                    (roll_kalman_filter.get_current_state() * 100.0_f32).round() / 100.0_f32;
                angles[0][1] = roll_kalman_filter.get_current_uncertainty();

                angles[1][0] =
                    (pitch_kalman_filter.get_current_state() * 100.0_f32).round() / 100.0_f32;
                angles[1][1] = pitch_kalman_filter.get_current_uncertainty();

                drop(angles);
                FreeRtos::delay_ms(1);
            }
        });
    }

    // Print values thread
    {
        // let gyro_angles_kalman = gyro_angles_kalman.clone();
        let motor_power = motor_power.clone();

        let _ = std::thread::Builder::new()
            .stack_size(4096)
            .spawn(move || loop {
                // let angles = gyro_angles_kalman.read().unwrap();
                let power = motor_power.read().unwrap();
                print!("Curent POWER {:?}\n", power);
                drop(power);
                FreeRtos::delay_ms(500);
            });
    }
}
