use std::{
    sync::{Arc, RwLock},
    time::SystemTime,
};

use esp_idf_svc::hal::{delay::FreeRtos, i2c::I2cDriver};

use crate::{
    control::{
        fligth_controllers::{
            AngleModeControllerInput, AngleModeFlightController, RotationRateControllerInput,
            RotationRateFlightController,
        },
        inertial_measurement::{
            imu_sensors::{Accelerometer, Gyroscope},
            mpu_6050::MPU6050Sensor,
            vectors::{AccelerationVector3D, RotationVector2D, RotationVector3D},
        },
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
        let motor_power = motor_power.clone();

        let _ = std::thread::Builder::new().stack_size(4096).spawn(move || {
            let system_time = SystemTime::now();

            let mut previous_time_us = 0_u128;
            const GYRO_DRIFT_DEG: f32 = 3.0_f32;
            const ACCEL_UNCERTAINTY_DEG: f32 = 3.0_f32;
            const MAX_ROTATION_RATE: f32 = 75.0_f32;
            const MIN_POWER: f32 = 10.0_f32;
            const MAX_POWER: f32 = 20.0_f32;

            let mut rotation_mode_flight_controller =
                RotationRateFlightController::new(MIN_POWER, MAX_POWER);
            let mut angle_flight_controller = AngleModeFlightController::new(
                MAX_ROTATION_RATE,
                GYRO_DRIFT_DEG,
                ACCEL_UNCERTAINTY_DEG,
            );

            loop {
                let throttle: f32 = 0.0_f32;
                let system_on = throttle > 20.0_f32;
                if system_on {
                    motors_manager.initialize_esc();
                    break;
                }
                FreeRtos::delay_ms(10);
            }

            loop {
                let throttle: f32 = 0.0_f32;

                let current_time_us: u128 = system_time.elapsed().unwrap().as_micros();
                let time_since_last_reading_seconds =
                    (current_time_us - previous_time_us) as f32 / 1_000_000.0_f32;
                previous_time_us = current_time_us;

                let rotation_rates = imu.get_rotation_rates();
                let acceleration_angles = imu.get_roll_pitch_angles();

                let desired_rotation = RotationVector3D {
                    pitch: 0.0,
                    roll: 0.0,
                    yaw: 0.0,
                };

                let angle_flight_controller_input = AngleModeControllerInput {
                    measured_rotation_rate: RotationVector2D::from(&rotation_rates),
                    measured_rotation: acceleration_angles.clone(),
                    desired_rotation: RotationVector2D::from(&desired_rotation),
                    iteration_time: time_since_last_reading_seconds,
                };

                let desired_rotation_rate_2d: RotationVector2D =
                    angle_flight_controller.get_next_output(angle_flight_controller_input);

                let mut desired_rotation_rate = RotationVector3D::from(&desired_rotation_rate_2d);
                desired_rotation_rate.yaw = desired_rotation.yaw;

                let rotation_flight_controller_input = RotationRateControllerInput {
                    throttle,
                    desired_rotation_rate,
                    measured_rotation_rate: rotation_rates,
                    iteration_time: time_since_last_reading_seconds,
                };

                let motor_output = rotation_mode_flight_controller
                    .get_next_output(rotation_flight_controller_input);

                motors_manager.set_motor_power(motor_output);

                let mut power = motor_power.write().unwrap();
                power[0] = motor_output[0];
                power[1] = motor_output[1];
                power[2] = motor_output[2];
                power[3] = motor_output[3];
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
                FreeRtos::delay_ms(200);
            });
    }
}
