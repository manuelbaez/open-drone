use std::{mem, sync::atomic::Ordering};

use esp_idf_svc::hal::delay::BLOCK;
use esp_idf_svc::hal::{delay::FreeRtos, i2c::I2cDriver, task::queue::Queue};
use once_cell::sync::Lazy;
use shared_definitions::controller::ControllerInput;

use crate::{
    config::constants::{
        ACCEL_UNCERTAINTY_DEG, GYRO_DRIFT_DEG, MAX_INCLINATION, MAX_ROTATION_RATE, MAX_THROTTLE,
        MIN_POWER,
    },
    control::flight_controllers::{
        AngleModeControllerInput, AngleModeFlightController, RotationRateControllerInput,
        RotationRateFlightController,
    },
    shared_core_values::{AtomicControllerInput, AtomicTelemetry},
    util::math::vectors::{AccelerationVector3D, RotationVector2D},
};
use crate::{drivers::imu_sensors::Accelerometer, util::math::vectors::RotationVector3D};
use crate::{
    drivers::{
        imu_sensors::{CombinedGyroscopeAccelerometer, Gyroscope},
        mpu_6050::device::MPU6050Sensor,
    },
    util::time::get_current_system_time,
};
#[cfg(feature = "wifi-tuning")]
use {crate::shared_core_values::SHARED_TUNING, shared_definitions::controller::PIDTuneInput};

const US_IN_SECOND: f32 = 1_000_000.0_f32;

#[derive(Clone, Copy)]
struct ImuMeasurement {
    measurement_time: i64,
    rotation_rates: RotationVector3D,
    acceleration: AccelerationVector3D,
}

static IMU_MEASUREMENTS_QUEUE: Lazy<Queue<ImuMeasurement>> = Lazy::new(|| Queue::new(3));

pub enum MainControlLoopOutCommands {
    KillMotors,
    UpdateMotorsState(FlightStabilizerOut),
    BypassThrottle(f32),
    StoreSensorsCalibration(AccelerationVector3D, RotationVector3D),
}

#[derive(Debug)]
pub struct FlightStabilizerOut {
    pub throttle: f32,
    pub rotation_output_command: RotationVector3D,
}

pub fn start_imu_measurements_loop(mut imu: MPU6050Sensor<I2cDriver<'_>>) -> ! {
    let mut acceleration = AccelerationVector3D::default();
    loop {
        // let (rotation_rates, acceleration) = imu.get_combined_gyro_accel_output();
        let rotation_rates = imu.get_rotation_rates();
        IMU_MEASUREMENTS_QUEUE
            .send_back(
                ImuMeasurement {
                    measurement_time: get_current_system_time(),
                    rotation_rates,
                    acceleration: acceleration.clone(),
                },
                0,
            )
            .ok();
    }
}

pub fn start_flight_controller_processing_loop(
    controller_input: &AtomicControllerInput,
    telemetry_data: &AtomicTelemetry,
    mut controllers_out_callback: impl FnMut(MainControlLoopOutCommands) -> (),
) -> ! {
    let mut previous_measurement_time_us = 0_i64;
    let mut previous_execution_time_us = 0_i64;

    let mut rotation_mode_flight_controller = RotationRateFlightController::new();
    let mut angle_flight_controller =
        AngleModeFlightController::new(MAX_ROTATION_RATE, GYRO_DRIFT_DEG, ACCEL_UNCERTAINTY_DEG);

    let mut drone_on = false;
    loop {
        let imu_measurement = IMU_MEASUREMENTS_QUEUE.recv_front(BLOCK);
        let imu_measurement = match imu_measurement {
            Some(measurement) => measurement.0,
            None => {
                continue;
            }
        };
        //Read remote control input values
        let input_values = ControllerInput {
            roll: controller_input.roll.load(Ordering::Relaxed),
            pitch: controller_input.pitch.load(Ordering::Relaxed),
            yaw: controller_input.yaw.load(Ordering::Relaxed),
            throttle: controller_input.throttle.load(Ordering::Relaxed),
            kill_motors: controller_input.kill_motors.load(Ordering::Relaxed),
            start: controller_input.start.load(Ordering::Relaxed),
            calibrate_esc: controller_input.calibrate_esc.load(Ordering::Relaxed),
            calibrate_sensors: controller_input.calibrate_sensors.load(Ordering::Relaxed),
        };

        #[cfg(feature = "wifi-tuning")]
        {
            let tune_values = PIDTuneInput {
                roll: SHARED_TUNING.roll.map_to_pid_input(),
                pitch: SHARED_TUNING.pitch.map_to_pid_input(),
                yaw: SHARED_TUNING.yaw.map_to_pid_input(),
            };
            rotation_mode_flight_controller.set_pid_tune(tune_values);
        }

        match drone_on {
            true => {
                if input_values.kill_motors {
                    drone_on = false;
                    controllers_out_callback(MainControlLoopOutCommands::KillMotors);
                    continue;
                }
            }
            false => {
                rotation_mode_flight_controller.reset();
                if input_values.start {
                    drone_on = true;
                }

                if input_values.calibrate_esc {
                    controllers_out_callback(MainControlLoopOutCommands::BypassThrottle(
                        (input_values.throttle as f32 / u8::max_value() as f32) * 100.0_f32,
                    ));
                }

                // if input_values.calibrate_sensors {
                //     log::info!("Calibrating gyro");
                //     let gyro_calibration_values = imu.calculate_drift_average();
                //     imu.set_drift_calibration(gyro_calibration_values.clone());
                //     log::info!("Gyro calibrated");

                //     log::info!("Calibrating Accelerometer");
                //     let accel_calibration_values = imu.calculate_deviation_average();
                //     imu.set_deviation_calibration(accel_calibration_values.clone());
                //     log::info!("Accelerometer calibrated");
                //     controllers_out_callback(MainControlLoopOutCommands::StoreSensorsCalibration(
                //         accel_calibration_values,
                //         gyro_calibration_values,
                //     ))
                // }

                FreeRtos::delay_ms(5);
                continue;
            }
        }

        // Calculate time since last iteration in seconds
        let current_execution_time = get_current_system_time();
        let current_measurement_time_us = imu_measurement.measurement_time;
        let time_since_last_measurement_seconds =
            (current_measurement_time_us - previous_measurement_time_us) as f32 / US_IN_SECOND;
        previous_measurement_time_us = current_measurement_time_us;

        let throttle: f32 = (input_values.throttle as f32 / u8::max_value() as f32)
            * (MAX_THROTTLE - MIN_POWER)
            + MIN_POWER;
        let desired_rotation = RotationVector3D {
            pitch: (input_values.pitch as f32 / i16::max_value() as f32) * MAX_INCLINATION,
            roll: (input_values.roll as f32 / i16::max_value() as f32) * MAX_INCLINATION,
            yaw: -(input_values.yaw as f32 / i16::max_value() as f32) * MAX_ROTATION_RATE, //My controller is inverted, probably shouldn't do that there lol
        };

        let acceleration_angles = imu_measurement.acceleration.calculate_orientation_angles();

        let angle_flight_controller_input = AngleModeControllerInput {
            measured_rotation_rate: RotationVector2D::from(&imu_measurement.rotation_rates),
            measured_rotation: acceleration_angles.clone(),
            desired_rotation: RotationVector2D::from(&desired_rotation),
            iteration_time: time_since_last_measurement_seconds,
        };

        let desired_rotation_rate_2d: RotationVector2D =
            angle_flight_controller.get_next_output(angle_flight_controller_input.clone());

        let mut desired_rotation_rate = RotationVector3D::from(&desired_rotation_rate_2d);
        desired_rotation_rate.yaw = desired_rotation.yaw;

        let rotation_flight_controller_input = RotationRateControllerInput {
            desired_rotation_rate,
            measured_rotation_rate: imu_measurement.rotation_rates.clone(),
            iteration_time: time_since_last_measurement_seconds,
        };

        let controller_output =
            rotation_mode_flight_controller.get_next_output(rotation_flight_controller_input);

        //Store telemetry data
        telemetry_data.loop_exec_time_us.store(
            (current_execution_time - previous_execution_time_us) as i32,
            Ordering::Relaxed,
        );

        previous_execution_time_us = current_execution_time;

        telemetry_data
            .throttle
            .store(throttle as u8, Ordering::Relaxed);
        telemetry_data
            .rotation_rate
            .store(imu_measurement.rotation_rates);

        controllers_out_callback(MainControlLoopOutCommands::UpdateMotorsState(
            FlightStabilizerOut {
                rotation_output_command: controller_output,
                throttle,
            },
        ));
    }
}
