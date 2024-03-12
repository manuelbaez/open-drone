use core::sync::atomic::Ordering;

use embassy_time::Timer;
use esp32_hal::i2c;
use shared_definitions::controller::ControllerInput;

use crate::drivers::{
    imu_sensors::{CombinedGyroscopeAccelerometer, Gyroscope},
    mpu_6050::device::MPU6050Sensor,
};
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
    util::time::get_current_system_time_us,
};
use crate::{drivers::imu_sensors::Accelerometer, util::math::vectors::RotationVector3D};
#[cfg(feature = "wifi-tuning")]
use {crate::shared_core_values::SHARED_TUNING, shared_definitions::controller::PIDTuneInput};

const US_IN_SECOND: f32 = 1_000_000.0_f32;

pub enum MainControlLoopOutCommands {
    KillMotors,
    UpdateFlightState(FlightStabilizerOut),
    BypassThrottle(f32),
    StoreSensorsCalibration(AccelerationVector3D, RotationVector3D),
}

#[derive(Debug)]
pub struct FlightStabilizerOut {
    pub throttle: f32,
    pub rotation_output_command: RotationVector3D,
}

pub async fn start_flight_controllers<'a, I: i2c::Instance>(
    controller_input: &'a AtomicControllerInput,
    mut imu: MPU6050Sensor<'a, I>,
    telemetry_data: &'a AtomicTelemetry,
    mut controllers_out_callback: impl FnMut(MainControlLoopOutCommands) -> (),
) -> ! {
    let mut previous_time_us = 0_u64;
    
    let mut rotation_mode_flight_controller = RotationRateFlightController::new();
    let mut angle_flight_controller =
    AngleModeFlightController::new(MAX_ROTATION_RATE, GYRO_DRIFT_DEG, ACCEL_UNCERTAINTY_DEG);
    
    let mut drone_on = false;
    
    loop {
        // let time_a = get_current_system_time_us();
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

                if input_values.calibrate_sensors {
                    esp_println::println!("Calibrating gyro");
                    let gyro_calibration_values = imu.calculate_drift_average();
                    imu.set_drift_calibration(gyro_calibration_values.clone());
                    esp_println::println!("Gyro calibrated");

                    esp_println::println!("Calibrating Accelerometer");
                    let accel_calibration_values = imu.calculate_deviation_average();
                    imu.set_deviation_calibration(accel_calibration_values.clone());
                    esp_println::println!("Accelerometer calibrated");
                    controllers_out_callback(MainControlLoopOutCommands::StoreSensorsCalibration(
                        accel_calibration_values,
                        gyro_calibration_values,
                    ))
                }

                Timer::after_millis(5).await;
                continue;
            }
        }

        // Calculate time since last iteration in seconds
        let current_time_us = get_current_system_time_us();
        let time_since_last_reading_seconds =
            (current_time_us - previous_time_us) as f32 / US_IN_SECOND;

        // esp_println::println!("From Control Loop {}", current_time_us - previous_time_us);
        let throttle: f32 = (input_values.throttle as f32 / u8::max_value() as f32)
            * (MAX_THROTTLE - MIN_POWER)
            + MIN_POWER;
        let desired_rotation = RotationVector3D {
            pitch: (input_values.pitch as f32 / i16::max_value() as f32) * MAX_INCLINATION,
            roll: (input_values.roll as f32 / i16::max_value() as f32) * MAX_INCLINATION,
            yaw: -(input_values.yaw as f32 / i16::max_value() as f32) * MAX_ROTATION_RATE, //My controller is inverted, probably shouldn't do that there lol
        };

        let (rotation_rates, acceleration_vector) = imu.get_combined_gyro_accel_output();
        let acceleration_angles = acceleration_vector.calculate_orientation_angles();

        let angle_flight_controller_input = AngleModeControllerInput {
            measured_rotation_rate: RotationVector2D::from(&rotation_rates),
            measured_rotation: acceleration_angles.clone(),
            desired_rotation: RotationVector2D::from(&desired_rotation),
            iteration_time: time_since_last_reading_seconds,
        };

        let desired_rotation_rate_2d: RotationVector2D =
            angle_flight_controller.get_next_output(angle_flight_controller_input.clone());

        let mut desired_rotation_rate = RotationVector3D::from(&desired_rotation_rate_2d);
        desired_rotation_rate.yaw = desired_rotation.yaw;

        let rotation_flight_controller_input = RotationRateControllerInput {
            desired_rotation_rate,
            measured_rotation_rate: rotation_rates.clone(),
            iteration_time: time_since_last_reading_seconds,
        };

        let controller_output =
            rotation_mode_flight_controller.get_next_output(rotation_flight_controller_input);

        // let time_b = get_current_system_time_us();

        //Store telemetry data
        telemetry_data.loop_exec_time_us.store(
            (current_time_us - previous_time_us) as i32,
            Ordering::Relaxed,
        );
        telemetry_data
            .throttle
            .store(throttle as u8, Ordering::Relaxed);
        telemetry_data.rotation_rate.store(rotation_rates);

        previous_time_us = current_time_us;

        controllers_out_callback(MainControlLoopOutCommands::UpdateFlightState(
            FlightStabilizerOut {
                rotation_output_command: controller_output,
                throttle,
            },
        ));
    }
}
