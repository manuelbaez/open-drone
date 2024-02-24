use std::{
    sync::{Arc, RwLock},
    time::SystemTime,
};

use esp_idf_svc::hal::{delay::FreeRtos, i2c::I2cDriver};
use shared_definitions::controller::ControllerInput;

use crate::drivers::{
    imu_sensors::{CombinedGyroscopeAccelerometer, Gyroscope},
    mpu_6050::device::MPU6050Sensor,
};
use crate::{
    config::constants::{
        ACCEL_UNCERTAINTY_DEG, GYRO_DRIFT_DEG, MAX_INCLINATION, MAX_ROTATION_RATE, MAX_THROTTLE,
    },
    control::fligth_controllers::{
        AngleModeControllerInput, AngleModeFlightController, RotationRateControllerInput,
        RotationRateFlightController,
    },
    util::vectors::RotationVector2D,
    TelemetryDataValues,
};
use crate::{drivers::imu_sensors::Accelerometer, util::vectors::RotationVector3D};

const US_IN_SECOND: f32 = 1_000_000.0_f32;

pub enum FlightStabilizerOutCommands {
    CalibrateMotorController(),
    KillMotors(),
    UpdateFlightState(FlightStabilizerOut),
}

#[derive(Debug)]
pub struct FlightStabilizerOut {
    pub throttle: f32,
    pub rotation_output_command: RotationVector3D,
}

pub fn start_flight_controllers(
    controller_input: Arc<RwLock<ControllerInput>>,
    mut imu: MPU6050Sensor<I2cDriver<'_>>,
    telemetry_data: Arc<RwLock<TelemetryDataValues>>,
    mut controllers_out_callback: impl FnMut(FlightStabilizerOutCommands) -> (),
) {
    let system_time = SystemTime::now();
    let mut previous_time_us = 0_u128;

    let mut rotation_mode_flight_controller = RotationRateFlightController::new();
    let mut angle_flight_controller =
        AngleModeFlightController::new(MAX_ROTATION_RATE, GYRO_DRIFT_DEG, ACCEL_UNCERTAINTY_DEG);

    let mut drone_on = false;

    loop {
        // let time_a = system_time.elapsed().unwrap().as_micros();
        //Read remote control input values
        let input_values_lock = controller_input.read().unwrap(); // This maybe could be improved for performance
        let input_values = input_values_lock.clone();
        drop(input_values_lock);

        // Calculate time since last iteration in seconds
        let current_time_us: u128 = system_time.elapsed().unwrap().as_micros();
        let time_since_last_reading_seconds =
            (current_time_us - previous_time_us) as f32 / US_IN_SECOND;

        if !drone_on {
            rotation_mode_flight_controller.reset();
            if input_values.start {
                drone_on = true;
                log::info!("Calibrating gyro");
                let calibration_values = imu.calculate_drift_average();
                imu.set_drift_calibration(calibration_values);
                log::info!("Gyro calibrated");
            }

            if input_values.calibrate {
                controllers_out_callback(FlightStabilizerOutCommands::CalibrateMotorController())
            }

            FreeRtos::delay_ms(5);
            continue;
        } else {
            if input_values.kill_motors {
                drone_on = false;
                controllers_out_callback(FlightStabilizerOutCommands::KillMotors());
            }
        }

        let throttle: f32 = (input_values.throttle as f32 / u8::max_value() as f32) * MAX_THROTTLE;

        let desired_rotation = RotationVector3D {
            pitch: (input_values.pitch as f32 / i16::max_value() as f32) * MAX_INCLINATION,
            roll: (input_values.roll as f32 / i16::max_value() as f32) * MAX_INCLINATION,
            yaw: -(input_values.yaw as f32 / i16::max_value() as f32) * MAX_ROTATION_RATE, //My controller is inverted, probably should do that there lol
        };

        let (rotation_rates, acceleration_vector) = imu.get_combined_gyro_accel_output();
        let acceleration_angles = imu.get_roll_pitch_angles(acceleration_vector);
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

        let mut telemetry_data_lock = telemetry_data.write().unwrap();
        telemetry_data_lock.rate_controller_output = controller_output.clone();
        telemetry_data_lock.angle_controller_output = desired_rotation_rate_2d;
        telemetry_data_lock.kalman_predicted_state =
            angle_flight_controller.get_current_kalman_predicted_state();
        telemetry_data_lock.rotation_rate = rotation_rates.clone();
        telemetry_data_lock.accelerometer_rotation = acceleration_angles.clone();
        // let time_b = system_time.elapsed().unwrap().as_micros();
        // telemetry_data_lock.loop_exec_time_us = time_b - time_a;
        telemetry_data_lock.loop_exec_time_us = current_time_us - previous_time_us;
        drop(telemetry_data_lock);
        previous_time_us = current_time_us;

        controllers_out_callback(FlightStabilizerOutCommands::UpdateFlightState(
            FlightStabilizerOut {
                rotation_output_command: controller_output,
                throttle,
            },
        ));
    }
}
