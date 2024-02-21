use std::{
    sync::{Arc, RwLock},
    time::SystemTime,
};

use esp_idf_svc::hal::{delay::FreeRtos, i2c::I2cDriver, peripherals::Peripherals};

use crate::{
    communication_interfaces::wifi_control::ControllerInput,
    config::constants::{
        ACCEL_X_DEVIATION, ACCEL_Y_DEVIATION, ACCEL_Z_DEVIATION, GYRO_PITCH_CALIBRATION_DEG,
        GYRO_ROLL_CALIBRATION_DEG, GYRO_YAW_CALIBRATION_DEG,
    },
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
    get_i2c_driver,
    output::motors_state_manager::MotorsStateManager,
};

const US_IN_SECOND: f32 = 1_000_000.0_f32;

#[derive(Default, Debug)]
struct DebugValues {
    exec_time_us: u128,
    motor_power_values: [f32; 4],
    angle_controller_output: RotationVector2D,
}

fn stabilizer_loop(
    imu: &mut MPU6050Sensor<I2cDriver<'static>>,
    motors_manager: &mut MotorsStateManager,
    debug_values: Arc<RwLock<DebugValues>>,
    control_input_values: Arc<RwLock<ControllerInput>>,
) {
    let system_time = SystemTime::now();

    let mut previous_time_us = 0_u128;

    let mut rotation_mode_flight_controller =
        RotationRateFlightController::new(MIN_POWER, MAX_POWER);
    let mut angle_flight_controller =
        AngleModeFlightController::new(MAX_ROTATION_RATE, GYRO_DRIFT_DEG, ACCEL_UNCERTAINTY_DEG);

    let mut drone_on = false;

    loop {
        let input_values_lock = control_input_values.read().unwrap();
        let current_time_us: u128 = system_time.elapsed().unwrap().as_micros();
        let time_since_last_reading_seconds =
            (current_time_us - previous_time_us) as f32 / US_IN_SECOND;

        let input_values = ControllerInput {
            roll: input_values_lock.roll,
            yaw: input_values_lock.yaw,
            pitch: input_values_lock.pitch,
            start: input_values_lock.start,
            kill_motors: input_values_lock.kill_motors,
            calibrate: input_values_lock.calibrate,
            throttle: input_values_lock.throttle,
        };
        drop(input_values_lock);

        let mut debug_values_lock = debug_values.write().unwrap();
        debug_values_lock.exec_time_us = current_time_us - previous_time_us;
        drop(debug_values_lock);

        previous_time_us = current_time_us;

        if input_values.start {
            drone_on = true;
        }
        if input_values.kill_motors {
            drone_on = false;
        }

        if input_values.calibrate && !drone_on {
            motors_manager.calibrate_esc();
        }

        if !drone_on {
            motors_manager.set_motor_power([0.0, 0.0, 0.0, 0.0]);
            rotation_mode_flight_controller.reset();
            FreeRtos::delay_ms(5);
            continue;
        }

        let throttle: f32 = (input_values.throttle as f32 / u8::max_value() as f32) * MAX_THROTTLE;

        let desired_rotation = RotationVector3D {
            pitch: (input_values.pitch as f32 / i32::max_value() as f32) * MAX_INCLINATION,
            roll: (input_values.roll as f32 / i32::max_value() as f32) * MAX_INCLINATION,
            yaw: (input_values.yaw as f32 / i32::max_value() as f32) * 100.0_f32,
        };

        let rotation_rates = imu.get_rotation_rates();
        let acceleration_angles = imu.get_roll_pitch_angles();

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

        let motor_output =
            rotation_mode_flight_controller.get_next_output(rotation_flight_controller_input);

        motors_manager.set_motor_power(motor_output);

        let mut debug_values_lock = debug_values.write().unwrap();
        debug_values_lock
            .motor_power_values
            .copy_from_slice(&motor_output);
        debug_values_lock.angle_controller_output = desired_rotation_rate_2d;
        drop(debug_values_lock);
    }
}

pub fn start_flight_stabilizer(control_input_value: Arc<RwLock<ControllerInput>>) {
    let mut peripherals: Peripherals = Peripherals::take().unwrap();
    let mut motors_manager = MotorsStateManager::new();
    motors_manager.initialize_motor_controllers(&mut peripherals);

    let i2c_driver = get_i2c_driver(&mut peripherals);

    let acceleromenter_calibration = AccelerationVector3D {
        x: ACCEL_X_DEVIATION,
        y: ACCEL_Y_DEVIATION,
        z: ACCEL_Z_DEVIATION,
    };

    let gyro_calibration = RotationVector3D {
        roll: GYRO_ROLL_CALIBRATION_DEG,
        pitch: GYRO_PITCH_CALIBRATION_DEG,
        yaw: GYRO_YAW_CALIBRATION_DEG,
    };

    let mut imu = MPU6050Sensor::new(i2c_driver, gyro_calibration, acceleromenter_calibration);

    let debug_values = Arc::new(RwLock::new(DebugValues::default()));
    // // Print values thread, for debugging purposes.
    {
        let control_input_arc = control_input_value.clone();
        let debug_values = debug_values.clone();

        let _ = std::thread::Builder::new()
            .stack_size(4096)
            .spawn(move || loop {
                // let control_input_values = control_input_arc.read().unwrap();
                let values = control_input_arc.read().unwrap();
                let debug_values_lock = debug_values.read().unwrap();
                println!("Debug Values {:?}", debug_values_lock);
                // println!("Curent values{:?}", values);
                drop(debug_values_lock);
                drop(values);
                FreeRtos::delay_ms(50);
            });
    }

    stabilizer_loop(
        &mut imu,
        &mut motors_manager,
        debug_values,
        control_input_value,
    );
}
