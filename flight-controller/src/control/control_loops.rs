use std::{
    sync::{Arc, RwLock},
    time::SystemTime,
};

use esp_idf_svc::hal::{delay::FreeRtos, i2c::I2cDriver, peripherals::Peripherals};

use crate::{
    communication_interfaces::wifi::{ControllerInput, CONTROLLER_INPUT_DATA},
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

const GYRO_PITCH_CALIBRATION_DEG: f32 = -1.9987461681754335;
const GYRO_ROLL_CALIBRATION_DEG: f32 = -0.03329770963243634;
const GYRO_YAW_CALIBRATION_DEG: f32 = -0.06639503742574737;
const US_IN_SECOND: f32 = 1_000_000.0_f32;

fn stabilizer_loop(
    imu: &mut MPU6050Sensor<I2cDriver<'static>>,
    motors_manager: &mut MotorsStateManager,
    motors_power: Arc<RwLock<[f32; 4]>>,
    control_input_values: Arc<RwLock<ControllerInput>>,
) {
    let system_time = SystemTime::now();

    let mut previous_time_us = 0_u128;
    const GYRO_DRIFT_DEG: f32 = 3.0_f32;
    const ACCEL_UNCERTAINTY_DEG: f32 = 3.0_f32;
    const MAX_ROTATION_RATE: f32 = 75.0_f32;
    const MIN_POWER: f32 = 10.0_f32;
    const MAX_POWER: f32 = 60.0_f32;

    let mut rotation_mode_flight_controller =
        RotationRateFlightController::new(MIN_POWER, MAX_POWER);
    let mut angle_flight_controller =
        AngleModeFlightController::new(MAX_ROTATION_RATE, GYRO_DRIFT_DEG, ACCEL_UNCERTAINTY_DEG);

    loop {
        let input_values = control_input_values.read().unwrap();
        if input_values.start {
            motors_manager.initialize_esc();
            drop(input_values);
            break;
        }
        drop(input_values);

        FreeRtos::delay_ms(10);
    }

    loop {
        let input_values = control_input_values.read().unwrap();
        if input_values.kill_motors {
            motors_manager.set_motor_power([0.0, 0.0, 0.0, 0.0]);
            break;
        }
        let throttle: f32 = (input_values.throttle as f32 / u8::max_value() as f32) * 100.0_f32;
        let desired_rotation = RotationVector3D {
            pitch: (input_values.pitch as f32 / i32::max_value() as f32) * 100.0_f32,
            roll: (input_values.roll as f32 / i32::max_value() as f32) * 100.0_f32,
            yaw: (input_values.yaw as f32 / i32::max_value() as f32) * 100.0_f32,
        };
        drop(input_values);

        let current_time_us: u128 = system_time.elapsed().unwrap().as_micros();
        let time_since_last_reading_seconds =
            (current_time_us - previous_time_us) as f32 / US_IN_SECOND;

        let rotation_rates = imu.get_rotation_rates();
        let acceleration_angles: RotationVector2D = imu.get_roll_pitch_angles();

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

        let mut power = motors_power.write().unwrap();
        power[0] = (current_time_us - previous_time_us) as f32;
        previous_time_us = current_time_us;
        power[1] = motor_output[1];
        power[2] = motor_output[2];
        power[3] = motor_output[3];
    }
}

pub fn start_flight_stabilizer(control_input_value: Arc<RwLock<ControllerInput>>) {
    let mut peripherals: Peripherals = Peripherals::take().unwrap();
    let mut motors_manager = MotorsStateManager::new();
    motors_manager.initialize_motor_controllers(&mut peripherals);

    let i2c_driver = get_i2c_driver(&mut peripherals);

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

    let motors_power = Arc::new(RwLock::new([0.0_f32, 0.0_f32, 0.0_f32, 0.0_f32]));
    // Print values thread, for debugging purposes.
    {
        let control_input_arc = control_input_value.clone();
        let motor_power = motors_power.clone();

        let _ = std::thread::Builder::new()
            .stack_size(4096)
            .spawn(move || loop {
                // let control_input_values = control_input_arc.read().unwrap();
                let values = control_input_arc.read().unwrap();
                let power = motor_power.read().unwrap();
                print!("Curent POWER {:?}\n", power);
                print!("Curent values{:?}\n", values);
                drop(values);
                drop(power);
                FreeRtos::delay_ms(500);
            });
    }

    stabilizer_loop(
        &mut imu,
        &mut motors_manager,
        motors_power,
        control_input_value,
    );
    // {
    //     let motors_power = motors_power.clone();
    //     let _thread = std::thread::Builder::new()
    //         .stack_size(4096)
    //         .spawn(move || stabilizer_loop(&mut imu, &mut motors_manager, motors_power));
    // }
}
