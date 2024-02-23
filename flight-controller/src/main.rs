#![feature(trait_alias)]
#![feature(ptr_metadata)]

mod communication_interfaces;
mod control;
mod drivers;
mod output;
mod util;
pub mod config {
    pub mod constants;
}

use std::ffi::{c_void, CString};
use std::sync::{Arc, RwLock};

use config::constants::{
    ACCEL_X_DEVIATION, ACCEL_Y_DEVIATION, ACCEL_Z_DEVIATION, GYRO_PITCH_CALIBRATION_DEG,
    GYRO_ROLL_CALIBRATION_DEG, GYRO_YAW_CALIBRATION_DEG, MAX_POWER, MIN_POWER, VEHICLE_TYPE,
};
use control::control_loops::FlightStabilizerOutCommands;
use drivers::mpu_6050::{LowPassFrequencyValues, MPU6050Sensor};
use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::sys::{esp_pm_config_t, esp_pm_configure, vTaskDelete, xTaskCreatePinnedToCore};
use output::motors_state_manager::QuadcopterMotorsStateManager;
use output::vehicle_movement_mappers::{
    FlyingVehicleMovementMapper, Quadcopter, VehicleTypesMapper,
};
use shared_definitions::controller::ControllerInput;
use util::vectors::{AccelerationVector3D, RotationVector2D, RotationVector3D};

use crate::communication_interfaces::wifi_control::WifiController;
use crate::config::constants::CONTROLLER_TYPE;
use crate::control::control_loops::start_flight_controllers;

use crate::communication_interfaces::{i2c::*, ControllerTypes};

#[derive(Default, Debug)]
pub struct TelemetryDataValues {
    loop_exec_time_us: u128,
    rate_controller_output: RotationVector3D,
    angle_controller_output: RotationVector2D,
    kalman_predicted_state: RotationVector2D,
    rotation_rate: RotationVector3D,
    accelerometer_rotation: RotationVector2D,
    motors_power: [f32; 4],
    throttle: f32,
}

struct FlightThreadInput {
    controller_input: Arc<RwLock<ControllerInput>>,
    telemetry_data: Arc<RwLock<TelemetryDataValues>>,
}

fn flight_thread(
    controller_input: Arc<RwLock<ControllerInput>>,
    telemetry_data: Arc<RwLock<TelemetryDataValues>>,
) {
    let mut peripherals: Peripherals = Peripherals::take().unwrap();
    let i2c_driver = get_i2c_driver(&mut peripherals);
    let mut motors_manager = QuadcopterMotorsStateManager::new(&mut peripherals);

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
    imu.enable_low_pass_filter(LowPassFrequencyValues::Freq10Hz);
    // Print telemetry values thread, for debugging/telemetry purposes, later will move this to it's own thread to send to controller.
    {
        // let controller_input = controller_input.clone();
        let telemetry_data = telemetry_data.clone();

        let _ = std::thread::Builder::new()
            .stack_size(4096)
            .spawn(move || loop {
                // let control_input_values = control_input_arc.read().unwrap();
                // let values = control_input_arc.read().unwrap();
                let debug_values_lock = telemetry_data.read().unwrap();
                println!(
                    "Run Time: {:?} \n Rate Out: {:?} \n Angle Out: {:?} \n Motor {:?} \n Throttle {:?}" ,
                    debug_values_lock.loop_exec_time_us,
                    debug_values_lock.rate_controller_output,
                    debug_values_lock.angle_controller_output,
                    debug_values_lock.motors_power,
                    debug_values_lock.throttle
                );
                // println!("Curent values{:?}", values);
                drop(debug_values_lock);
                // drop(values);
                FreeRtos::delay_ms(1000);
            });
    }

    let output_handler = match VEHICLE_TYPE {
        VehicleTypesMapper::Quadcopter => {
            let quadcoper_movement_mapper = Quadcopter::new(MIN_POWER, MAX_POWER);
            let telemetry_data = telemetry_data.clone();
            move |result| match result {
                FlightStabilizerOutCommands::CalibrateMotorController() => {
                    motors_manager.calibrate_esc();
                }
                FlightStabilizerOutCommands::KillMotors() => motors_manager.kill_motors(),
                FlightStabilizerOutCommands::UpdateFlightState(state) => {
                    let quad_out = quadcoper_movement_mapper
                        .map_controller_output_to_actuators_input(
                            state.throttle,
                            state.rotation_output_command,
                        );
                    let mut telemetry_data_lock = telemetry_data.write().unwrap();
                    let mapped_to_motors_numbers = [
                        quad_out.motor_rear_left,
                        quad_out.motor_rear_right,
                        quad_out.motor_front_left,
                        quad_out.motor_front_right,
                    ];
                    telemetry_data_lock
                        .motors_power
                        .copy_from_slice(&mapped_to_motors_numbers);
                    telemetry_data_lock.throttle = state.throttle;
                    drop(telemetry_data_lock);
                    motors_manager.set_motor_power(mapped_to_motors_numbers);
                }
            }
        }
    };

    start_flight_controllers(controller_input, imu, telemetry_data, output_handler);
}

unsafe extern "C" fn flight_thread_task_entrypoint(params: *mut core::ffi::c_void) {
    let controller_input_ptr = params as *const _ as *const FlightThreadInput;
    let controller_input_shared = controller_input_ptr.read();
    flight_thread(
        controller_input_shared.controller_input,
        controller_input_shared.telemetry_data,
    );
    vTaskDelete(std::ptr::null_mut());
}

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Running");

    unsafe {
        let config = esp_pm_config_t {
            max_freq_mhz: 240,
            min_freq_mhz: 240,
            light_sleep_enable: false,
        };
        let config_pointer: *const c_void = &config as *const _ as *const c_void;
        esp_pm_configure(config_pointer);
        log::info!("Set core frequency");
    }

    let controller_input_shared = ControllerInput {
        yaw: 0,
        pitch: 0,
        roll: 0,
        throttle: 0,
        kill_motors: false,
        start: false,
        calibrate: false,
    };

    let control_input = Arc::new(RwLock::new(controller_input_shared));
    let telemetry_data = Arc::new(RwLock::new(TelemetryDataValues::default()));

    unsafe {
        xTaskCreatePinnedToCore(
            Some(flight_thread_task_entrypoint),
            CString::new("Comms Task").unwrap().as_ptr(),
            4096,
            &FlightThreadInput {
                controller_input: control_input.clone(),
                telemetry_data: telemetry_data.clone(),
            } as *const _ as *mut c_void,
            10,
            std::ptr::null_mut(),
            1,
        )
    };
    match CONTROLLER_TYPE {
        ControllerTypes::Wifi => {
            WifiController::init_monitor(13, control_input.clone());
        }
        _ => {
            panic!()
        }
    }
}
