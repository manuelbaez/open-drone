#![feature(trait_alias)]
#![feature(ptr_metadata)]

mod communication_interfaces;
mod control;
mod drivers;
mod output;
mod shared_core_values;
mod telemetry;
mod util;
pub mod config {
    pub mod constants;
}

use crate::communication_interfaces::controller::RemoteControl;
use crate::communication_interfaces::ibus::IBusController;
use crate::communication_interfaces::wifi_control::WifiController;
use crate::communication_interfaces::{i2c::*, ControllerTypes};
use crate::config::constants::CONTROLLER_TYPE;
use crate::control::control_loops::start_flight_controllers;
use crate::shared_core_values::{INPUT_SHARED, TELEMETRY_SHARED};
use crate::telemetry::start_telemetry_thread;
use config::constants::{
    ACCEL_X_DEVIATION, ACCEL_Y_DEVIATION, ACCEL_Z_DEVIATION, GYRO_PITCH_CALIBRATION_DEG,
    GYRO_ROLL_CALIBRATION_DEG, GYRO_YAW_CALIBRATION_DEG, MAX_POWER_OVER_THROTTLE, MIN_POWER,
    VEHICLE_TYPE,
};
use control::control_loops::FlightStabilizerOutCommands;
use drivers::mpu_6050::device::MPU6050Sensor;
use drivers::mpu_6050::registers::LowPassFrequencyValues;
use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::sys::{
    esp_pm_config_t, esp_pm_configure, esp_pm_lock_acquire, esp_pm_lock_handle_t,
    esp_pm_lock_type_t_ESP_PM_CPU_FREQ_MAX, vTaskDelete, xTaskCreatePinnedToCore,
};
use output::motors_state_manager::QuadcopterMotorsStateManager;
use output::vehicle_movement_mappers::{
    FlyingVehicleMovementMapper, Quadcopter, VehicleTypesMapper,
};
use shared_core_values::{AtomicControllerInput, AtomicTelemetry};
use std::ffi::{c_void, CString};
use std::sync::atomic::Ordering;
use util::vectors::{AccelerationVector3D, RotationVector3D};

struct FlightThreadInput<'a> {
    peripherals: &'a mut Peripherals,
}

fn flight_thread(
    controller_input: &AtomicControllerInput,
    telemetry_data: &AtomicTelemetry,
    peripherals: &mut Peripherals,
) {
    let i2c_driver = get_i2c_driver(peripherals);
    //Wait for the i2c driver to initialize
    let mut motors_manager = QuadcopterMotorsStateManager::new(peripherals);

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
    imu.init();

    let output_handler = match VEHICLE_TYPE {
        VehicleTypesMapper::Quadcopter => {
            let quadcoper_movement_mapper = Quadcopter::new(MIN_POWER, MAX_POWER_OVER_THROTTLE);
            move |result| match result {
                FlightStabilizerOutCommands::KillMotors() => motors_manager.kill_motors(),
                FlightStabilizerOutCommands::BypassThrottle(throttle) => {
                    motors_manager.set_motor_power([throttle; 4]);
                }
                FlightStabilizerOutCommands::UpdateFlightState(state) => {
                    let quad_out = quadcoper_movement_mapper
                        .map_controller_output_to_actuators_input(
                            state.throttle,
                            state.rotation_output_command,
                        );
                    let mapped_to_motors_numbers = [
                        quad_out.motor_rear_left,
                        quad_out.motor_rear_right,
                        quad_out.motor_front_left,
                        quad_out.motor_front_right,
                    ];

                    telemetry_data
                        .motor_1_power
                        .store(mapped_to_motors_numbers[0] as u8, Ordering::Relaxed);
                    telemetry_data
                        .motor_2_power
                        .store(mapped_to_motors_numbers[1] as u8, Ordering::Relaxed);
                    telemetry_data
                        .motor_3_power
                        .store(mapped_to_motors_numbers[2] as u8, Ordering::Relaxed);
                    telemetry_data
                        .motor_4_power
                        .store(mapped_to_motors_numbers[3] as u8, Ordering::Relaxed);

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
        &INPUT_SHARED,
        &TELEMETRY_SHARED,
        controller_input_shared.peripherals,
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
        esp_pm_lock_acquire(esp_pm_lock_type_t_ESP_PM_CPU_FREQ_MAX as esp_pm_lock_handle_t);
        log::info!("Set core frequency");
    }

    let mut peripherals: Peripherals = Peripherals::take().unwrap();

    // start_telemetry_thread(&TELEMETRY_SHARED);

    let flight_task_name = CString::new("Flight Controller Task").unwrap();
    unsafe {
        xTaskCreatePinnedToCore(
            Some(flight_thread_task_entrypoint),
            flight_task_name.as_ptr(),
            4096,
            &FlightThreadInput {
                peripherals: &mut peripherals,
            } as *const _ as *mut c_void,
            1,
            std::ptr::null_mut(),
            1,
        )
    };

    match CONTROLLER_TYPE {
        ControllerTypes::Wifi => {
            let controller = WifiController::new();
            controller.start_changes_monitor(&INPUT_SHARED)
        }
        ControllerTypes::Ibus => {
            let controller = IBusController::new(&mut peripherals);
            controller.start_changes_monitor(&INPUT_SHARED)
        }
    };
}
