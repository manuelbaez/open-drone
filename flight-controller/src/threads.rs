use std::{ops::DerefMut, sync::atomic::Ordering};

use esp_idf_svc::{
    hal::{
        adc::{config::Config, AdcChannelDriver, AdcDriver},
        delay::FreeRtos,
        gpio::Gpio34,
        peripheral::Peripheral,
    },
    sys::adc_atten_t_ADC_ATTEN_DB_11,
};

use crate::{
    config::constants::{
        ACCEL_X_DEVIATION, ACCEL_Y_DEVIATION, ACCEL_Z_DEVIATION, GYRO_PITCH_CALIBRATION_DEG,
        GYRO_ROLL_CALIBRATION_DEG, GYRO_YAW_CALIBRATION_DEG, MAX_MOTOR_POWER, MIN_POWER,
        VEHICLE_TYPE,
    },
    control::control_loops::{start_flight_controllers, FlightStabilizerOutCommands},
    drivers::mpu_6050::{device::MPU6050Sensor, registers::LowPassFrequencyValues},
    get_i2c_driver,
    output::{
        motors_state_manager::QuadcopterMotorsStateManager,
        vehicle_movement_mappers::{FlyingVehicleMovementMapper, Quadcopter, VehicleTypesMapper},
    },
    shared_core_values::{SHARED_CONTROLLER_INPUT, SHARED_TELEMETRY},
    util::math::vectors::{AccelerationVector3D, RotationVector3D},
    SHARED_PERIPHERALS,
};

const VOLTAGE_DIVIDER_MULTIPLIER: f32 = 9.648; // 999k + 119k
const ADC_1_VOLT: u16 = 672_u16;
const MULTPLIER: f32 = (1.0 / ADC_1_VOLT as f32) * VOLTAGE_DIVIDER_MULTIPLIER;

pub fn flight_thread() {
    let controller_input_shared = &SHARED_CONTROLLER_INPUT;
    let telemetry_shared = &SHARED_TELEMETRY;
    let peripherals_shared = &SHARED_PERIPHERALS;

    let mut peripherals_lock = peripherals_shared.lock().unwrap();
    let mut i2c_driver = get_i2c_driver(peripherals_lock.deref_mut());
    FreeRtos::delay_ms(50); //Wait for i2c to initialize
    let mut motors_manager = QuadcopterMotorsStateManager::new(peripherals_lock.deref_mut());
    drop(peripherals_lock);

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

    let mut imu = MPU6050Sensor::new(&mut i2c_driver, gyro_calibration, acceleromenter_calibration);
    imu.enable_low_pass_filter(LowPassFrequencyValues::Freq10Hz);
    imu.init();

    let output_handler = match VEHICLE_TYPE {
        VehicleTypesMapper::Quadcopter => {
            let quadcoper_movement_mapper = Quadcopter::new(MIN_POWER, MAX_MOTOR_POWER);
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
                        quad_out.motor_front_right,
                        quad_out.motor_front_left,
                        quad_out.motor_rear_right,
                        quad_out.motor_rear_left,
                    ];

                    telemetry_shared
                        .motor_1_power
                        .store(mapped_to_motors_numbers[0] as u8, Ordering::Relaxed);
                    telemetry_shared
                        .motor_2_power
                        .store(mapped_to_motors_numbers[1] as u8, Ordering::Relaxed);
                    telemetry_shared
                        .motor_3_power
                        .store(mapped_to_motors_numbers[2] as u8, Ordering::Relaxed);
                    telemetry_shared
                        .motor_4_power
                        .store(mapped_to_motors_numbers[3] as u8, Ordering::Relaxed);

                    motors_manager.set_motor_power(mapped_to_motors_numbers);
                }
            }
        }
    };

    start_flight_controllers(
        controller_input_shared,
        imu,
        telemetry_shared,
        output_handler,
    );
}

pub fn telemetry_thread() {
    let telemetry_data = &SHARED_TELEMETRY;
    loop {
        log::info!(
            " 
                Iteration Time: {:?}
                Rotation rate {:?}
                Motor {:?}
                Throttle {:?}",
            telemetry_data.loop_exec_time_us.load(Ordering::Relaxed),
            telemetry_data.rotation_rate.read(),
            [
                telemetry_data.motor_1_power.load(Ordering::Relaxed),
                telemetry_data.motor_2_power.load(Ordering::Relaxed),
                telemetry_data.motor_3_power.load(Ordering::Relaxed),
                telemetry_data.motor_4_power.load(Ordering::Relaxed)
            ],
            telemetry_data.throttle.load(Ordering::Relaxed),
        );
        FreeRtos::delay_ms(250);
    }
}

pub fn measurements_thread() {
    let mut peripherals_lock = SHARED_PERIPHERALS.lock().unwrap();
    let adc = unsafe { peripherals_lock.adc1.clone_unchecked() };
    let pin = unsafe { peripherals_lock.pins.gpio34.clone_unchecked() };
    drop(peripherals_lock);

    let mut adc_pin =
        AdcChannelDriver::<'_, adc_atten_t_ADC_ATTEN_DB_11, Gpio34>::new(pin).unwrap();

    let mut adc_driver = AdcDriver::new(adc, &Config::new()).unwrap();

    loop {
        // Do multiple spaced out samples to get more stable reads,
        // apparently this is bad for the flight thread performance, seems to be allocated on core 1
        // for _ in [0; 4] {
        //     sample += adc_driver.read(&mut adc_pin).unwrap();
        //     // FreeRtos::delay_ms(100);
        // }
        // sample /= 4;

        let sample: u16 = adc_driver.read(&mut adc_pin).unwrap();
        let voltage = sample as f32 * MULTPLIER;
        SHARED_TELEMETRY
            .battery_voltage
            .store(voltage, Ordering::Relaxed);
        FreeRtos::delay_ms(2000);
    }
}
