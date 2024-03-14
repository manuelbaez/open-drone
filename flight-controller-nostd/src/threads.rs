use core::sync::atomic::Ordering;

use embassy_time::Timer;
use esp_hal::{
    clock::Clocks,
    gpio::{self, Gpio12, Gpio13, Gpio14, Gpio16, Gpio17, Gpio21, Gpio22, Gpio27, Unknown},
    i2c::I2C,
    ledc::{self, timer, LSGlobalClkSource, LEDC},
    peripherals::{self, I2C0, UART2},
    prelude::*,
    uart,
};

use crate::{
    communication_interfaces::ibus::{controller::IBusController, protocol::IBusUartMonitor},
    config::{
        constants::{ESC_PWM_FREQUENCY_HZ, MAX_MOTOR_POWER, MIN_POWER, VEHICLE_TYPE},
        store::{AppStoredConfig, ConfigStorage},
    },
    control::control_loops::{start_flight_controllers, MainControlLoopOutCommands},
    drivers::mpu_6050::{device::MPU6050Sensor, registers::LowPassFrequencyValues},
    output::{
        motors_state_manager::QuadcopterMotorsStateManager,
        vehicle_movement_mappers::{FlyingVehicleMovementMapper, Quadcopter, VehicleTypesMapper},
    },
    shared_core_values::{AtomicControllerInput, SHARED_CONTROLLER_INPUT, SHARED_TELEMETRY},
};

// const VOLTAGE_DIVIDER_MULTIPLIER: f32 = 9.648; // 999k + 119k
// const ADC_1_VOLT: u16 = 672_u16;
// const MULTPLIER: f32 = (1.0 / ADC_1_VOLT as f32) * VOLTAGE_DIVIDER_MULTIPLIER;

#[embassy_executor::task]
pub async fn flight_thread(
    i2c_dev: I2C0,
    sda: Gpio21<Unknown>,
    scl: Gpio22<Unknown>,
    ledc_peripheral: peripherals::LEDC,
    motor_1_pin: Gpio13<gpio::Output<gpio::PushPull>>,
    motor_2_pin: Gpio12<gpio::Output<gpio::PushPull>>,
    motor_3_pin: Gpio14<gpio::Output<gpio::PushPull>>,
    motor_4_pin: Gpio27<gpio::Output<gpio::PushPull>>,
    clocks: &'static Clocks<'static>,
) -> ! {
    let controller_input_shared = &SHARED_CONTROLLER_INPUT;
    let telemetry_shared = &SHARED_TELEMETRY;

    let mut ledc_driver = LEDC::new(ledc_peripheral, clocks);
    ledc_driver.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let mut timer = ledc_driver.get_timer::<ledc::HighSpeed>(timer::Number::Timer0);
    timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty14Bit,
            clock_source: timer::HSClockSource::APBClk,
            frequency: ESC_PWM_FREQUENCY_HZ.Hz(),
        })
        .unwrap();

    let mut motors_manager = QuadcopterMotorsStateManager::new_with_pins(
        &ledc_driver,
        &timer,
        motor_1_pin,
        motor_2_pin,
        motor_3_pin,
        motor_4_pin,
    );

    let mut config_store = ConfigStorage::new();
    let app_config = AppStoredConfig::default();
    let accelerometer_calibration = app_config.accelerometer_calibration.clone();
    let gyro_calibration = app_config.gyro_calibration.clone();

    let mut i2c_driver = I2C::new(i2c_dev, sda, scl, 400_u32.kHz(), clocks);
    let mut imu = MPU6050Sensor::new(&mut i2c_driver, gyro_calibration, accelerometer_calibration);
    imu.enable_low_pass_filter(LowPassFrequencyValues::Freq10Hz);
    imu.init().await;

    let output_handler = match VEHICLE_TYPE {
        VehicleTypesMapper::Quadcopter => {
            let quadcoper_movement_mapper = Quadcopter::new(MIN_POWER, MAX_MOTOR_POWER);
            move |result| match result {
                MainControlLoopOutCommands::KillMotors => motors_manager.kill_motors(),
                MainControlLoopOutCommands::BypassThrottle(throttle) => {
                    motors_manager.set_motor_power([throttle; 4]);
                }
                MainControlLoopOutCommands::StoreSensorsCalibration(accelerometer, gyro) => {
                    // let mut config_lock = APP_CONFIG_STORE.lock().unwrap();
                    let config = AppStoredConfig {
                        initialized: true,
                        accelerometer_calibration: accelerometer,
                        gyro_calibration: gyro,
                    };
                    let result = config_store.store_to_flash(config);
                    if result.is_err() {
                        match result.err() {
                            Some(e) => {
                                esp_println::println!("{:?}", e)
                            }
                            None => (),
                        };
                    } else {
                        result.unwrap();
                    }
                    // drop(config_lock);
                }
                MainControlLoopOutCommands::UpdateFlightState(state) => {
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
    )
    .await
}

#[embassy_executor::task]
pub async fn telemetry_thread() -> ! {
    let telemetry_data = &SHARED_TELEMETRY;
    loop {
        esp_println::println!(
            "
                Iteration Time: {:?}
                Rotation {:?}
                Motor {:?}
                Throttle {:?}
                Input Start {}",
            telemetry_data.loop_exec_time_us.load(Ordering::Relaxed),
            telemetry_data.rotation_rate.read(),
            [
                telemetry_data.motor_1_power.load(Ordering::Relaxed),
                telemetry_data.motor_2_power.load(Ordering::Relaxed),
                telemetry_data.motor_3_power.load(Ordering::Relaxed),
                telemetry_data.motor_4_power.load(Ordering::Relaxed)
            ],
            telemetry_data.throttle.load(Ordering::Relaxed),
            SHARED_CONTROLLER_INPUT.start.load(Ordering::Relaxed)
        );
        Timer::after_millis(1000).await;
    }
}

#[embassy_executor::task]
pub async fn controller_input_task(
    tx: Gpio17<Unknown>,
    rx: Gpio16<Unknown>,
    uart: UART2,
    clocks: &'static Clocks<'static>,
) -> ! {
    let mut ibus_controller = IBusController::new(tx, rx, uart, &SHARED_CONTROLLER_INPUT, clocks);
    ibus_controller.start_monitor_on_uart().await
}

// pub fn measurements_thread() {
//     let mut peripherals_lock = SHARED_PERIPHERALS.lock().unwrap();
//     let adc = unsafe { peripherals_lock.adc1.clone_unchecked() };
//     let pin = unsafe { peripherals_lock.pins.gpio34.clone_unchecked() };
//     drop(peripherals_lock);

//     let mut adc_pin =
//         AdcChannelDriver::<'_, adc_atten_t_ADC_ATTEN_DB_11, Gpio34>::new(pin).unwrap();

//     let mut adc_driver = AdcDriver::new(adc, &Config::new()).unwrap();

//     loop {
//         // Do multiple spaced out samples to get more stable reads,
//         // apparently this is bad for the flight thread performance, seems to be allocated on core 1
//         // for _ in [0; 4] {
//         //     sample += adc_driver.read(&mut adc_pin).unwrap();
//         //     // FreeRtos::delay_ms(100);
//         // }
//         // sample /= 4;

//         let sample: u16 = adc_driver.read(&mut adc_pin).unwrap();
//         let voltage = sample as f32 * MULTPLIER;
//         SHARED_TELEMETRY
//             .battery_voltage
//             .store(voltage, Ordering::Relaxed);
//         FreeRtos::delay_ms(2000);
//     }
// }
