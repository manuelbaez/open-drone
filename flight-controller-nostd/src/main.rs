#![no_std]
#![no_main]
#![feature(const_float_bits_conv)]
#![feature(generic_const_exprs)]

use config::constants::ESC_PWM_FREQUENCY_HZ;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp32_hal::{
    clock::ClockControl,
    embassy,
    ledc::{self, timer, LSGlobalClkSource, LEDC},
    peripheral::Peripheral,
    peripherals::Peripherals,
    prelude::*,
    uart, Uart, IO,
};
use esp_backtrace as _;
use output::motors_state_manager::QuadcopterMotorsStateManager;

mod communication_interfaces;
mod config;
mod drivers;
mod output;
mod shared_core_values;
mod util;
mod control;

#[main]
async fn main(spawner: Spawner) {
    let mut peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();

    let timer = esp32_hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timer);

    // spawner.spawn(one_second_task()).unwrap();

    // This line is for Wokwi only so that the console output is formatted correctly
    // print!("\x1b[20h");

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let ledc_peripheral = unsafe { peripherals.LEDC.clone_unchecked() };
    let mut ledc_driver = LEDC::new(ledc_peripheral, &clocks);
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
        io.pins.gpio13.into_push_pull_output(),
        io.pins.gpio12.into_push_pull_output(),
        io.pins.gpio14.into_push_pull_output(),
        io.pins.gpio27.into_push_pull_output(),
    );

    //uart sample code

    let mut count = 0;
    loop {
        esp_println::println!("Main Task Count: {}", count);
        count += 1;
        Timer::after(Duration::from_millis(500)).await;
        motors_manager.set_motor_power([0.0_f32; 4]);
        Timer::after(Duration::from_millis(500)).await;
        motors_manager.set_motor_power([100.0_f32; 4]);
    }
}

// #[entry]
// fn main() -> ! {
//     let peripherals = Peripherals::take();
//     let system = peripherals.SYSTEM.split();

//     let clocks = ClockControl::max(system.clock_control).freeze();
//     let mut delay = Delay::new
//     (&clocks);

//     println!("Hello world!");
//     loop {
//         println!("Loop...");
//         delay.delay_ms(500u32);
//     }
// }
