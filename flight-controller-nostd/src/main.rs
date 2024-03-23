#![no_std]
#![no_main]
#![feature(const_float_bits_conv)]
#![feature(generic_const_exprs)]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::{
    clock::{ClockControl, Clocks},
    cpu_control::{CpuControl, Stack},
    embassy::{self, executor::Executor},
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
};

use static_cell::StaticCell;
use tasks::{controller_input_task, flight_controller_task, telemetry_task};

mod communication_interfaces;
mod config;
mod control;
mod drivers;
mod output;
mod shared_core_values;
mod tasks;
mod util;

static CLOCKS: StaticCell<Clocks> = StaticCell::new();
static mut APP_CORE_STACK: Stack<8192> = Stack::new();

#[main]
async fn main(spawner: Spawner) -> ! {
    // init_heap();
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();
    let clocks = CLOCKS.init(clocks);

    let timer_group0 = esp_hal::timer::TimerGroup::new(peripherals.TIMG0, clocks);
    embassy::init(clocks, timer_group0);
    // let timer_group1 = esp_hal::timer::TimerGroup::new(peripherals.TIMG1, clocks);
    // let mut wdt1 = timer_group1.wdt;
    // wdt1.disable();

    let mut cpu_control = CpuControl::new(system.cpu_control);

    // This line is for Wokwi only so that the console output is formatted correctly
    // print!("\x1b[20h");

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    spawner
        .spawn(flight_controller_task(
            peripherals.I2C0,
            io.pins.gpio21,
            io.pins.gpio22,
            peripherals.LEDC,
            io.pins.gpio13.into_push_pull_output(),
            io.pins.gpio12.into_push_pull_output(),
            io.pins.gpio14.into_push_pull_output(),
            io.pins.gpio27.into_push_pull_output(),
            clocks,
        ))
        .unwrap();

    let cpu1_fn = || {
        static CORE_1_EXECUTOR: StaticCell<Executor> = StaticCell::new();
        let executor = CORE_1_EXECUTOR.init(Executor::new());

        executor.run(|spawner| {
            spawner.spawn(telemetry_task()).unwrap();
            spawner
                .spawn(controller_input_task(
                    io.pins.gpio17,
                    io.pins.gpio16,
                    peripherals.UART2,
                    clocks,
                ))
                .unwrap();
        });
        // let mut delay = esp_hal::Delay::new(clocks);
    };
    let _guard = cpu_control
        .start_app_core(unsafe { &mut APP_CORE_STACK }, cpu1_fn)
        .unwrap();

    let mut count = 0;
    loop {
        esp_println::println!("Main Task Count: {} ", count);
        count += 1;
        Timer::after_millis(u32::MAX as u64).await;
    }
}

// #[entry]
// fn main() -> ! {
//     let peripherals = Peripherals::take();
//     let system = peripherals.SYSTEM.split();

//     let clocks = ClockControl::max(system.clock_control).freeze();
//     // let mut delay = Delay::new(&clocks);
//     esp_println::logger::init_logger_from_env();

//     static CORE_0_EXECUTOR: StaticCell<Executor> = StaticCell::new();

//     let core_0_executor = CORE_0_EXECUTOR.init(Executor::new());
//     let timer_group = TimerGroup::new(peripherals.TIMG0, &clocks);
//     embassy::init(&clocks, timer_group);

//     let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
//     let clocks = CLOCKS.init(clocks);

//     core_0_executor.run(|spawner| {
//         spawner.spawn(telemetry_task()).unwrap();
//         spawner
//             .spawn(controller_input_task(
//                 io.pins.gpio17,
//                 io.pins.gpio16,
//                 peripherals.UART2,
//                 clocks,
//             ))
//             .unwrap();
//     });

//     // // println!("Hello world!");
//     // loop {
//     //     // println!("Loop...");
//     //     delay.delay_ms(500u32);
//     // }
// }
