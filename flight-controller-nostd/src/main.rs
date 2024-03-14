#![no_std]
#![no_main]
#![feature(const_float_bits_conv)]
#![feature(generic_const_exprs)]

use communication_interfaces::ibus::{controller::IBusController, protocol::IBusUartMonitor};
use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::{
    clock::{ClockControl, Clocks},
    cpu_control::{CpuControl, Stack},
    embassy::{
        self,
        executor::{self, Executor},
    },
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Delay, IO,
};
use static_cell::StaticCell;
use threads::{controller_input_task, flight_thread, telemetry_thread};

mod communication_interfaces;
mod config;
mod control;
mod drivers;
mod output;
mod shared_core_values;
mod threads;
mod util;

static CLOCKS: StaticCell<Clocks> = StaticCell::new();
static CORE1_EXECUTOR: StaticCell<Executor> = StaticCell::new();
static mut CORE1_STACK: Stack<16384> = Stack::new();
#[main]
async fn main(spawner: Spawner) -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();
    let clocks = CLOCKS.init(clocks);

    let timer = esp_hal::timer::TimerGroup::new(peripherals.TIMG0, clocks);
    embassy::init(clocks, timer);

    let mut cpu_control = CpuControl::new(system.cpu_control);

    // This line is for Wokwi only so that the console output is formatted correctly
    // print!("\x1b[20h");

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    spawner
        .spawn(flight_thread(
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

    spawner
        .spawn(controller_input_task(
            io.pins.gpio17,
            io.pins.gpio16,
            peripherals.UART2,
            clocks,
        ))
        .unwrap();
    
    let cpu1_fn = || {
        let executor = CORE1_EXECUTOR.init(Executor::new());
        executor.run(|spawner| {
            spawner.spawn(telemetry_thread()).unwrap();
        });
    };
    let _guard = cpu_control
        .start_app_core(unsafe { &mut CORE1_STACK }, cpu1_fn)
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
//     let mut delay = Delay::new(&clocks);
//     esp_println::logger::init_logger_from_env();

//     static CORE_0_EXECUTOR: StaticCell<Executor> = StaticCell::new();

//     let core_0_executor = CORE_0_EXECUTOR.init(Executor::new());
//     let timer_group = TimerGroup::new(peripherals.TIMG0, &clocks);
//     embassy::init(&clocks, timer_group);

//     core_0_executor.run(|spawner| {

//     });

//     // println!("Hello world!");
//     loop {
//         // println!("Loop...");
//         delay.delay_ms(500u32);
//     }
// }
