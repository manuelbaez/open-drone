#![no_std]
#![no_main]
#![feature(const_float_bits_conv)]
#![feature(generic_const_exprs)]

use communication_interfaces::ibus::{controller::IBusController, protocol::IbusUartMonitor};
use embassy_executor::Spawner;
use embassy_time::Timer;
use esp32_hal::{
    clock::{ClockControl, Clocks},
    cpu_control::{CpuControl, Stack},
    embassy::{self, executor::Executor},
    peripherals::Peripherals,
    prelude::*,
    IO,
};
use esp_backtrace as _;
use shared_core_values::SHARED_CONTROLLER_INPUT;
use static_cell::StaticCell;
use threads::{flight_thread, input_thread, telemetry_thread};

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
static mut CORE1_STACK: Stack<8192> = Stack::new();
#[main]
async fn main(spawner: Spawner) -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();
    let clocks = CLOCKS.init(clocks);

    let timer = esp32_hal::timer::TimerGroup::new(peripherals.TIMG0, clocks);
    embassy::init(clocks, timer);

    let mut cpu_control = CpuControl::new(system.cpu_control);

    // This line is for Wokwi only so that the console output is formatted correctly
    // print!("\x1b[20h");

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    spawner
        .spawn(input_thread(
            io.pins.gpio17,
            io.pins.gpio16,
            peripherals.UART2,
            clocks,
        ))
        .unwrap();
    spawner.spawn(telemetry_thread()).unwrap();

    let cpu1_fn = || {
        let executor = CORE1_EXECUTOR.init(Executor::new());
        executor.run(|spawner| {
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
        });
    };
    let _guard = cpu_control
        .start_app_core(unsafe { &mut CORE1_STACK }, cpu1_fn)
        .unwrap();

    let mut count = 0;
    loop {
        esp_println::println!("Main Task Count: {} ", count);
        count += 1;
        Timer::after_millis(1000).await;
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
