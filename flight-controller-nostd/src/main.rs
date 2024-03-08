#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp32_hal::{clock::ClockControl, embassy, peripherals::Peripherals, prelude::*};
use esp_backtrace as _;
// mod drivers;
mod util;

#[main]
async fn main(spawner: Spawner) {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();

    let timer = esp32_hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timer);

    // spawner.spawn(one_second_task()).unwrap();

    // This line is for Wokwi only so that the console output is formatted correctly
    // print!("\x1b[20h");

    let mut count = 0;
    loop {
        esp_println::println!("Main Task Count: {}", count);
        count += 1;
        Timer::after(Duration::from_millis(5_000)).await;
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
