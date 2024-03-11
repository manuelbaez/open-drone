use embassy_time::Instant;

pub fn get_current_system_time_us() -> u64 {
    Instant::now().as_micros()
}
