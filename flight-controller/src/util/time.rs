use esp_idf_svc::sys::esp_timer_get_time;

pub fn get_current_system_time() -> i64 {
    unsafe { esp_timer_get_time() }
}
