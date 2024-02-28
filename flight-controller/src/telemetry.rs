use std::sync::atomic::Ordering;

use esp_idf_svc::hal::delay::FreeRtos;

use crate::shared_core_values::AtomicTelemetry;

pub fn start_telemetry_thread(telemetry_data: &'static AtomicTelemetry) {
    // Print telemetry values thread, for debugging/telemetry purposes, later will move this to it's own thread to send to controller.
    {
        let telemetry_data = telemetry_data;
        let _ = std::thread::Builder::new()
            .stack_size(4096)
            .spawn(move || loop {
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
            });
    }
}
