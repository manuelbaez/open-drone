use crate::shared_core_values::{AtomicControllerInput, AtomicTelemetry};

pub trait RemoteControl {
    fn start_input_changes_monitor(&mut self, shared_controller_input: &AtomicControllerInput);
}

pub trait RemoteTelemetry{
    fn start_telemetry_tx_loop(&mut self, shared_telemetry: &AtomicTelemetry);
}