use crate::shared_core_values::AtomicControllerInput;

pub trait RemoteControl {
    fn start_changes_monitor(&self, shared_controller_input: &AtomicControllerInput);
}
