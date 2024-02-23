use std::sync::{Arc, RwLock};

use shared_definitions::controller::ControllerInput;

pub trait RemoteControl {
    fn start_changes_monitor(&self, shared_controller_input: Arc<RwLock<ControllerInput>>);
}
