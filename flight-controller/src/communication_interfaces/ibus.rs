use std::sync::{Arc, RwLock};

use shared_definitions::controller::ControllerInput;

use super::controller::RemoteControl;

pub struct IBusController;

impl IBusController {
    pub fn new() -> Self {
        Self
    }
}

impl RemoteControl for IBusController {
    fn start_changes_monitor(&self, shared_controller_input: Arc<RwLock<ControllerInput>>) {}
}
