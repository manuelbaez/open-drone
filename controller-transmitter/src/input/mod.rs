use core::slice;
use std::{
    fs::OpenOptions,
    io::Read,
    mem,
    ops::Deref,
    sync::{Arc, Mutex},
};

use self::controller::{AnalogInputCodes, EventTypes, InputEvent};

pub mod controller;

#[derive(Clone)]
pub struct DroneMovementInput {
    pub roll: i32,
    pub pitch: i32,
    pub yaw: i32,
    pub throttle: u16,
}

#[derive(Clone)]
pub struct ControlInputMapper {
    drone_controls: Arc<Mutex<DroneMovementInput>>,
    // device_path: &'a str,
}

impl ControlInputMapper {
    pub fn new(device_path: &str) -> Self {
        ControlInputMapper {
            drone_controls: Arc::new(Mutex::new(DroneMovementInput {
                roll: 0,
                pitch: 0,
                yaw: 0,
                throttle: 0,
            })),
            // device_path,
        }
    }

    fn map_event_to_action(&mut self, event: InputEvent) {
        match event.event_type {
            EventTypes::ButtonPress => {
                // println!("Button Pressed {}- {}", event.code, event.value);
            }
            EventTypes::AnalogInput => {
                // println!("Analog{}- {}", event.code, event.value);
                let event_code = AnalogInputCodes::try_from(event.code);
                if !event_code.is_err() {
                    self.handle_analog_input(event_code.unwrap(), event.value)
                }
            }
            _default => {
                // println!("Something Pressed {}", _default as u16);
            }
        }
    }

    fn handle_analog_input(&mut self, event_code: AnalogInputCodes, value: i32) {
        match event_code {
            AnalogInputCodes::LT => {
                let mut lock = self.drone_controls.lock().unwrap();
                lock.throttle = value as u16;
                drop(lock);
            }
            AnalogInputCodes::RightX => {
                let mut lock = self.drone_controls.lock().unwrap();
                lock.roll = value;
                drop(lock);
            }
            AnalogInputCodes::RightY => {
                let mut lock = self.drone_controls.lock().unwrap();
                lock.pitch = value;
                drop(lock);
            }
            _default => {}
        }
    }

    pub fn get_current_input(&self) -> DroneMovementInput {
        let input_lock = &self.drone_controls.lock().unwrap();
        let current_values = input_lock.deref().clone();
        drop(input_lock);
        current_values
    }

    pub fn start_event_handler_thread(&self) {
        let mut cloned = self.clone();
        let _ = std::thread::Builder::new()
            .stack_size(4096)
            .spawn(move || cloned.event_thread_fn());
    }

    fn event_thread_fn(&mut self) {
        let mut file_options = OpenOptions::new();
        file_options.read(true);
        file_options.write(false);
        let mut dev_file = file_options.open("/dev/input/event23").unwrap();

        loop {
            let struct_size = mem::size_of::<InputEvent>();
            let event: InputEvent = unsafe { mem::zeroed() };
            let mut event_buffer =
                unsafe { slice::from_raw_parts_mut(&event as *const _ as *mut u8, struct_size) };
            dev_file.read(&mut event_buffer).unwrap();
            self.map_event_to_action(event)
        }
    }
}
