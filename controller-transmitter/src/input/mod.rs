use core::slice;
use std::{
    fs::OpenOptions,
    io::Read,
    mem,
    ops::Deref,
    result,
    sync::{Arc, Mutex},
};

use self::controller::{AnalogInputCodes, ButtonPressCodes, EventTypes, InputEvent};

pub mod controller;

const CONTROLLER_THROTTLE_MAX_VALUE: u32 = 1024;
const STICKS_DEAD_ZONES: i32 = 1800;

#[derive(Clone)]
pub struct DroneControllerInput {
    pub roll: i16,
    pub pitch: i16,
    pub yaw: i16,
    pub throttle: u8,
    pub kill_motors: bool,
    pub start_motors: bool,
}

#[derive(Clone)]
pub struct ControlInputMapper {
    drone_controls: Arc<Mutex<DroneControllerInput>>,
    device_path: Arc<&'static str>,
}

impl ControlInputMapper {
    pub fn new(device_path: &'static str) -> Self {
        ControlInputMapper {
            drone_controls: Arc::new(Mutex::new(DroneControllerInput {
                roll: 0,
                pitch: 0,
                yaw: 0,
                throttle: 0,
                start_motors: false,
                kill_motors: false,
            })),
            device_path: Arc::new(device_path),
        }
    }

    fn map_event_to_action(&mut self, event: InputEvent) {
        match event.event_type {
            EventTypes::ButtonPress => {
                let event_code = ButtonPressCodes::try_from(event.code);
                if !event_code.is_err() {
                    self.handle_button_press(event_code.unwrap(), event.value);
                }
            }
            EventTypes::AnalogInput => {
                let event_code = AnalogInputCodes::try_from(event.code);
                if !event_code.is_err() {
                    self.handle_analog_input(event_code.unwrap(), event.value)
                }
            }
            _default => (),
        }
    }

    fn filter_dead_zones(value: i16) -> i16 {
        if (value as i32).abs() > STICKS_DEAD_ZONES {
            return value;
        }
        return 0;
    }
    fn handle_analog_input(&mut self, event_code: AnalogInputCodes, value: i32) {
        match event_code {
            AnalogInputCodes::LT => {
                let mut lock = self.drone_controls.lock().unwrap();
                lock.throttle = ((value as u32 * 256) / CONTROLLER_THROTTLE_MAX_VALUE) as u8;
                drop(lock);
            }
            AnalogInputCodes::RightX => {
                let mut lock = self.drone_controls.lock().unwrap();
                lock.roll = Self::filter_dead_zones(value as i16);
                drop(lock);
            }
            AnalogInputCodes::RightY => {
                let mut lock = self.drone_controls.lock().unwrap();
                lock.pitch = Self::filter_dead_zones(value as i16);
                drop(lock);
            }
            AnalogInputCodes::LeftX => {
                let mut lock = self.drone_controls.lock().unwrap();
                lock.yaw = Self::filter_dead_zones(value as i16);
                drop(lock);
            }
            _default => {}
        }
    }

    fn handle_button_press(&mut self, event_code: ButtonPressCodes, value: i32) {
        match event_code {
            ButtonPressCodes::Y => {
                let mut lock = self.drone_controls.lock().unwrap();
                lock.start_motors = value != 0;
                drop(lock);
            }
            ButtonPressCodes::A => {
                let mut lock = self.drone_controls.lock().unwrap();
                lock.kill_motors = value != 0;
                drop(lock);
            }
            _default => (),
        }
    }

    pub fn get_current_input(&self) -> DroneControllerInput {
        let input_lock = &self.drone_controls.lock().unwrap();
        let current_values = input_lock.deref().clone();
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
        let mut dev_file = file_options.open(self.device_path.deref()).unwrap();

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
