use super::protocol::{
    IBusMessageParser, IbusCommands, CHANNEL_CENTER_POINT, CHANNEL_MAX_POINT, CHANNEL_MIN_POINT,
    PROTOCOL_CHANNELS, PROTOCOL_OVERHEAD, PROTOCOL_SIZE,
};
use crate::communication_interfaces::controller::RemoteControl;
use crate::shared_core_values::AtomicControllerInput;

use esp_idf_svc::hal::peripheral::Peripheral;
use esp_idf_svc::hal::prelude::*;
use esp_idf_svc::hal::{
    gpio,
    peripherals::Peripherals,
    uart::{config, UartDriver},
};
use shared_definitions::controller::ControllerInput;
use std::sync::atomic::Ordering;

#[repr(u8)]
enum ChannelMappings {
    Unused,
    Roll,
    Pitch,
    Yaw,
    Throttle,
    PowerOn,
    CalibrateSensors,
    CalibrateESC,
}

impl TryFrom<u8> for ChannelMappings {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(ChannelMappings::Roll),
            2 => Ok(ChannelMappings::Pitch),
            3 => Ok(ChannelMappings::Throttle),
            4 => Ok(ChannelMappings::Yaw),
            7 => Ok(ChannelMappings::PowerOn),
            8 => Ok(ChannelMappings::CalibrateSensors),
            10 => Ok(ChannelMappings::CalibrateESC),
            _ => Ok(ChannelMappings::Unused),
        }
    }
}

pub struct IBusController<'a> {
    uart_driver: UartDriver<'a>,
}

impl<'a> IBusController<'a> {
    pub fn new(peripherals: &mut Peripherals) -> Self {
        let tx = unsafe { peripherals.pins.gpio17.clone_unchecked() };
        let rx = unsafe { peripherals.pins.gpio16.clone_unchecked() };

        let uart2 = unsafe { peripherals.uart2.clone_unchecked() };
        let config = config::Config::new().baudrate(115_200.Hz());
        let uart_driver = UartDriver::new(
            uart2,
            tx,
            rx,
            Option::<gpio::AnyIOPin>::None,
            Option::<gpio::AnyIOPin>::None,
            &config,
        )
        .unwrap();
        Self { uart_driver }
    }

    fn map_data_to_channels(
        data: [u8; PROTOCOL_SIZE - PROTOCOL_OVERHEAD],
    ) -> [u16; PROTOCOL_CHANNELS] {
        let mut channels = [0_u16; PROTOCOL_CHANNELS];
        for index in 0..(PROTOCOL_CHANNELS * 2) {
            channels[index / 2] = (data[index] as u16) | ((data[index + 1] as u16) << 8);
        }
        channels
    }
    fn map_to_i16_range(channel_value: u16) -> i16 {
        ((channel_value as i16 - CHANNEL_CENTER_POINT as i16) as i64 * i16::max_value() as i64
            / (CHANNEL_CENTER_POINT - CHANNEL_MIN_POINT) as i64) as i16
    }

    fn map_to_u8_range(channel_value: u16) -> u8 {
        ((channel_value - CHANNEL_MIN_POINT) as u64 * u8::max_value() as u64
            / (CHANNEL_MAX_POINT - CHANNEL_MIN_POINT) as u64) as u8
    }

    fn map_to_boolean(channel_value: u16) -> bool {
        channel_value != CHANNEL_MIN_POINT
    }

    fn process_ibus_channels_message(
        data: [u8; PROTOCOL_SIZE - PROTOCOL_OVERHEAD],
        shared_controller_input: &AtomicControllerInput,
    ) {
        let channels = Self::map_data_to_channels(data);
        let mut input_values = ControllerInput::default();
        for (index, data) in channels.into_iter().enumerate() {
            let channel: ChannelMappings = ChannelMappings::try_from((index + 1) as u8).unwrap();
            match channel {
                ChannelMappings::Pitch => input_values.pitch = -Self::map_to_i16_range(data),
                ChannelMappings::Roll => input_values.roll = Self::map_to_i16_range(data),
                ChannelMappings::Yaw => input_values.yaw = Self::map_to_i16_range(data),
                ChannelMappings::Throttle => input_values.throttle = Self::map_to_u8_range(data),
                ChannelMappings::PowerOn => {
                    let value = Self::map_to_boolean(data);
                    input_values.start = value;
                    input_values.kill_motors = !value;
                }
                ChannelMappings::CalibrateSensors => {
                    input_values.calibrate_sensors = Self::map_to_boolean(data);
                }
                ChannelMappings::CalibrateESC => {
                    input_values.calibrate_esc = Self::map_to_boolean(data);
                }
                _ => (),
            }
        }

        shared_controller_input
            .roll
            .store(input_values.roll, Ordering::Relaxed);
        shared_controller_input
            .pitch
            .store(input_values.pitch, Ordering::Relaxed);
        shared_controller_input
            .yaw
            .store(input_values.yaw, Ordering::Relaxed);
        shared_controller_input
            .throttle
            .store(input_values.throttle, Ordering::Relaxed);
        shared_controller_input
            .kill_motors
            .store(input_values.kill_motors, Ordering::Relaxed);
        shared_controller_input
            .start
            .store(input_values.start, Ordering::Relaxed);
        shared_controller_input
            .calibrate_esc
            .store(input_values.calibrate_esc, Ordering::Relaxed);
        shared_controller_input
            .calibrate_sensors
            .store(input_values.calibrate_sensors, Ordering::Relaxed);
    }

    fn process_ibus_buffer(
        &self,
        message_buffer: [u8; PROTOCOL_SIZE - PROTOCOL_OVERHEAD],
        shared_controller_input: &AtomicControllerInput,
    ) {
        let message_command = message_buffer[0];
        match message_command {
            IbusCommands::CHANNELS_DATA => {
                Self::process_ibus_channels_message(message_buffer, shared_controller_input);
            }
            _ => (),
        }
    }
}

impl<'a> RemoteControl for IBusController<'a> {
    fn start_input_changes_monitor(&self, shared_controller_input: &AtomicControllerInput) {
        IBusMessageParser::start_monitor_on_uart(&self.uart_driver, |buffer, _| {
            self.process_ibus_buffer(buffer, shared_controller_input);
        });
    }
}
