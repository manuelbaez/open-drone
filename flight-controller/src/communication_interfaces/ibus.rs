use super::controller::RemoteControl;
use crate::shared_core_values::{AtomicControllerInput, AtomicTelemetry};
use crate::util::time::get_current_system_time;
use esp_idf_svc::hal::delay::{BLOCK, TICK_PERIOD_MS};
use esp_idf_svc::hal::peripheral::Peripheral;
use esp_idf_svc::hal::prelude::*;
use esp_idf_svc::hal::{
    gpio,
    peripherals::Peripherals,
    uart::{config, UartDriver},
};
use shared_definitions::controller::ControllerInput;
use std::sync::atomic::Ordering;

const PROTOCOL_MESSAGE_TIMEGAP_US: i64 = 3500;
const PROTOCOL_SIZE: usize = 0x20;
const PROTOCOL_OVERHEAD: u8 = 3; // <len><cmd><data....><chkl><chkh>
const PROTOCOL_CHANNELS: usize = 10;
const CHANNEL_CENTER_POINT: u16 = 1500;
const CHANNEL_MAX_POINT: u16 = 2000;
const CHANNEL_MIN_POINT: u16 = 1000;
const IBUS_MESSAGE_SIZE: usize = PROTOCOL_SIZE + PROTOCOL_OVERHEAD as usize;

struct IbusCommands;
impl IbusCommands {
    pub const CHANNELS_DATA: u8 = 0x40; // Command is always 0x40
    pub const SENSORS_DISCOVER: u8 = 0x80; // Command discover sensor (lowest 4 bits are sensor)
    pub const POLL_SENSOR_TYPE: u8 = 0x90; // Command discover sensor type (lowest 4 bits are sensor)
    pub const POLL_SENSOR_VALUE: u8 = 0xA0; // Command send sensor data (lowest 4 bits are sensor)
    pub const SENSOR_DISCOVERED_RESPONSE_COMMAND: u8 = 0x04;
    pub const SENSOR_VALUE_RESPONSE_COMMAND: u8 = 0x04;
    pub const SENSOR_TYPE_RESPONSE_COMMAND: u8 = 0x06;
}
enum ReadingStages {
    Length,
    Data,
    GetChecksumLByte,
    GetChecksumHByte,
    Discard,
}

pub struct IbusTelemetrySensorsIds;
impl IbusTelemetrySensorsIds {
    pub const IBUS_SENSOR_TYPE_CELL: u8 = 0x04;
}

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

    fn map_data_to_channels(data: [u8; PROTOCOL_SIZE]) -> [u16; PROTOCOL_CHANNELS] {
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
        data: [u8; PROTOCOL_SIZE],
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
        buffer: [u8; PROTOCOL_SIZE + PROTOCOL_OVERHEAD as usize],
        shared_controller_input: &AtomicControllerInput,
    ) {
        let mut ibus_packet_parsing_buffer = [0_u8; PROTOCOL_SIZE];
        let current_message_length = buffer[0] as usize;
        //Copy message data to parsing buffer excluding the message length(Fist byte)
        ibus_packet_parsing_buffer[0..(current_message_length - 1)]
            .copy_from_slice(&buffer[1..current_message_length]);

        let checksum = (buffer[current_message_length - 1] as u16) << 8
            | buffer[current_message_length - 2] as u16;
        //Vaidate checksum, add all bytes in the receive buffer except for the last two
        let mut calculated_checksum = 0xffff_u16;
        for byte in buffer[0..(current_message_length - 2)].into_iter() {
            calculated_checksum -= byte.clone() as u16;
        }

        if checksum == calculated_checksum {
            let message_command = ibus_packet_parsing_buffer[0];
            match message_command {
                IbusCommands::CHANNELS_DATA => {
                    Self::process_ibus_channels_message(
                        ibus_packet_parsing_buffer,
                        shared_controller_input,
                    );
                }
                _ => (),
            }
        }

        // let checksum_value = buffer[0..(current_message_length - 3)]
        //     .into_iter()
        //     .map(|number| number as u32)
        //     .sum::<u32>();

        //     for current_byte in buffer {
        //         match state {
        //             ReadingStages::Length => {
        //                 current_byte_count = 0;
        //                 if current_byte <= PROTOCOL_SIZE as u8 && current_byte > PROTOCOL_OVERHEAD {
        //                     current_message_length = current_byte - PROTOCOL_OVERHEAD;
        //                     state = ReadingStages::Data;
        //                     target_checksum = 0xffff - current_byte as u32
        //                 } else {
        //                     state = ReadingStages::Discard;
        //                 }
        //             }
        //             ReadingStages::Data => {
        //                 if current_byte_count >= PROTOCOL_SIZE as u8 {
        //                     state = ReadingStages::Discard;
        //                     continue;
        //                 }
        //                 ibus_packet_parsing_buffer[current_byte_count as usize] = current_byte;
        //                 target_checksum -= current_byte as u32;
        //                 current_byte_count += 1;
        //                 if current_byte_count >= current_message_length {
        //                     state = ReadingStages::GetChecksumLByte;
        //                 }
        //             }
        //             ReadingStages::GetChecksumLByte => {
        //                 checksum = current_byte as u32;
        //                 state = ReadingStages::GetChecksumHByte
        //             }
        //             ReadingStages::GetChecksumHByte => {
        //                 checksum = ((current_byte as u16) << 8) as u32 + checksum as u32;
        //                 if target_checksum == checksum {
        //                     match ibus_packet_parsing_buffer[0] {
        //                         IbusCommands::CHANNELS_DATA => {
        //                             Self::process_ibus_channels_message(
        //                                 ibus_packet_parsing_buffer,
        //                                 shared_controller_input,
        //                             );
        //                         }
        //                         _ => {
        //                             let sensor_addr: u8 = ibus_packet_parsing_buffer[0] & 0x0f; //4 LS Bits of the read byte
        //                             if sensor_addr <= 1
        //                                 && sensor_addr > 0
        //                                 && current_message_length == 1
        //                             {
        //                                 // println!("Polled Sensor");
        //                                 let mapped_command = ibus_packet_parsing_buffer[0] & 0xf0; //4 MS Bits of the read byte
        //                                 match mapped_command {
        //                                     IbusCommands::SENSORS_DISCOVER => {
        //                                         println!("Polled sensor");
        //                                         self.uart_driver
        //                                                 .write(&[
        //                                                     IbusCommands::SENSOR_DISCOVERED_RESPONSE_COMMAND,
        //                                                     IbusCommands::SENSORS_DISCOVER + sensor_addr,
        //                                                 ])
        //                                                 .unwrap();
        //                                     }
        //                                     IbusCommands::POLL_SENSOR_TYPE => {
        //                                         self.uart_driver
        //                                             .write(&[
        //                                                 IbusCommands::SENSOR_TYPE_RESPONSE_COMMAND,
        //                                                 IbusCommands::POLL_SENSOR_TYPE + sensor_addr,
        //                                                 IbusTelemetrySensorsIds::IBUS_SENSOR_TYPE_CELL,
        //                                                 0x2, //Sensor data lenth
        //                                             ])
        //                                             .unwrap();
        //                                     }
        //                                     IbusCommands::POLL_SENSOR_VALUE => {
        //                                         self.uart_driver
        //                                                 .write(&[
        //                                                     IbusCommands::SENSOR_VALUE_RESPONSE_COMMAND + 0x2, //Sensor Data Length,
        //                                                     IbusCommands::POLL_SENSOR_VALUE + sensor_addr,
        //                                                     (35_u16 & 0x00ff) as u8, /*Sensor Value lower byte */
        //                                                     ((35_u16 >> 8) & 0x00ff) as u8, /*Sensor Value higher byte */
        //                                                 ])
        //                                                 .unwrap();
        //                                     }
        //                                     _ => (),
        //                                 }
        //                             }
        //                         }
        //                     };
        //                 }
        //                 state = ReadingStages::Discard;
        //             }
        //             ReadingStages::Discard => {
        //                 break;
        //             }
        //         }
        //     }
    }
}

impl<'a> RemoteControl for IBusController<'a> {
    fn start_input_changes_monitor(&self, shared_controller_input: &AtomicControllerInput) {
        let mut previous_time = get_current_system_time();
        loop {
            let mut read_buffer = [0_u8];
            self.uart_driver.read(&mut read_buffer, BLOCK).unwrap();

            let current_time = get_current_system_time();
            let elapsed_time = current_time - previous_time;
            previous_time = current_time;

            //Ignore any uart bytes that come befor the next message timeframe
            if elapsed_time < PROTOCOL_MESSAGE_TIMEGAP_US {
                continue;
            }

            let mut raw_message_buffer = [0_u8; PROTOCOL_SIZE + PROTOCOL_OVERHEAD as usize];
            let current_message_size: usize = read_buffer[0] as usize;
            let mut current_index: usize = 0;
            let mut buffer_processed = false;
            loop {
                if current_index < IBUS_MESSAGE_SIZE {
                    raw_message_buffer[current_index] = read_buffer[0];
                }
                //Message read completed process the frame buffer
                if !buffer_processed
                    && (current_index >= (IBUS_MESSAGE_SIZE - 1)
                        || current_index >= (current_message_size - 1))
                {
                    buffer_processed = true;
                    self.process_ibus_buffer(raw_message_buffer, shared_controller_input)
                }
                // Read until there is nothing to read then break the loop
                // and return back to validate for the protocol time gap
                let read_length = self.uart_driver.read(&mut read_buffer, 0).unwrap();
                if read_length <= 0 {
                    break;
                }
                current_index += 1;
            }
        }
    }
}
