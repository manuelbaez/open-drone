use esp_idf_svc::hal::{delay::BLOCK, uart::UartDriver};

use crate::util::time::get_current_system_time;

pub const PROTOCOL_MESSAGE_TIMEGAP_US: i64 = 3500;
pub const PROTOCOL_SIZE: usize = 0x20; // The max message length is 32 bytes
pub const PROTOCOL_OVERHEAD: usize = 3; // <len><cmd><data....><chkl><chkh> this one accounts for len, and two checksum bytes
pub const PROTOCOL_CHANNELS: usize = 10;
pub const CHANNEL_CENTER_POINT: u16 = 1500;
pub const CHANNEL_MAX_POINT: u16 = 2000;
pub const CHANNEL_MIN_POINT: u16 = 1000;

pub struct IbusCommands;
impl IbusCommands {
    pub const CHANNELS_DATA: u8 = 0x40; // Data command is always 0x40
    pub const SENSORS_DISCOVER: u8 = 0x80; // Command discover sensor (lowest 4 bits are the sensor number)
    pub const POLL_SENSOR_TYPE: u8 = 0x90; // Command discover sensor type (lowest 4 bits are the sensor number)
    pub const MEASUREMENT: u8 = 0xA0; // Command send sensor data (lowest 4 bits are sensor)
}

pub struct IbusTelemetrySensorsIds;
#[allow(dead_code)]
impl IbusTelemetrySensorsIds {
    pub const AVG_CELL_VOLTAGE: u8 = 0x04;
    pub const EXTERNAL_VOLTAGE: u8 = 0x03;
}

pub struct IBusMessageParser;
impl IBusMessageParser {
    fn calculate_message_checksum(buffer: &[u8]) -> u16 {
        let mut calculated_checksum = 0xffff_u16;
        for byte in buffer.into_iter() {
            calculated_checksum -= byte.clone() as u16;
        }
        calculated_checksum
    }

    pub fn extract_and_validate_message_content(
        raw_message_buffer: [u8; PROTOCOL_SIZE],
    ) -> Result<([u8; PROTOCOL_SIZE - PROTOCOL_OVERHEAD], usize), ()> {
        if raw_message_buffer[0] < PROTOCOL_OVERHEAD as u8
            || raw_message_buffer[0] > PROTOCOL_SIZE as u8
        {
            return Err(());
        }
        let mut ibus_packet_parsing_buffer = [0_u8; PROTOCOL_SIZE - PROTOCOL_OVERHEAD];
        let current_message_length = raw_message_buffer[0] as usize - PROTOCOL_OVERHEAD;
        //Copy message data to parsing buffer excluding the message length(Fist byte).
        //Since the length and checksum are subtracted of the message length we just need
        //to shift the slice one position to the right and that will get the whole message data
        //without the checksum
        ibus_packet_parsing_buffer[0..(current_message_length)]
            .copy_from_slice(&raw_message_buffer[1..current_message_length + 1]);

        //Get the received message checksum, since the PROTOCOL_OVERHEAD was subtracted
        //it's added back to get the message end
        let checksum = (raw_message_buffer[current_message_length + PROTOCOL_OVERHEAD - 1] as u16)
            << 8
            | raw_message_buffer[current_message_length + PROTOCOL_OVERHEAD - 2] as u16;

        //Vaidate checksum, add all bytes in the receive buffer except for the last two
        let calculated_checksum = Self::calculate_message_checksum(
            &raw_message_buffer[0..(current_message_length + PROTOCOL_OVERHEAD - 2)],
        );
        if checksum == calculated_checksum {
            return Ok((ibus_packet_parsing_buffer, current_message_length));
        }
        Err(())
    }

    pub fn format_ibus_message<const SIZE: usize>(
        payload: [u8; SIZE],
    ) -> [u8; SIZE + PROTOCOL_OVERHEAD] {
        let mut tx_buffer = [0; SIZE + PROTOCOL_OVERHEAD];
        tx_buffer[0] = (SIZE + PROTOCOL_OVERHEAD) as u8;

        tx_buffer[1..(SIZE + 1)].copy_from_slice(&payload);

        //Add checksum
        let checksum = Self::calculate_message_checksum(&tx_buffer[0..(SIZE + 1)]);
        tx_buffer[(SIZE + 1)..(SIZE + PROTOCOL_OVERHEAD)]
            .copy_from_slice(&[(checksum & 0xff) as u8, ((checksum >> 8) & 0xff) as u8]);

        tx_buffer
    }
    pub fn start_monitor_on_uart(
        uart_driver: &UartDriver<'_>,
        mut message_callback: impl FnMut([u8; PROTOCOL_SIZE - PROTOCOL_OVERHEAD], usize) -> (),
    ) {
        let mut previous_time = get_current_system_time();
        loop {
            let mut read_buffer = [0_u8];
            uart_driver.read(&mut read_buffer, BLOCK).unwrap();

            let current_time = get_current_system_time();
            let elapsed_time = current_time - previous_time;
            previous_time = current_time;

            //Ignore any uart bytes that come befor the next message timeframe
            if elapsed_time < PROTOCOL_MESSAGE_TIMEGAP_US {
                continue;
            }

            let mut raw_message_buffer = [0_u8; PROTOCOL_SIZE];
            let current_message_size: usize = read_buffer[0] as usize;
            let mut current_index: usize = 0;
            let mut buffer_processed = false;
            loop {
                if current_index < PROTOCOL_SIZE {
                    raw_message_buffer[current_index] = read_buffer[0];
                }
                //Message read completed process the message buffer
                if !buffer_processed
                    && (current_index >= (PROTOCOL_SIZE - 1)
                        || current_index >= (current_message_size - 1))
                {
                    buffer_processed = true;
                    let parsing_result =
                        IBusMessageParser::extract_and_validate_message_content(raw_message_buffer);
                    if !parsing_result.is_err() {
                        let (buffer, size) = parsing_result.unwrap();
                        message_callback(buffer, size);
                    }
                }
                // Read until there is nothing to read then break the loop
                // and return back to validate for the protocol time gap
                let read_length = uart_driver.read(&mut read_buffer, 0).unwrap();
                if read_length <= 0 {
                    break;
                }
                current_index += 1;
            }
        }
    }
}
