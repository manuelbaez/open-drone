use std::sync::{Arc, RwLock};
use std::time::SystemTime;

use esp_idf_svc::hal::delay::BLOCK;
use esp_idf_svc::hal::peripheral::Peripheral;
use esp_idf_svc::hal::prelude::*;
use esp_idf_svc::hal::{
    gpio,
    peripherals::Peripherals,
    uart::{config, UartDriver},
};
use shared_definitions::controller::ControllerInput;

use super::controller::RemoteControl;

const PROTOCOL_MESSAGE_TIMEGAP_US: u128 = 3500;
const PROTOCOL_SIZE: usize = 0x20;
const PROTOCOL_OVERHEAD: u8 = 3; // <len><cmd><data....><chkl><chkh>
const PROTOCOL_CHANNELS: u8 = 10;
const PROTOCOL_COMMAND40: u8 = 0x40; // Command is always 0x40

pub struct IBusController<'a> {
    uart_driver: UartDriver<'a>,
}

enum ReadingStages {
    Length,
    Data,
    GetChecksumLByte,
    GetChecksumHByte,
    Discard,
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
}

impl<'a> RemoteControl for IBusController<'a> {
    fn start_changes_monitor(&self, shared_controller_input: Arc<RwLock<ControllerInput>>) {
        let system_time = SystemTime::now();

        let mut previous_time = system_time.elapsed().unwrap().as_micros();
        let mut data_buffer = [0_u8; PROTOCOL_SIZE];
        let mut current_byte_count = 0_u8;
        let mut message_length = 0_u8;

        let mut checksum_lbyte = 0_u8;
        let mut checksum_hbyte = 0_u8;
        let mut target_checksum = 0_u16;
        
        let mut state: ReadingStages = ReadingStages::Length;
        let mut read_buffer: [u8; 1] = [0_u8];
        loop {
            self.uart_driver.read(&mut read_buffer, BLOCK).unwrap();
            
            let current_time = system_time.elapsed().unwrap().as_micros();
            let elapsed_time = current_time - previous_time;
            previous_time = current_time;
            

            if elapsed_time > PROTOCOL_MESSAGE_TIMEGAP_US {
                state = ReadingStages::Length;
            }
            match state {
                ReadingStages::Length => {
                    current_byte_count = 0;
                    if read_buffer[0] <= PROTOCOL_SIZE as u8 {
                        message_length = read_buffer[0] - PROTOCOL_OVERHEAD;
                        state = ReadingStages::Data;
                        target_checksum = 0xffff - read_buffer[0] as u16;
                    } else {
                        state = ReadingStages::Discard;
                    }
                }
                ReadingStages::Data => {
                    data_buffer[current_byte_count as usize] = read_buffer[0];
                    target_checksum -= read_buffer[0] as u16;
                    current_byte_count += 1;
                    if current_byte_count == message_length {
                        state = ReadingStages::GetChecksumLByte;
                    }
                }
                ReadingStages::GetChecksumLByte => {
                    checksum_lbyte = read_buffer[0];
                    state = ReadingStages::GetChecksumHByte
                }
                ReadingStages::GetChecksumHByte => {
                    checksum_hbyte = read_buffer[0];
                    let checksum = ((checksum_hbyte as u16) << 8) + checksum_lbyte as u16;
                    if target_checksum == checksum {
                        match data_buffer[0] {
                            PROTOCOL_COMMAND40 => {
                                println!("IBus Full: {:?}", data_buffer);
                            }
                            _ => (),
                        }
                    }
                }
                _ => (),
            }

            // data_buffer[current_byte_count] = buffer[0];
            // current_byte_count += 1;
        }
    }
}
