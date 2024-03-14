use embassy_time::Timer;
use esp_hal::clock::Clocks;
use esp_hal::gpio::{InputPin, OutputPin};
use esp_hal::peripheral::Peripheral;
use esp_hal::{prelude::*, uart, Uart};

use super::protocol::{
    IBusMessageParser, IBusUartMonitor, IbusCommands, IbusTelemetrySensorsIds, PROTOCOL_OVERHEAD,
    PROTOCOL_SIZE,
};
use crate::shared_core_values::AtomicTelemetry;
use core::sync::atomic::Ordering;
struct IbusSensor {
    sensor_type: u8,
    size: u8, //can be 2 or 4 bytes
}

static IBUS_SENSORS: [IbusSensor; 1] = [IbusSensor {
    sensor_type: IbusTelemetrySensorsIds::EXTERNAL_VOLTAGE,
    size: 0x2,
}];

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

pub struct IBusTelemetry<'a, T> {
    uart_driver: Uart<'a, T>,
    shared_telemetry: &'a AtomicTelemetry,
}

impl<'a, T> IBusTelemetry<'a, T>
where
    T: uart::Instance,
{
    pub fn new<TX: OutputPin, RX: InputPin>(
        tx: impl Peripheral<P = TX> + 'a,
        rx: impl Peripheral<P = RX> + 'a,
        uart: impl Peripheral<P = T> + 'a,
        shared_telemetry: &'a AtomicTelemetry,
        clocks: &Clocks,
    ) -> Self {
        let uart_config = uart::config::Config::default();
        let uart_pins = uart::TxRxPins::new_tx_rx(tx, rx);
        let uart_driver = Uart::new_with_config(uart, uart_config, Some(uart_pins), clocks);
        Self {
            uart_driver,
            shared_telemetry,
        }
    }

    fn send_sensor_value(
        &mut self,
        sensor_addr: u8,
        sensor_type: u8,
        shared_telemetry: &AtomicTelemetry,
    ) {
        match sensor_type {
            IbusTelemetrySensorsIds::EXTERNAL_VOLTAGE => {
                let battery_voltage = shared_telemetry.battery_voltage.load(Ordering::Relaxed);
                let battery_voltage = (battery_voltage * 100.0) as u16;
                let message = [
                    IbusCommands::MEASUREMENT + sensor_addr,
                    (battery_voltage & 0x00ff) as u8, /*Sensor Value lower byte */
                    ((battery_voltage >> 8) & 0x00ff) as u8, /*Sensor Value higher byte */
                ];
                let tx_buffer: [u8; 6] = IBusMessageParser::format_ibus_message(message);
                self.uart_driver.write_bytes(&tx_buffer).unwrap();
            }
            _ => (),
        }
    }
}

impl<'a, T> IBusUartMonitor<T> for IBusTelemetry<'a, T>
where
    T: uart::Instance,
{
    async fn read_uart_bytes(&mut self) -> Result<u8, ()> {
        while T::get_rx_fifo_count() == 0 {
            Timer::after_micros(100).await;
        }
        let result = self.uart_driver.read();
        if !result.is_err() {
            Ok(result.unwrap())
        } else {
            esp_println::println!("Error: {:?}", result.err());
            Err(())
        }
    }

    fn process_ibus_buffer(
        &mut self,
        message_buffer: [u8; PROTOCOL_SIZE - PROTOCOL_OVERHEAD],
        message_size: usize,
    ) {
        let message_command = message_buffer[0];
        match message_command {
            IbusCommands::CHANNELS_DATA => (),
            _ => {
                let sensor_addr: u8 = message_buffer[0] & 0x0f; //4 LS Bits of the read byte
                if sensor_addr <= IBUS_SENSORS.len() as u8 && sensor_addr > 0 && message_size == 1 {
                    let mapped_command = message_buffer[0] & 0xf0; //4 MS Bits of the read byte
                    let sensor = &IBUS_SENSORS[sensor_addr as usize - 1];
                    match mapped_command {
                        IbusCommands::SENSORS_DISCOVER => {
                            let message = [IbusCommands::SENSORS_DISCOVER + sensor_addr];
                            let tx_buffer: [u8; 4] =
                                IBusMessageParser::format_ibus_message(message);
                            self.uart_driver.write_bytes(&tx_buffer).unwrap();
                        }
                        IbusCommands::POLL_SENSOR_TYPE => {
                            let message = [
                                IbusCommands::POLL_SENSOR_TYPE + sensor_addr,
                                IbusTelemetrySensorsIds::EXTERNAL_VOLTAGE,
                                sensor.size, //Sensor data length
                            ];
                            let tx_buffer: [u8; 6] =
                                IBusMessageParser::format_ibus_message(message);
                            self.uart_driver.write_bytes(&tx_buffer).unwrap();
                        }
                        IbusCommands::MEASUREMENT => self.send_sensor_value(
                            sensor_addr,
                            sensor.sensor_type,
                            self.shared_telemetry,
                        ),
                        _ => (),
                    }
                }
            }
        }
    }

    fn get_read_buffer_count(&self) -> u16 {
        T::get_rx_fifo_count()
    }
}
