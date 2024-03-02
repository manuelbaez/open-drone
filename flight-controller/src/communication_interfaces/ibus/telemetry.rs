use super::protocol::{
    IBusMessageParser, IbusCommands, IbusTelemetrySensorsIds, PROTOCOL_OVERHEAD, PROTOCOL_SIZE,
};
use crate::communication_interfaces::controller::RemoteTelemetry;
use crate::shared_core_values::AtomicTelemetry;
use esp_idf_svc::hal::peripheral::Peripheral;
use esp_idf_svc::hal::prelude::*;
use esp_idf_svc::hal::{
    gpio,
    peripherals::Peripherals,
    uart::{config, UartDriver},
};
use std::sync::atomic::Ordering;
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

pub struct IBusTelemetry<'a> {
    uart_driver: UartDriver<'a>,
}

impl<'a> IBusTelemetry<'a> {
    pub fn new(peripherals: &mut Peripherals) -> Self {
        let tx = unsafe { peripherals.pins.gpio18.clone_unchecked() };
        let rx = unsafe { peripherals.pins.gpio5.clone_unchecked() };

        let uart2 = unsafe { peripherals.uart1.clone_unchecked() };
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

    fn send_sensor_value(
        &self,
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
                self.uart_driver.write(&tx_buffer).unwrap();
            }
            _ => (),
        }
    }

    fn process_ibus_buffer(
        &self,
        message_buffer: [u8; PROTOCOL_SIZE - PROTOCOL_OVERHEAD],
        message_length: usize,
        shared_telemetry: &AtomicTelemetry,
    ) {
        let message_command = message_buffer[0];
        match message_command {
            IbusCommands::CHANNELS_DATA => (),
            _ => {
                let sensor_addr: u8 = message_buffer[0] & 0x0f; //4 LS Bits of the read byte
                if sensor_addr <= IBUS_SENSORS.len() as u8 && sensor_addr > 0 && message_length == 1
                {
                    let mapped_command = message_buffer[0] & 0xf0; //4 MS Bits of the read byte
                    let sensor = &IBUS_SENSORS[sensor_addr as usize - 1];
                    match mapped_command {
                        IbusCommands::SENSORS_DISCOVER => {
                            let message = [IbusCommands::SENSORS_DISCOVER + sensor_addr];
                            let tx_buffer: [u8; 4] =
                                IBusMessageParser::format_ibus_message(message);
                            self.uart_driver.write(&tx_buffer).unwrap();
                        }
                        IbusCommands::POLL_SENSOR_TYPE => {
                            let message = [
                                IbusCommands::POLL_SENSOR_TYPE + sensor_addr,
                                IbusTelemetrySensorsIds::EXTERNAL_VOLTAGE,
                                sensor.size, //Sensor data length
                            ];
                            let tx_buffer: [u8; 6] =
                                IBusMessageParser::format_ibus_message(message);
                            self.uart_driver.write(&tx_buffer).unwrap();
                        }
                        IbusCommands::MEASUREMENT => self.send_sensor_value(
                            sensor_addr,
                            sensor.sensor_type,
                            shared_telemetry,
                        ),
                        _ => (),
                    }
                }
            }
        }
    }
}

impl<'a> RemoteTelemetry for IBusTelemetry<'a> {
    fn start_telemetry_tx_loop(&self, shared_telemetry: &AtomicTelemetry) {
        IBusMessageParser::start_monitor_on_uart(&self.uart_driver, |buffer, message_length| {
            self.process_ibus_buffer(buffer, message_length, shared_telemetry);
        });
    }
}
