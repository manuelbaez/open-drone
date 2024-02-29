#![feature(ptr_metadata)]

extern crate rawsock;
use std::{mem, thread::sleep, time::Duration};

use pcap::Device;
use shared_definitions::{
    controller::{ControllerInput, PIDTuneConfig, PIDTuneInput},
    wifi::{
        builders::{build_ibss_broadcast_data_frame, build_radiotap},
        ieee80211_frames::{IBSSWifiPacketFrame, MacAddr},
        payloads::CustomSAPs,
        radiotap::{DataRate, RadiotapPacket},
    },
};

use crate::input::ControlInputMapper;

mod input;
use proctitle::set_title;

const PAIRING_BSSID: [u8; 6] = [0x50, 0xba, 0x61, 0x2a, 0x4a, 0x5e];
const TRANSMITTER_ADDRESS: [u8; 6] = [0xf2, 0xda, 0xd7, 0x60, 0x7e, 0xa9];

fn main() {
    set_title("drone-controller");
    let mut device = Device::from("wlp6s0").open().unwrap();
    println!("Open Interface for Tuning!");

    #[cfg(feature = "tuning")]
    {
        loop {
            let packet_payload = PIDTuneInput {
                pitch: PIDTuneConfig {
                    proportional_multiplier: 0.3,
                    integral_multiplier: 0.0,
                    derivative_multiplier: 0.07,
                    max_accumulated_error: 20.0,
                },
                roll: PIDTuneConfig {
                    proportional_multiplier: 0.3,
                    integral_multiplier: 0.0,
                    derivative_multiplier: 0.07,
                    max_accumulated_error: 20.0,
                },
                yaw: PIDTuneConfig {
                    proportional_multiplier: 0.3,
                    integral_multiplier: 0.0,
                    derivative_multiplier: 0.0,
                    max_accumulated_error: 0.0,
                },
            };
            let wifi_frame = build_ibss_broadcast_data_frame(
                MacAddr { mac: PAIRING_BSSID },
                MacAddr {
                    mac: TRANSMITTER_ADDRESS,
                },
                packet_payload,
                CustomSAPs::PidTuningFrame as u8,
            );

            let radiotap_packet = build_radiotap(DataRate::Rate1Mbps, wifi_frame);
            let radiotap_packet_size: usize =
                mem::size_of::<RadiotapPacket<IBSSWifiPacketFrame<PIDTuneInput>>>();
            let radiotap_packet_pointer = &radiotap_packet as *const _ as *mut u8;
            let radiotap_data = unsafe {
                core::slice::from_raw_parts(radiotap_packet_pointer, radiotap_packet_size)
            };
            device.sendpacket(radiotap_data).unwrap();
            println!("Sent Tuning {:?}", packet_payload);
            sleep(Duration::from_millis(1000));
        }
    }
    #[cfg(feature = "controller")]
    {
        let mut device = Device::from("wlp6s0").open().unwrap();
        let control_mapper: ControlInputMapper = ControlInputMapper::new("/dev/input/event23");
        control_mapper.start_event_handler_thread();
        loop {
            let current_input = control_mapper.get_current_input();
            println!(
                "Throttle: {} - Pitch: {} - Roll: {} - Yaw: {} - Kill: {} - Start: {} - Calibrate : {}",
                current_input.throttle,
                current_input.pitch,
                current_input.roll,
                current_input.yaw,
                current_input.kill_motors,
                current_input.start_motors,
                current_input.calibrate,
            );
            let packet_payload = ControllerInput {
                throttle: current_input.throttle,
                pitch: current_input.pitch,
                roll: current_input.roll,
                yaw: current_input.yaw,
                start: current_input.start_motors,
                kill_motors: current_input.kill_motors,
                calibrate_esc: current_input.calibrate,
                calibrate_sensors: false,
            };

            let wifi_frame = build_ibss_broadcast_data_frame(
                MacAddr { mac: PAIRING_BSSID },
                MacAddr {
                    mac: TRANSMITTER_ADDRESS,
                },
                packet_payload,
                CustomSAPs::ControllerFrame as u8,
            );

            let radiotap_packet = build_radiotap(DataRate::Rate1Mbps, wifi_frame);
            let radiotap_packet_size: usize =
                mem::size_of::<RadiotapPacket<IBSSWifiPacketFrame<ControllerInput>>>();
            let radiotap_packet_pointer = &radiotap_packet as *const _ as *mut u8;
            let radiotap_data = unsafe {
                core::slice::from_raw_parts(radiotap_packet_pointer, radiotap_packet_size)
            };
            device.sendpacket(radiotap_data).unwrap();

            sleep(Duration::from_millis(5));
        }
    }
}
