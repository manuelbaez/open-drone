#![feature(ptr_metadata)]

extern crate rawsock;
use std::{mem, thread::sleep, time::Duration};

use pcap::Device;
use wifi_protocol::{
    builders::{build_ibss_broadcast_data_frame, build_radiotap},
    ieee80211_frames::{IBSSWifiPacketFrame, MacAddr},
    payloads::{CustomSAPs, DroneMovementsFramePayload},
    radiotap::{DataRate, RadiotapPacket},
};

use crate::input::ControlInputMapper;

mod input;
use proctitle::set_title;

const PAIRING_BSSID: [u8; 6] = [0x50, 0xba, 0x61, 0x2a, 0x4a, 0x5e];
const TRANSMITTER_ADDRESS: [u8; 6] = [0xf2, 0xda, 0xd7, 0x60, 0x7e, 0xa9];

fn main() {
    set_title("drone-controller");
    println!("Open Interface!");

    let mut device = Device::from("wlp6s0").open().unwrap();
    let control_mapper: ControlInputMapper = ControlInputMapper::new("/dev/input/event23");
    control_mapper.start_event_handler_thread();
    loop {
        let current_input = control_mapper.get_current_input();
        println!(
            "Throttle: {} - Pitch: {} - Roll: {} - Yaw: {} - Kill: {} - Start: {}",
            current_input.throttle,
            current_input.pitch,
            current_input.roll,
            current_input.yaw,
            current_input.kill_motors,
            current_input.start_motors
        );
        let packet_payload = DroneMovementsFramePayload {
            throttle: current_input.throttle,
            pitch: current_input.pitch,
            roll: current_input.roll,
            yaw: current_input.yaw,
            start: current_input.start_motors,
            kill_motors: current_input.kill_motors,
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
            mem::size_of::<RadiotapPacket<IBSSWifiPacketFrame<DroneMovementsFramePayload>>>();
        let radiotap_packet_pointer = &radiotap_packet as *const _ as *mut u8;
        let radiotap_data =
            unsafe { core::slice::from_raw_parts(radiotap_packet_pointer, radiotap_packet_size) };
        device.sendpacket(radiotap_data).unwrap();

        sleep(Duration::from_millis(5));
    }
}
