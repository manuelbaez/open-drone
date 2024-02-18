#![feature(ptr_metadata)]

extern crate rawsock;
use std::{io::Read, mem, ptr::from_raw_parts, thread::sleep, time::Duration};

use libwifi::frame::Data;
use pcap::{Device, Linktype};
use radiotap::Radiotap;

use crate::wifi::{
    ieee80211_frame::{FrameControl, IBSSWifiPacketFrame, LogicalLinkControl, MacAddr, SequenceControl},
    radiotap::{
        DataRate, RadiotapFlags, RadiotapHeader, RadiotapHeaderFields, RadiotapPacket,
        WifiChannelFreq,
    },
};
mod wifi;

fn main() {
    // let mut cap = Device::lookup().unwrap().unwrap().open().unwrap();
    println!("Open Interface!");
    let mut device = Device::from("wlp6s0").open().unwrap();

    loop {
        let radiotap_header_fields = RadiotapHeaderFields {
            rate: DataRate::Rate54Mbps,
            // channel_padding: 0x00, //This needs to be 0 it's just padding,
            // channel_freq: WifiChannelFreq::Channel3,
            // channel_flags: 0x0080,
            tx_power: 0x0c,
        };

        let bitmap_flags = RadiotapFlags::RATE | RadiotapFlags::TX_POWER;
        let mut radiotap_header = RadiotapHeader {
            version: 0x00,
            pad: 0x00,
            header_length: 0x0000,
            fields_bitmap: bitmap_flags,
            fields: radiotap_header_fields,
        };

        let header_size = mem::size_of::<RadiotapHeader>();
        radiotap_header.header_length = header_size as u16;

        let data = [0x00, 0x01, 0x02, 0x03, 0x04, 0x05];

        let wifi_frame = IBSSWifiPacketFrame {
            frame_control: FrameControl::get_data_ibss_frame_control(),
            duration: 0x0000,
            destination_address: MacAddr {
                mac: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF],
            },
            source_address: MacAddr {
                mac: [0x13, 0x22, 0x33, 0x44, 0x55, 0x66],
            },
            bssid: MacAddr {
                mac: [0x11, 0x22, 0x33, 0x44, 0x55, 0x66],
            },
            sequence_control: SequenceControl::new()
                .with_fragment_number(0)
                .with_sequence_number(0),
            logical_link_control: LogicalLinkControl::new().with_dsap(0x00).with_ssap(0x11).with_control_field(0x1100),
            data,
            // crc: 0x00000000,
        };

        let frame_pointer = &wifi_frame as *const _ as *mut u8;
        let wifi_frame_data = unsafe {
            core::slice::from_raw_parts_mut(frame_pointer, mem::size_of::<IBSSWifiPacketFrame<6>>())
        };

        // println!("Wifi frame control {:02x?}", wifi_frame_data);
        match libwifi::parse_frame(wifi_frame_data) {
            Ok(frame) => {
                println!("Got frame: {:?}", frame);
            }
            Err(err) => {
                println!("Error during parsing :\n{}", err);
            }
        };

        let packet = RadiotapPacket {
            header: radiotap_header,
            data: wifi_frame,
        };

        let packet_pointer = &packet as *const _ as *mut u8;
        let packet_size: usize = mem::size_of::<RadiotapPacket<6>>();

        let packet_array = unsafe { core::slice::from_raw_parts_mut(packet_pointer, packet_size) };

        println!("Sent packet! {:02x?}", packet_array);
        device.sendpacket(packet_array).unwrap();
        sleep(Duration::from_secs(1));
    }

    // loop {
    //     let packet = device.next_packet().unwrap();
    //     print!("Packet: {:02x?}", packet.bytes());
    // }

    // println!("Packet: {:#02X?}", device.next_packet().unwrap().bytes());
    // loop {
    //     cap.sendpacket(WIFI_PACKET).unwrap();
    //     sleep(Duration::from_secs(1));
    // }
}
