#![feature(ptr_metadata)]

extern crate rawsock;
use std::{io::Read, mem};

use pcap::{Device, Linktype};
use radiotap::Radiotap;

// const WIFI_PACKET: [u8; 41] = [
//     0x00, 0x00, // <-- radiotap version
//     0x0f, 0x00, // <- radiotap header length
//     0x04, 0x0c, 0x00, 0x00, // <-- bitmap
//     0x6c, // <-- rate
//     0x09, 0xA8, 0x80, 0x00, //<-- Channel
//     0x0c, //<-- tx power
//     0x01, //  antenna
//     0x08, 0x01, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x13, 0x22, 0x33, 0x44, 0x55, 0x66,
//     0x13, 0x22, 0x33, 0x44, 0x55, 0x66, 0x10, 0x86, 0xF1, 0xF1,
// ];

const RATE_54_MBPS: u8 = 0x6c;

// #[repr(C, packed)]
// struct MacAddr {
//     mac: [u8; 6],
// }
// #[repr(C, packed)]
// struct WifiPacketPayload {
//     fctl: u16,
//     duration: u16,
//     destination_address: MacAddr,
//     source_address: MacAddr,
//     bssid: MacAddr,
//     seqctl: u16,
//     addr_4: MacAddr,
//     payload: &'static [u8],
// }

// Bits start at 0
bitflags::bitflags! {
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
    struct RadiotapFlags: u32 {
        const RATE =0x04;
        const CHANNEL =0x08;
        const TX_POWER =0x0400;

    }
}

#[repr(C, packed)]
struct RadiotapHeader<const PAYLOAD_SIZE: usize> {
    version: u8,
    pad: u8,
    header_length: u16,
    bitmap: RadiotapFlags,
    payload: [u8; PAYLOAD_SIZE],
}

#[repr(C, packed)]
struct RadiotapPacket<const HEADER_PAYLOAD_SIZE: usize, const DATA_SIZE: usize> {
    header: RadiotapHeader<HEADER_PAYLOAD_SIZE>,
    data: [u8; DATA_SIZE],
}

fn main() {
    // let mut cap = Device::lookup().unwrap().unwrap().open().unwrap();
    println!("Open Interface!");
    let mut device = Device::from("mon0").open().unwrap();

    let radiotap_header_payload = [
        RATE_54_MBPS,
        0x00, /*Wft is this byte? maybe padding?*/
        0xA8,
        0x09, // <- channel freq 2472
        0x80,
        0x00, // <- channel flags 2ghz
        0x0c, // <- Tx power
    ];

    let bitmap_flags = RadiotapFlags::RATE | RadiotapFlags::CHANNEL | RadiotapFlags::TX_POWER;
    let mut radiotap_header = RadiotapHeader {
        version: 0x00,
        pad: 0x00,
        header_length: 0x00,
        bitmap: bitmap_flags,
        payload: radiotap_header_payload,
    };
    let header_size = mem::size_of::<RadiotapHeader<7>>();
    radiotap_header.header_length = header_size as u16;

    let data = [
        0x08, 0x01, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x13, 0x22, 0x33, 0x44, 0x55,
        0x66, 0x13, 0x22, 0x33, 0x44, 0x55, 0x66, 0x10, 0x86, 0xF1, 0xF1,
    ];

    let packet: RadiotapPacket<7, 26> = RadiotapPacket {
        header: radiotap_header,
        data,
    };

    let packet_pointer = &packet as *const _ as *mut u8;
    let packet_size = mem::size_of::<RadiotapPacket<7, 26>>();

    let packet_array = unsafe { core::slice::from_raw_parts_mut(packet_pointer, packet_size) };

    // println!("Packet Array{:02X?}", packet);

    let radiotap_packet = Radiotap::from_bytes(&packet_array).unwrap();
    // // match radiotap_header.antenna. {

    // // }
    println!("Radiotap Freq {}", radiotap_packet.channel.unwrap().freq);
    // println!(
    //     "Radiotap Rate {:02X?}",
    //     radiotap_header.tx_power.unwrap().value
    // );

    // radiotap::Radiotap::from()

    // match libwifi::parse_frame(&data) {
    //     Ok(frame) => {
    //         println!("Got frame: {:?}", frame);
    //     }
    //     Err(err) => {
    //         println!("Error during parsing :\n{}", err);
    //     }
    // };

    device.sendpacket(packet_array).unwrap();
    println!("Sent packet!");

    // println!("Packet: {:#02X?}", device.next_packet().unwrap().bytes());
    // loop {
    //     cap.sendpacket(WIFI_PACKET).unwrap();
    //     sleep(Duration::from_secs(1));
    // }
}
