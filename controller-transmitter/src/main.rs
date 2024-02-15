#![feature(ptr_metadata)]

extern crate rawsock;
use std::{io::Read, mem};

use pcap::{Device, Linktype};
use radiotap::Radiotap;

use crate::wifi::radiotap::{RadiotapFlags, RadiotapHeader, RadiotapHeaderFields, RadiotapPacket};
mod wifi;
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
const RATE_1_MBPS: u8 = 0x2;

#[repr(C, packed)]
struct MacAddr {
    mac: [u8; 6],
}
#[repr(C, packed)]
struct WifiPacketPayload {
    fctl: u16,
    duration: u16,
    destination_address: MacAddr,
    source_address: MacAddr,
    bssid: MacAddr,
    seqctl: u16,
    addr_4: MacAddr,
    payload: &'static [u8],
}

fn main() {
    // let mut cap = Device::lookup().unwrap().unwrap().open().unwrap();
    println!("Open Interface!");
    let mut device = Device::from("wlp6s0").open().unwrap();

    // let radiotap_header_payload = [
    //     RATE_1_MBPS,
    //     0x00, /*Wft is this byte? maybe padding?*/
    //     0xA8,
    //     0x09, // <- channel freq 2472
    //     0x80,
    //     0x00, // <- channel flags 2ghz
    //     0x0c, // <- Tx power
    // ];

    let radiotap_header_fields = RadiotapHeaderFields {
        rate: RATE_1_MBPS,
        channel_padding: 0x00, //This needs to be 0 it's just padding,
        channel_freq: 0x09A8,
        channel_flags: 0x0080,
        tx_power: 0x0c,
    };

    let bitmap_flags = RadiotapFlags::RATE | RadiotapFlags::CHANNEL | RadiotapFlags::TX_POWER;
    let mut radiotap_header = RadiotapHeader {
        version: 0x00,
        pad: 0x00,
        header_length: 0x00,
        fields_bitmap: bitmap_flags,
        fields: radiotap_header_fields,
    };
    
    let header_size = mem::size_of::<RadiotapHeader>();
    radiotap_header.header_length = header_size as u16;

    let data: [u8; 26] = [
        0x08, 0x01, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x13, 0x22, 0x33, 0x44, 0x55,
        0x66, 0x13, 0x22, 0x33, 0x44, 0x55, 0x66, 0x10, 0x86, 0xF1, 0xF1,
    ];
    // let data = [0x01];

    // let wifi_frame= WifiPacketPayload{

    // }

    let packet = RadiotapPacket {
        header: radiotap_header,
        data,
    };

    let packet_pointer = &packet as *const _ as *mut u8;
    let packet_size = mem::size_of::<RadiotapPacket<26>>();

    let packet_array = unsafe { core::slice::from_raw_parts_mut(packet_pointer, packet_size) };

    // println!("Packet Array{:02X?}", packet_array);

    // let radiotap_packet = Radiotap::from_bytes(&packet_array).unwrap();
    // // match radiotap_header.antenna. {

    // // }
    // println!("Radiotap Freq {}", radiotap_packet.channel.unwrap().freq);
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
