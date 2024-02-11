extern crate rawsock;
use std::{thread::sleep, time::Duration};

use pcap::Device;
use radiotap::{field::Header, Radiotap, RadiotapIterator};
use rawsock::open_best_library;

const WIFI_PACKET: [u8; 41] = [
    0x00, 0x00, // <-- radiotap version
    0x0f, 0x00, // <- radiotap header length
    0x04, 0x0c, 0x00, 0x00, // <-- bitmap
    0x6c, // <-- rate
    0x09, 0xA8, 0x80, 0x00, //<-- Channel
    0x0c, //<-- tx power
    0x01, //  antenna
    0x08, 0x01, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x13, 0x22, 0x33, 0x44, 0x55, 0x66,
    0x13, 0x22, 0x33, 0x44, 0x55, 0x66, 0x10, 0x86, 0xF1, 0xF1,
];

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
    let mut cap = Device::from("mon0").open().unwrap();

    loop {
        cap.sendpacket(WIFI_PACKET).unwrap();
        println!("Sent packet!");
        sleep(Duration::from_secs(1));
    }
}
