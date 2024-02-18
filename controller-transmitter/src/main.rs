#![feature(ptr_metadata)]

extern crate rawsock;
use core::slice;
use std::{
    default,
    f32::consts::E,
    fmt::Display,
    fs::OpenOptions,
    io::{Cursor, Read},
    mem,
    thread::sleep,
    time::Duration,
};

use byteorder::{NativeEndian, ReadBytesExt};
use pcap::{Active, Capture, Device};
use wifi_protocol::{
    ieee80211_frame::{
        FrameControl, IBSSWifiPacketFrame, LogicalLinkControl, MacAddr, SequenceControl,
    },
    radiotap::{DataRate, RadiotapFlags, RadiotapHeader, RadiotapHeaderFields, RadiotapPacket},
};

use crate::input::controller::{EventTypes, InputEvent};

mod input;

fn build_and_send_packet(device: &mut Capture<Active>) {
    let radiotap_header_fields = RadiotapHeaderFields {
        rate: DataRate::Rate1Mbps,
        // channel_padding: 0x00, //This needs to be 0 it's just padding,
        // channel_freq: WifiChannelFreq::Channel3,
        // channel_flags: 0x0080,
        tx_power: 0x16,
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
        logical_link_control: LogicalLinkControl::new()
            .with_dsap(0x00)
            .with_ssap(0x11)
            .with_control_field(0x1100),
        data,
        // crc: 0x00000000,
    };

    let frame_pointer = &wifi_frame as *const _ as *mut u8;
    let wifi_frame_data = unsafe {
        core::slice::from_raw_parts_mut(frame_pointer, mem::size_of::<IBSSWifiPacketFrame<6>>())
    };

    let packet = RadiotapPacket {
        header: radiotap_header,
        data: wifi_frame,
    };

    let packet_pointer = &packet as *const _ as *mut u8;
    let packet_size: usize = mem::size_of::<RadiotapPacket<6>>();

    let packet_array = unsafe { core::slice::from_raw_parts_mut(packet_pointer, packet_size) };

    // println!(
    //     "Sent packet! Frame length:{} - Radiotap Data: {:02x?}",
    //     wifi_frame_data.len(),
    //     packet_array
    // );
    device.sendpacket(packet_array).unwrap();
}

fn main() {
    // let mut cap = Device::lookup().unwrap().unwrap().open().unwrap();
    println!("Open Interface!");
    let mut device = Device::from("wlp6s0").open().unwrap();

    let mut file_options = OpenOptions::new();
    file_options.read(true);
    file_options.write(false);

    let mut dev_file = file_options.open("/dev/input/event23").unwrap();

    loop {
        let struct_size = mem::size_of::<InputEvent>();
        let event: InputEvent = unsafe { mem::zeroed() };
        let mut event_buffer =
            unsafe { slice::from_raw_parts_mut(&event as *const _ as *mut u8, struct_size) };
        let size = dev_file.read(&mut event_buffer).unwrap();

        // let mut rdr = Cursor::new(packet);
        // let tv_sec = rdr.read_u64::<NativeEndian>().unwrap();
        // let tv_usec = rdr.read_u64::<NativeEndian>().unwrap();
        // let evtype = rdr.read_u16::<NativeEndian>().unwrap();
        // let code = rdr.read_u16::<NativeEndian>().unwrap();
        // let value = rdr.read_i32::<NativeEndian>().unwrap();
        // println!(
        //     "{} {} Event type:{} Event code:{}  Value:{}",
        //     event.tv_sec, event.tv_usec, event.evtype, event.code, event.value
        // );

        match event.event_type {
            EventTypes::ButtonPress => {
                println!("Button Pressed {}- {}", event.code, event.value);
            }
            EventTypes::AnalogInput => {
                println!("Analog{}- {}", event.code, event.value);
                // if event.code == AnalogInputCodes::LeftY as u16 {
                // }
            }
            _default => {
                // println!("Something Pressed {}", _default as u16);
            }
        }
        build_and_send_packet(&mut device);
        // sleep(Duration::from_secs(1));
    }
}
