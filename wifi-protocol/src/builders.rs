use std::mem;

use crate::{
    ieee80211_frames::{
        FrameControl, IBSSWifiPacketFrame, LogicalLinkControl, MacAddr, SequenceControl,
    },
    radiotap::{DataRate, RadiotapFlags, RadiotapHeader, RadiotapHeaderFields, RadiotapPacket},
};

pub fn build_radiotap<T>(data_rate: DataRate, data_frame: T) -> RadiotapPacket<T> {
    let radiotap_header_fields = RadiotapHeaderFields {
        rate: data_rate,
        // These fields apparently do nothing
        // channel_padding: 0x00, //This needs to be 0 it's just padding,
        // channel_freq: WifiChannelFreq::Channel3,
        // channel_flags: 0x0080,
        tx_power: 0x16, // 22dbm
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

    RadiotapPacket {
        header: radiotap_header,
        data: data_frame,
    }
}

pub fn build_ibss_broadcast_data_frame<T>(
    bssid: MacAddr,
    transmitter_addr: MacAddr,
    data: T,
    sap: u8,
) -> IBSSWifiPacketFrame<T> {
    IBSSWifiPacketFrame {
        frame_control: FrameControl::get_data_ibss_frame_control(),
        duration: 0x0000,
        destination_address: MacAddr {
            mac: [0xff, 0xff, 0xff, 0xff, 0xff, 0xff],
        },
        source_address: transmitter_addr,
        bssid,
        sequence_control: SequenceControl::new()
            .with_fragment_number(0)
            .with_sequence_number(0),
        logical_link_control: LogicalLinkControl::new()
            .with_dsap(sap)
            .with_ssap(sap)
            .with_control_field(0x1100), //Just a random value to display correctly in wireshark, theres no meaning here
        data,
        // crc: 0x00000000,
    }
}
