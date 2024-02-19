use bitfield_struct::bitfield;

/// Ordered LSB to MSB
#[bitfield(u16)]
pub struct FrameControl {
    #[bits(2)]
    pub protocol: u8,
    #[bits(2)]
    pub frame_type: u8,
    #[bits(4)]
    pub frame_sub_type: u8,
    //Flags
    pub to_ds: bool,
    pub from_ds: bool,
    pub more_fragments: bool,
    pub retry: bool,
    pub power_management: bool,
    pub more_data: bool,
    pub protected_frame: bool,
    pub order: bool,
}

impl FrameControl {
    pub fn get_data_ibss_frame_control() -> FrameControl {
        FrameControl::new()
            .with_frame_type(0x2)
            .with_protocol(0x0)
            .with_frame_sub_type(0x0)
            .with_to_ds(false)
            .with_from_ds(false)
            .with_retry(false)
            .with_power_management(false)
            .with_more_fragments(false)
            .with_more_data(false)
            .with_protected_frame(false)
            .with_order(false)
    }
}

///Pointless for sending, the physical layer adds it automagically
#[bitfield(u16)]
pub struct SequenceControl {
    #[bits(12)]
    pub sequence_number: u16,
    #[bits(4)]
    pub fragment_number: u8,
}

#[bitfield(u32)]
pub struct LogicalLinkControl {
    pub dsap: u8,
    pub ssap: u8,
    //control field
    pub control_field: u16,
}

#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct MacAddr {
    pub mac: [u8; 6],
}

#[repr(C, packed)]
pub struct IBSSWifiPacketFrame<T> {
    pub frame_control: FrameControl,
    /// Set this to zeroes, this is added automatically at the physical layer
    pub duration: u16,
    /// Set this to broadcast(all bytes 0xff), this is the only way I know for it to not expect an ACK atm,
    /// otherwise the card will enter a death loop sending the same packet
    pub destination_address: MacAddr, // For DS(0,0) Destination
    pub source_address: MacAddr, // For DS(0,0) Source
    pub bssid: MacAddr,          // For DS(0,0) BSSID
    ///Set to zero, this apparently is handled at the hardware/driver level so there's no point on setting it other than padding
    pub sequence_control: SequenceControl,
    pub logical_link_control: LogicalLinkControl,
    pub data: T,
}

#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct GenericWifiPacketFrameHeader {
    pub frame_control: FrameControl,
    pub duration: u16,
    pub address_1: MacAddr,
    pub address_2: MacAddr,
    pub address_3: MacAddr,
    pub sequence_control: SequenceControl,
    /// This is not a generic field but I added it to validate the frame type before parsing frames on the receiving end
    pub logical_link_control: LogicalLinkControl,
}
