use bitfield_struct::bitfield;

#[bitfield(u16)]
pub struct FrameControl {
    /// Ordered LSB to MSB
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

#[bitfield(u16)]
pub struct SequenceControl {
    #[bits(12)]
    pub sequence_number: u16,
    #[bits(4)]
    pub fragment_number: u8,
}

impl FrameControl {
    pub fn get_data_no_ds_frame_control() -> FrameControl {
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

#[repr(C, packed)]
pub struct MacAddr {
    pub mac: [u8; 6],
}

#[repr(C, packed)]
pub struct NoDsWifiPacketFrame<const DATA_SIZE: usize> {
    pub frame_control: FrameControl,
    pub duration: u16,
    pub address_1: MacAddr, // For DS(0,0) Destination
    pub address_2: MacAddr, // For DS(0,0) Source
    pub address_3: MacAddr, // For DS(0,0) BSSID
    pub sequence_control: SequenceControl,
    //pub addr_4: MacAddr, // For DS(0,0) not used
    pub payload: [u8; DATA_SIZE],
    // pub crc: u32,
}
