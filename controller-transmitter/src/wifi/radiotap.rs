// Bits start at 0
bitflags::bitflags! {
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
    pub struct RadiotapFlags: u32 {
        const RATE =0x04;
        const CHANNEL =0x08;
        const TX_POWER =0x0400;

    }
}

//The fields in this struct need to be ordered in the same position as the bit that activates them in the fields bitmap, 
//check the RadiotapFlags struct for more context.
#[repr(C, packed)]
pub struct RadiotapHeaderFields {
    pub rate: u8,
    pub channel_padding: u8, //This is needed because the channel is a 16bit field and need to start at the next 16bit boundary(That's what I understood from the docs)
    pub channel_freq: u16,
    pub channel_flags: u16,
    pub tx_power: u8,
}

#[repr(C, packed)]
pub struct RadiotapHeader {
    pub version: u8,
    pub pad: u8,
    pub header_length: u16,
    pub fields_bitmap: RadiotapFlags,
    pub fields: RadiotapHeaderFields,
}

#[repr(C, packed)]
pub struct RadiotapPacket<const DATA_SIZE: usize> {
    pub header: RadiotapHeader,
    pub data: [u8; DATA_SIZE],
}
