/// I'm using the SAP fields to tell the receiver what type of data the frame
/// carries, not sure if that's the intended purpose though
/// (https://en.wikipedia.org/wiki/Service_Access_Point)
#[repr(u8)]
pub enum CustomSAPs {
    ControllerFrame = 0x01,
    PidTuningFrame = 0x02,
}

impl TryFrom<u8> for CustomSAPs {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            value if value == CustomSAPs::ControllerFrame as u8 => Ok(CustomSAPs::ControllerFrame),
            value if value == CustomSAPs::PidTuningFrame as u8 => Ok(CustomSAPs::PidTuningFrame),
            _ => Err(()),
        }
    }
}
