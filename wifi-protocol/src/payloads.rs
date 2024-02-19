/// I'm usinf the SAP fields to tell the receiver what type of data the frame
/// carries, not sure if that's the intended purpose though
/// (https://en.wikipedia.org/wiki/Service_Access_Point)
#[repr(u8)]
pub enum CustomSAPs {
    ControllerFrame = 0x01,
}

impl TryFrom<u8> for CustomSAPs {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            value if value == CustomSAPs::ControllerFrame as u8 => Ok(CustomSAPs::ControllerFrame),
            _ => Err(()),
        }
    }
}

#[derive(Debug)]
#[repr(C, packed)]
pub struct DroneMovementsFramePayload {
    pub roll: i32,
    pub pitch: i32,
    pub yaw: i32,
    pub throttle: u16,
    pub kill_motors: bool,
    pub start: bool,
}
