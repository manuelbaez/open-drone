#[repr(u16)]
pub enum ButtonPressCodes {
    Y = 308,
    B = 305,
    A = 304,
    X = 307,
    Menu = 315,
    Windows = 314,
    Logo = 316,
    LB = 310,
    RB = 311,
}

impl TryFrom<u16> for ButtonPressCodes {
    type Error = ();

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        match value {
            x if x == ButtonPressCodes::Y as u16 => Ok(ButtonPressCodes::Y),
            x if x == ButtonPressCodes::B as u16 => Ok(ButtonPressCodes::B),
            x if x == ButtonPressCodes::A as u16 => Ok(ButtonPressCodes::A),
            x if x == ButtonPressCodes::X as u16 => Ok(ButtonPressCodes::X),
            x if x == ButtonPressCodes::Menu as u16 => Ok(ButtonPressCodes::Menu),
            x if x == ButtonPressCodes::Windows as u16 => Ok(ButtonPressCodes::Windows),
            x if x == ButtonPressCodes::Logo as u16 => Ok(ButtonPressCodes::Logo),
            x if x == ButtonPressCodes::LB as u16 => Ok(ButtonPressCodes::LB),
            x if x == ButtonPressCodes::RB as u16 => Ok(ButtonPressCodes::RB),
            _ => Err(()),
        }
    }
}
#[repr(u16)]
pub enum AnalogInputCodes {
    LeftX = 0,
    LeftY = 1,
    RightX = 3,
    RightY = 4,
    LT = 2,
    RT = 5,
}

impl TryFrom<u16> for AnalogInputCodes {
    type Error = ();

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        match value {
            x if x == AnalogInputCodes::LeftX as u16 => Ok(AnalogInputCodes::LeftX),
            x if x == AnalogInputCodes::LeftY as u16 => Ok(AnalogInputCodes::LeftY),
            x if x == AnalogInputCodes::RightX as u16 => Ok(AnalogInputCodes::RightX),
            x if x == AnalogInputCodes::RightY as u16 => Ok(AnalogInputCodes::RightY),
            x if x == AnalogInputCodes::LT as u16 => Ok(AnalogInputCodes::LT),
            x if x == AnalogInputCodes::RT as u16 => Ok(AnalogInputCodes::RT),
            _ => Err(()),
        }
    }
}

#[repr(u16)]
#[derive(Debug)]
#[allow(dead_code)]
pub enum EventTypes {
    None = 0,
    ButtonPress = 1,
    AnalogInput = 3,
}
#[repr(C)]
#[derive(Debug)]
pub struct InputEvent {
    pub tv_sec: u64,
    pub tv_usec: u64,
    pub event_type: EventTypes,
    pub code: u16,
    pub value: i32,
}
