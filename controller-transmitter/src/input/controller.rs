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

#[repr(u16)]
pub enum AnalogInputCodes {
    LeftX = 0,
    LeftY = 1,
    RightX = 3,
    RightY = 4,
    LT = 2,
    RT = 5,
}

#[repr(u16)]
#[derive(Debug)]
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
