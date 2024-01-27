use core::fmt;
use std::fmt::{Debug, Display, Formatter};

#[derive(Debug, Clone)]
pub struct AppError {
    pub message: String,
}

impl Display for AppError {
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}
