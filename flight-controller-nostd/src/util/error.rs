use core::fmt::{self, Debug, Display, Formatter};

#[derive(Debug)]
pub struct AppError<E> {
    pub message: &'static str,
    pub error: E,
}

impl<E> Display for AppError<E>
where
    E: Debug,
{
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        write!(f, "{} {:?}", self.message, self.error)
    }
}
