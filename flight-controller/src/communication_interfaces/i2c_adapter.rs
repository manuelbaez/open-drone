use crate::util::error::AppError;

pub trait I2CAdapter {
    fn write_to_device(&mut self, address: u8, bytes: &[u8]) -> Result<(), AppError>;
    fn read_from_device<const BUFFER_SIZE: usize>(
        &mut self,
        address: u8,
    ) -> Result<[u8; BUFFER_SIZE], AppError>;
}
