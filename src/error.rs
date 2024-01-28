use thiserror::Error;

use socketcan::Error;

#[derive(Error, Debug)]
pub struct Error {
    #[from]
    pub socketcan_error: socketcan::Error,
    #[from]
    pub std_io_error: std::io::Error,
    #[error("invalid data arguments")]
    InvalidDataArguments,
}

pub type Result<T> = std::result::Result<T, Error>;