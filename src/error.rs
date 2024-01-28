use thiserror::Error;

#[derive(Error, Debug)]
pub enum Error {
    #[error("socketcan error")]
    SocketcanError(#[from] socketcan::Error),
    #[error("stdio error")]
    StdioError(#[from] std::io::Error),
    #[error("invalid data arguments")]
    InvalidDataArguments,
    #[error("invalid response header")]
    InvalidResponseHeader,
    #[error("invalid response arguments")]
    InvalidResponseArguments,
}

pub type Result<T> = std::result::Result<T, Error>;