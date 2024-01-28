use std::io;

use socketcan::{
    tokio::CanSocket, CanFilter, CanFrame, EmbeddedFrame, Id, Result, SocketOptions, StandardId,
};

pub mod messages;

pub struct Motor {
    socket: CanSocket,
    id: Id,
}

impl Motor {
    pub fn new(socket_name: &str, id: u8) -> io::Result<Self> {
        let socket = CanSocket::open(socket_name)?;

        socket.set_filters(&[CanFilter::new(0x140 + id as u32, u32::MAX)])?;

        let id = Id::Standard(
            StandardId::new(0x140 + id as u16)
                .ok_or_else(|| io::Error::new(io::ErrorKind::Other, "Invalid ID"))?,
        );

        Ok(Self { socket, id })
    }

    async fn send_message(&mut self, message: &[u8]) -> Result<()> {
        let frame = CanFrame::new(self.id, message)
            .ok_or_else(|| io::Error::new(io::ErrorKind::Other, "Invalid frame"))?;
        self.socket.write_frame(frame)?.await?;
        Ok(())
    }

    pub async fn send_motor_off(&mut self) -> Result<()> {
        let mut data = [0u8; 8];
        data[0] = 0x80; // Command byte for motor off
        self.send_message(&data).await?;
        Ok(())
    }
}
