use serde::Serialize;

use crate::error::Error;
use std::convert::TryInto;

#[derive(Serialize)]
struct EmptyResponse {}

#[derive(Serialize)]
pub struct OpenLoopResponse {
    motor_temp: u8,        // degree C
    output_power: u16,     // -850 to 850
    speed: u16,            // 1dps
    encoder_position: u16, // 15bit encoder range is 0~32767;18bit encoder range is 0~65535(keeping high 16bit, Omit the lower 2 bit).
}

impl std::convert::TryFrom<&[u8; 8]> for OpenLoopResponse {
    type Error = crate::error::Error;

    fn try_from(value: &[u8; 8]) -> Result<Self, Self::Error> {
        Ok(OpenLoopResponse {
            motor_temp: value[1],
            output_power: u16::from_le_bytes(
                value[2..4]
                    .try_into()
                    .map_err(|_e| Error::InvalidResponseArguments)?,
            ),
            speed: u16::from_le_bytes(
                value[4..6]
                    .try_into()
                    .map_err(|_e| Error::InvalidResponseArguments)?,
            ),
            encoder_position: u16::from_le_bytes(
                value[6..8]
                    .try_into()
                    .map_err(|_e| Error::InvalidResponseArguments)?,
            ),
        })
    }
}

#[derive(Serialize)]
pub struct ClosedLoopResponse {
    pub motor_temp: u8,         // degree C, 1°C/LSB
    pub torque_current_iq: i16, // range is -2048~2048
    pub speed: i16,             // 1dps/LSB
    pub encoder_position: u16, // 14bit encoder range is 0~16383; 15bit encoder range is 0~32767; 18bit encoder range is 0~65535(keeping high 16bit, Omit the lower 2 bit).
}

impl std::convert::TryFrom<&[u8; 8]> for ClosedLoopResponse {
    type Error = crate::error::Error;

    fn try_from(value: &[u8; 8]) -> Result<Self, Self::Error> {
        Ok(ClosedLoopResponse {
            motor_temp: u8::from_le_bytes([value[1]]),
            torque_current_iq: i16::from_le_bytes(
                value[2..4]
                    .try_into()
                    .map_err(|_e| Error::InvalidResponseArguments)?,
            ),
            speed: i16::from_le_bytes(
                value[4..6]
                    .try_into()
                    .map_err(|_e| Error::InvalidResponseArguments)?,
            ),
            encoder_position: u16::from_le_bytes(
                value[6..8]
                    .try_into()
                    .map_err(|_e| Error::InvalidResponseArguments)?,
            ),
        })
    }
}

#[derive(Serialize)]
pub struct MotorOffResponse(EmptyResponse);

impl std::convert::TryFrom<&[u8; 8]> for MotorOffResponse {
    type Error = crate::error::Error;

    fn try_from(_value: &[u8; 8]) -> Result<Self, Self::Error> {
        Ok(MotorOffResponse(EmptyResponse {}))
    }
}

#[derive(Serialize)]
pub struct MotorOnResponse(EmptyResponse);

impl std::convert::TryFrom<&[u8; 8]> for MotorOnResponse {
    type Error = crate::error::Error;

    fn try_from(_value: &[u8; 8]) -> Result<Self, Self::Error> {
        Ok(MotorOnResponse(EmptyResponse {}))
    }
}

#[derive(Serialize)]
pub struct MotorStopResponse(EmptyResponse);

impl std::convert::TryFrom<&[u8; 8]> for MotorStopResponse {
    type Error = crate::error::Error;

    fn try_from(_value: &[u8; 8]) -> Result<Self, Self::Error> {
        Ok(MotorStopResponse(EmptyResponse {}))
    }
}

#[derive(Serialize)]
pub struct ReadEncoderResponse {
    pub encoder: u16,
    encoder_raw: u16,
    encoder_offset: u16,
}

impl std::convert::TryFrom<&[u8; 8]> for ReadEncoderResponse {
    type Error = crate::error::Error;

    fn try_from(value: &[u8; 8]) -> Result<Self, Self::Error> {
        Ok(ReadEncoderResponse {
            encoder: u16::from_le_bytes(
                value[1..3]
                    .try_into()
                    .map_err(|_e| Error::InvalidResponseArguments)?,
            ),
            encoder_raw: u16::from_le_bytes(
                value[3..5]
                    .try_into()
                    .map_err(|_e| Error::InvalidResponseArguments)?,
            ),
            encoder_offset: u16::from_le_bytes(
                value[5..7]
                    .try_into()
                    .map_err(|_e| Error::InvalidResponseArguments)?,
            ),
        })
    }
}

#[derive(Serialize)]
#[repr(u8)]
pub enum ErrorState {
    VoltageNormal = 0,
    OverVoltageProtect = 1 << 0,
    TemperatureNormal = 1 << 3,
    OverTemperatureProtect = 1 << 3 | 1,
}

impl std::convert::TryFrom<u8> for ErrorState {
    type Error = crate::error::Error;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(ErrorState::VoltageNormal),
            1 => Ok(ErrorState::OverVoltageProtect),
            2 => Ok(ErrorState::TemperatureNormal),
            3 => Ok(ErrorState::OverTemperatureProtect),
            _ => Err(Error::InvalidResponseArguments),
        }
    }
}

#[derive(Serialize)]
pub struct ReadState1Response {
    temp: u8,
    pub voltage: u16,
    pub error_state: ErrorState,
}

impl std::convert::TryFrom<&[u8; 8]> for ReadState1Response {
    type Error = crate::error::Error;

    fn try_from(value: &[u8; 8]) -> Result<Self, Self::Error> {
        Ok(ReadState1Response {
            temp: value[1],
            voltage: u16::from_le_bytes(
                value[3..5]
                    .try_into()
                    .map_err(|_e| Error::InvalidResponseArguments)?,
            ),
            error_state: ErrorState::try_from(value[5])?,
        })
    }
}

#[derive(Serialize)]
pub struct ReadState3Response {
    temperature: u8,
    pub a_phase_current: i16,
    pub b_phase_current: i16,
    pub c_phase_current: i16,
}

impl std::convert::TryFrom<&[u8; 8]> for ReadState3Response {
    type Error = crate::error::Error;

    fn try_from(value: &[u8; 8]) -> Result<Self, Self::Error> {
        Ok(ReadState3Response {
            temperature: value[1],
            a_phase_current: i16::from_le_bytes(
                value[2..4]
                    .try_into()
                    .map_err(|_e| Error::InvalidResponseArguments)?,
            ),
            b_phase_current: i16::from_le_bytes(
                value[4..6]
                    .try_into()
                    .map_err(|_e| Error::InvalidResponseArguments)?,
            ),
            c_phase_current: i16::from_le_bytes(
                value[6..8]
                    .try_into()
                    .map_err(|_e| Error::InvalidResponseArguments)?,
            ),
        })
    }
}

#[derive(Serialize)]
pub struct MultiAngleResponse {
    pub angle: i64,
}

impl std::convert::TryFrom<&[u8; 8]> for MultiAngleResponse {
    type Error = crate::error::Error;

    fn try_from(value: &[u8; 8]) -> Result<Self, Self::Error> {
        // Construct the angle from the bytes, considering little-endian order
        let angle = i64::from_le_bytes([
            value[1], value[2], value[3], value[4], value[5], value[6], value[7], 
            // Sign-extend the 7th byte to the 8th byte of i64
            if value[7] & 0x80 == 0x80 { 0xFF } else { 0x00 },
        ]);

        Ok(MultiAngleResponse { angle })

    }
}


#[derive(Serialize)]
pub enum Response {
    MotorOff(MotorOffResponse),
    MotorOn(MotorOnResponse),
    MotorStop(MotorStopResponse),
    OpenLoop(OpenLoopResponse),
    TorqueClosedLoop(ClosedLoopResponse),
    ReadEncoder(ReadEncoderResponse),
    MultiAngle(MultiAngleResponse),
    ReadState1(ReadState1Response),
    ReadState3(ReadState3Response),
}

impl Response {
    pub fn temp(&self) -> Option<u8> {
        match self {
            Response::OpenLoop(data) => Some(data.motor_temp),
            Response::TorqueClosedLoop(data) => Some(data.motor_temp),
            Response::ReadState1(data) => Some(data.temp),
            Response::ReadState3(data) => Some(data.temperature),
            _ => None,
        }
    }
}

impl std::convert::TryFrom<&[u8; 8]> for Response {
    type Error = crate::error::Error;

    fn try_from(value: &[u8; 8]) -> Result<Self, Self::Error> {
        match value[0] {
            0x80 => Ok(Response::MotorOff(value.try_into()?)),
            0x81 => Ok(Response::MotorOn(value.try_into()?)),
            0x82 => Ok(Response::MotorStop(value.try_into()?)),
            0xA0 => Ok(Response::OpenLoop(value.try_into()?)),
            0xA1..=0xA8 | 0x9C => Ok(Response::TorqueClosedLoop(value.try_into()?)),
            0x90 => Ok(Response::ReadEncoder(value.try_into()?)),
            0x92 => Ok(Response::MultiAngle(value.try_into()?)),
            0x9A => Ok(Response::ReadState1(value.try_into()?)),
            0x9D => Ok(Response::ReadState3(value.try_into()?)),
            _ => Err(crate::error::Error::InvalidResponseHeader),
        }
    }
}
