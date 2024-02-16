import struct
from dataclasses import dataclass
from typing import Optional, Union

@dataclass
class EmptyResponse:
    pass

@dataclass
class OpenLoopResponse:
    motor_temp: int
    output_power: int
    speed: int
    encoder_position: int

@dataclass
class ClosedLoopResponse:
    motor_temp: int
    torque_current_iq: int
    speed: int
    encoder_position: int

@dataclass
class MotorOffResponse:
    response: EmptyResponse

@dataclass
class MotorOnResponse:
    response: EmptyResponse

@dataclass
class MotorStopResponse:
    response: EmptyResponse

@dataclass
class ReadEncoderResponse:
    encoder: int
    encoder_raw: int
    encoder_offset: int

@dataclass
class ErrorState:
    state: str

@dataclass
class ReadState1Response:
    temp: int
    voltage: int
    error_state: ErrorState

@dataclass
class ReadState3Response:
    temperature: int
    a_phase_current: int
    b_phase_current: int
    c_phase_current: int

@dataclass
class MultiAngleResponse:
    angle: int


def parse_response(data: bytes) -> Optional[Union[OpenLoopResponse, ClosedLoopResponse, MotorOffResponse, MotorOnResponse, MotorStopResponse, ReadEncoderResponse, ReadState1Response, ReadState3Response, MultiAngleResponse]]:
    if len(data) != 8:
        raise ValueError("Invalid data length")

    command_byte = data[0]
    if command_byte == 0x80:
        return MotorOffResponse(EmptyResponse())
    elif command_byte == 0x81:
        return MotorOnResponse(EmptyResponse())
    elif command_byte == 0x82:
        return MotorStopResponse(EmptyResponse())
    elif command_byte == 0xA0:
        motor_temp, output_power, speed, encoder_position = struct.unpack('<BHHH', data[1:])
        return OpenLoopResponse(motor_temp, output_power, speed, encoder_position)
    elif command_byte in range(0xA1, 0xA9) or command_byte == 0x9C:
        motor_temp, torque_current_iq, speed, encoder_position = struct.unpack('<BhHH', data[1:])
        return ClosedLoopResponse(motor_temp, torque_current_iq, speed, encoder_position)
    elif command_byte == 0x90:
        encoder, encoder_raw, encoder_offset = struct.unpack('<HHH', data[1:7])
        return ReadEncoderResponse(encoder, encoder_raw, encoder_offset)
    elif command_byte == 0x92:
        # Assuming the angle is stored in the last 6 bytes, similar to the Rust code
        angle = struct.unpack('<q', data[1:] + b'\x00\x00')[0]  # Sign-extend the last byte
        return MultiAngleResponse(angle)
    elif command_byte == 0x9A:
        temp, voltage = struct.unpack('<BxH', data[1:5])
        error_state = ErrorState(state=data[5])  # Simplified, assuming single byte state
        return ReadState1Response(temp, voltage, error_state)
    elif command_byte == 0x9D:
        temperature, a_phase_current, b_phase_current, c_phase_current = struct.unpack('<BhHh', data[1:])
        return ReadState3Response(temperature, a_phase_current, b_phase_current, c_phase_current)
    else:
        return None