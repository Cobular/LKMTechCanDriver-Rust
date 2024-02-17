from dataclasses import dataclass

@dataclass
class MotorOffCommand:
    def to_data_array(self) -> bytes:
        return bytes([0x80, 0, 0, 0, 0, 0, 0, 0])

@dataclass
class OpenLoopControl:
    power_control: int

    def to_data_array(self) -> bytes:
        arr = bytearray([0xA0, 0, 0, 0, 0, 0, 0, 0])
        arr[4] = self.power_control & 0xFF
        arr[5] = (self.power_control >> 8) & 0xFF
        return bytes(arr)
    
@dataclass
class SpeedClosedLoopControl:
    speed_dps: float

    def to_data_array(self) -> bytes:
        arr = bytearray([0xA2, 0, 0, 0, 0, 0, 0, 0])
        speed = int(self.speed_dps * 100.0 * 10.0)
        arr[4] = speed & 0xFF
        arr[5] = (speed >> 8) & 0xFF
        arr[6] = (speed >> 16) & 0xFF
        arr[6] = (speed >> 16) & 0xFF
        arr[7] = (speed >> 24) & 0xFF
        return bytes(arr)
    
@dataclass
class ReadMotorState2:
    def to_data_array(self) -> bytes:
        return bytes([0x9C, 0, 0, 0, 0, 0, 0, 0])

@dataclass
class TorqueClosedLoopControl:
    torque: float  # Assuming a float value needs to be converted to the motor's data format

    def to_data_array(self) -> bytes:
        # Placeholder for conversion logic
        torque_bytes = int(self.torque * 2048).to_bytes(2, byteorder='little', signed=True)
        return bytes([0xA1, 0, 0, 0]) + torque_bytes + bytes(2)
    
@dataclass
class MotorOnCommand:
    def to_data_array(self) -> bytes:
        return bytes([0x88, 0, 0, 0, 0, 0, 0, 0])

@dataclass
class MotorStopCommand:
    def to_data_array(self) -> bytes:
        return bytes([0x81, 0, 0, 0, 0, 0, 0, 0])

@dataclass
class ClearMotorErrorState:
    def to_data_array(self) -> bytes:
        return bytes([0x9B, 0, 0, 0, 0, 0, 0, 0])


@dataclass
class ReadMotorState1AndErrorState:
    def to_data_array(self) -> bytes:
        return bytes([0x9A, 0, 0, 0, 0, 0, 0, 0])

@dataclass
class ReadMultiAngleLoop:
    def to_data_array(self) -> bytes:
        return bytes([0x92, 0, 0, 0, 0, 0, 0, 0])

