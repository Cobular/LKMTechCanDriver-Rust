import can
from commands import MotorOffCommand, MotorOnCommand, MotorStopCommand, OpenLoopControl, SpeedClosedLoopControl, TorqueClosedLoopControl, ReadMotorState2, ReadMotorState1AndErrorState, ReadMultiAngleLoop
import threading
from typing import Callable

from messges import Messages, parse_response


class Motor:
    def __init__(self, channel: str, bitrate: int = 500000, motor_id: int = 2, on_message_received: Callable[[Messages], None] = lambda msg: None):
        self.bus = can.interface.Bus(
            channel=channel, bustype="socketcan", bitrate=bitrate
        )
        self.motor_id = motor_id
        self.thread = threading.Thread(target=self.listen_for_responses, daemon=True)
        self.thread.start()
        self.on_message_received: Callable[[Messages], None] = on_message_received

    def send_command(self, command):
        # print(f"Sending command: {command}")
        msg = can.Message(
            arbitration_id=0x140 + self.motor_id,
            data=command.to_data_array(),
            is_extended_id=False,
        )
        self.bus.send(msg)


    def listen_for_responses(self):
        while True:
            message = self.bus.recv(timeout=1.0)  # Adjust timeout as needed
            if message:
                self.process_message(message)

    def process_message(self, message: can.Message):
        # Placeholder for message processing logic
        # This method should be customized based on how you want to handle incoming messages
        parsed_message = parse_response(message.data)
        self.on_message_received(parsed_message)

    def motor_off(self):
        command = MotorOffCommand()
        self.send_command(command)

    def motor_on(self):
        command = MotorOnCommand()
        self.send_command(command)

    def motor_stop(self):
        command = MotorStopCommand()
        self.send_command(command)

    def open_loop_control(self, power_control: int):
        command = OpenLoopControl(power_control)
        self.send_command(command)

    def torque_closed_loop_control_zero_one(self, torque: float):
        command = TorqueClosedLoopControl(torque)
        self.send_command(command)

    def refresh_motor_data(self):
        self.send_read_motor_state1_and_error_state()
        self.send_read_motor_state2()
        # self.send_read_motor_state3()
        self.send_read_multi_angle_loop()

    def send_read_motor_state1_and_error_state(self):
        command = ReadMotorState1AndErrorState()
        self.send_command(command)

    def send_read_motor_state2(self):
        command = ReadMotorState2()
        self.send_command(command)

    # def send_read_motor_state3(self):
    #     command = ReadMotorState3Command()
    #     self.send_command(command)

    def send_read_multi_angle_loop(self):
        command = ReadMultiAngleLoop()
        self.send_command(command)

    def send_speed_closed_loop(self, speed_dps: float):
        command = SpeedClosedLoopControl(speed_dps)
        self.send_command(command)


