import time
from motor import Motor
import can
from messges import ClosedLoopResponse, Messages, MultiAngleResponse

angle = None
speed = None
torque = None

def on_message_received(msg: Messages) -> None:
    global angle, speed, torque
    if isinstance(msg, MultiAngleResponse):
        angle = msg.angle_deg()
    if isinstance(msg, ClosedLoopResponse):
        speed = msg.speed_dps()
        torque = msg.torque_nm()
    else:
      print(msg)

motor = Motor("can0", motor_id=2, on_message_received=on_message_received)

motor.refresh_motor_data()
time.sleep(0.1)

while True:
    print(angle, speed, torque)
    motor.refresh_motor_data()
    motor.send_speed_closed_loop(10)
    time.sleep(0.1)
