import time
from motor import Motor
import can
from messges import Messages, MultiAngleResponse

def on_message_received(msg: Messages) -> None:
    if isinstance(msg, MultiAngleResponse):
        print(msg.angle_deg())
    else:
      print(msg)

motor = Motor("can0", motor_id=2, on_message_received=on_message_received)


while True:
    motor.refresh_motor_data()
    time.sleep(0.01)

