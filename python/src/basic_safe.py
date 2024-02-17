import time
from motor import Motor
import can
from messges import Messages, MultiAngleResponse


motor = Motor("can0", motor_id=2)

motor.motor_stop()
