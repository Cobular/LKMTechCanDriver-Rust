import time
from messges import ClosedLoopResponse, Messages, MultiAngleResponse
from motor import Motor
import torch
from ray.rllib.utils.nested_dict import NestedDict
import numpy as np

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
      # print(msg)
      pass

motor = Motor("can0", motor_id=2, on_message_received=on_message_received)

model = torch.load("model.pt", map_location="cpu")

nested_dict = NestedDict([
  (("obs", ), torch.zeros((1, 1, 3), dtype=torch.float32)),
  (("state_in", "actor", "h"), torch.zeros((1, 1, 128), dtype=torch.float32)),
  (("state_in", "actor", "c"), torch.zeros((1, 1, 128), dtype=torch.float32)),
  (("state_in", "critic", "h"), torch.zeros((1, 1, 128), dtype=torch.float32)),
  (("state_in", "critic", "c"), torch.zeros((1, 1, 128), dtype=torch.float32)),
])

state_out = nested_dict["state_in"]

def update_dict(angle_degs: float, speed_dps: float, torque: float, state_out: NestedDict):
  nested_dict["obs"] = torch.tensor([[[speed_dps, angle_degs, torque]]], dtype=torch.float32)
  nested_dict["state_in"] = state_out

# Controller loop speed is 100hz, 
# but we need to make sure we have fresh data from right before 
# so we run the refresh loop at 500hz and skip every 5
counter = 0
while True:
  counter += 1
  motor.refresh_motor_data()
  if counter % 5 != 0:
    time.sleep(1/500)
    continue

  if angle is None or speed is None or torque is None:
    print("One of the inputs is none, can't continue", angle, speed, torque)
    continue

  print(torque)
  update_dict(angle, speed, torque, state_out)

  output = model(nested_dict)

  action = output["action_dist_inputs"].detach().numpy()
  state_out = output["state_out"]

  print(f"Requesting torque: {action[0,0,0]}")

  motor.torque_closed_loop_control_zero_one(action[0,0,0])

  time.sleep(1/500)
