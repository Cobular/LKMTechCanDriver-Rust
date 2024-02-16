import json
from typing import Any
from tensordict import TensorDict
import torch
import torch.nn as nn
import numpy as np

model: nn.Module = torch.load("/tmp/tmp63p6jlm8/").to("cpu")

dict = model.input_specs_inference()

print(dict)

# class TorchTensorEncoder(json.JSONEncoder):
#     def default(self, obj: Any) -> Any:
#         if torch.is_tensor(obj):
#             return {"TensorSize": obj.size()}
#         # Let the base class default method raise the TypeError
#         return json.JSONEncoder.default(self, obj)


# with open("test.json", "w") as f:
#     json.dump(dict, f, indent=4, cls=TorchTensorEncoder)

print(torch.ones((1, 1, 3), dtype=torch.float32).shape)

from ray.rllib.utils.nested_dict import NestedDict

# Assuming NestedDict is imported and available

# Original TensorDict creation and usage
# dict = TensorDict(
#     {
#         "obs": torch.ones((1, 1, 3), dtype=torch.float32),
#         "h": torch.from_numpy(np.zeros((1, 1, 256), dtype=np.float32)),
#     },
#     batch_size=1,
# )
# dict.create_nested("state_in")
# dict["state_in"]["actor"] = torch.ones((1, 1, 256), dtype=torch.float32)
# dict["state_in"]["critic"] = torch.ones((1, 1, 256), dtype=torch.float32)

# Translated to NestedDict
nested_dict = NestedDict([
  (("obs", ), torch.ones((1, 1, 3), dtype=torch.float32)),
  (("h", ), torch.from_numpy(np.zeros((1, 1, 256), dtype=np.float32)) * 0),
  (("state_in", "actor", "h"), torch.ones((1, 1, 256), dtype=torch.float32)* 5),
  (("state_in", "actor", "c"), torch.ones((1, 1, 256), dtype=torch.float32)* 5),
  (("state_in", "critic", "h"), torch.ones((1, 1, 256), dtype=torch.float32)* 2) ,
  (("state_in", "critic", "c"), torch.ones((1, 1, 256), dtype=torch.float32)* 2) ,
  (("state_in", "h"), torch.ones((1, 1, 256), dtype=torch.float32) * 3),
  (("state_in", "c"), torch.ones((1, 1, 256), dtype=torch.float32) * 4)
])
# nested_dict['h'] = torch.from_numpy(np.zeros((1, 1, 256), dtype=np.float32))
# nested_dict['state_in', 'actor'] = torch.ones((1, 1, 256), dtype=torch.float32)
# nested_dict['state_in', 'critic'] = torch.ones((1, 1, 256), dtype=torch.float32)
# nested_dict['state_in', 'h' ] = torch.ones((1, 1, 256), dtype=torch.float32)

# print(nested_dict)

res = model(nested_dict)

print(res)

# for param in model.parameters():
#     print(param.shape)

# rand_input = torch.tensor([1, 2, 3], dtype=torch.float32)
# lstm_state = torch.zeros((1, 1, 256), dtype=torch.float32)

# print(rand_input)
# print(lstm_state)

nested_dict2 = {
    "obs": torch.ones((1, 1, 3), dtype=torch.float32),
    "h": torch.from_numpy(np.zeros((1, 1, 256), dtype=np.float32)) * 0,
    "state_in": {
        "actor": {
            "h": torch.ones((1, 1, 256), dtype=torch.float32) * 5,
            "c": torch.ones((1, 1, 256), dtype=torch.float32) * 5,
        },
        "critic": {
            "h": torch.ones((1, 1, 256), dtype=torch.float32) * 2,
            "c": torch.ones((1, 1, 256), dtype=torch.float32) * 2,
        },
        "h": torch.ones((1, 1, 256), dtype=torch.float32) * 3,
        "c": torch.ones((1, 1, 256), dtype=torch.float32) * 4,
    }
}

scripted_model = torch.jit.script(model)
print(scripted_model)
jit_model = torch.jit.trace(model, nested_dict2)
print(jit_model)
torch.save(jit_model, 'model_jit.pt')
