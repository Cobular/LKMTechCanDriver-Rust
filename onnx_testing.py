import onnx

onnx_model = onnx.load("/tmp/model2/model.onnx")
onnx.checker.check_model(onnx_model)

import onnxruntime as ort
import numpy as np

model = ort.InferenceSession("/tmp/model2/model.onnx")

input_all = [node for node in onnx_model.graph.input]
print('Inputs: ', input_all)
input_initializer =  [node.name for node in onnx_model.graph.initializer]
print('Initializers: ', input_initializer)

outputs =  model.run(
    None,
    {
      "obs": np.array([[1, 2, 3]], dtype=np.float32),
      "state_ins": np.ones((256), dtype=np.float32),
    },
)

print(outputs)