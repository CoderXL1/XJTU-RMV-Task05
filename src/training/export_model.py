import torch
from train import TinyLeNet  # 你的模型定义
# import torch._dynamo as dynamo
import json

# dynamo.disable()
# 配置
IMG_SIZE = (32, 48)
NUM_CLASSES = 9
MODEL_PATH = "best_model.pth"
ONNX_PATH = "tiny_lenet.onnx"
CLASSES_JSON = "classes.json"

# 加载模型
model = TinyLeNet(NUM_CLASSES)
checkpoint = torch.load(MODEL_PATH, map_location="cpu")
model.load_state_dict(checkpoint['model_state'])
classes = checkpoint['classes']
model.eval()

# 保存类别列表，用于 C++ 映射
with open(CLASSES_JSON, "w") as f:
    json.dump(classes, f)

# 创建一个 dummy input
dummy_input = torch.randn(1, 1, IMG_SIZE[0], IMG_SIZE[1])

# 导出 ONNX
torch.onnx.export(
    model,
    dummy_input,
    "tiny_lenet.onnx",
    input_names=["input"],
    output_names=["output"],
    dynamic_axes={"input": {0: "batch_size"}, "output": {0: "batch_size"}},
    opset_version=11,  # 改成 18 或 17 会导致 opencv 不兼容
    verbose=False,
    dynamo=False
)


print("ONNX export done.")
