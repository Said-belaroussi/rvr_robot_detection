#! /usr/bin/env python3

from pathlib import Path
import torch
from torch import nn
import kornia
import onnx
from onnxsim import simplify
import blobconverter

if len(sys.argv) < 2:
    print("usage: Python3 ./utils/pytoch_to_blob pytorch_file_path")
    exit(1)
else:
    model_name = sys.argv[1]
name = 'blur'

# class Model(nn.Module):
#     def forward(self, image):
#         return kornia.filters.gaussian_blur2d(image, (9, 9), (2.5, 2.5))
#
# # Define the expected input shape (dummy input)
# shape = (1, 3, 300, 300)
# model = Model()
# X = torch.ones(shape, dtype=torch.float32)

# model = torchvision.models.resnet18(pretrained=True)
model = torch.load("./yolov5s.pt")

# Define the expected input shape
shape = (1, 3, 224, 224)
X = torch.randn(shape) # Define dummy input

path = Path("out/")
path.mkdir(parents=True, exist_ok=True)
onnx_path = str(path / (name + '.onnx'))

print(f"Writing to {onnx_path}")
torch.onnx.export(
    model,
    X,
    onnx_path,
    opset_version=12,
    do_constant_folding=True,
)

onnx_simplified_path = str(path / (name + '_simplified.onnx'))

# Use onnx-simplifier to simplify the onnx model
onnx_model = onnx.load(onnx_path)
model_simp, check = simplify(onnx_model)
onnx.save(model_simp, onnx_simplified_path)

# Use blobconverter to convert onnx->IR->blob
blobconverter.from_onnx(
    model=onnx_simplified_path,
    data_type="FP16",
    shaves=6,
    use_cache=False,
    output_dir="../models",
    optimizer_params=[]
)