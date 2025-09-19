from ultralytics import YOLO
model = YOLO("onnxmodel/doorshumans.onnx")
results = model("/home/anast/Workspaces/Datasets/cafe1-2/color/1560025133.842223.png")
results.print()
