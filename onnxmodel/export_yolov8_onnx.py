#export_yolov8_480x360_onnx.py
from ultralytics import YOLO

IMG_W, IMG_H = 480, 360


model = YOLO("/home/anast/Workspaces/PhD_ws/src/SLAM_Methods/Rover-SLAM/onnxmodel/doors_and_humans.pt")   # or trained .pt

model.export(
    format="onnx",
    imgsz=(IMG_H, IMG_W),   
    opset=17,              
    dynamic=False,           
    simplify=True,
    half=True,               # FP16 weights 
    nms=True
)
print("Exported onnx model succesfully.")

