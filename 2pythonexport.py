from ultralytics import YOLO
import cv2

model = YOLO("yolov8n.pt")
img = cv2.imread("/home/anast/Desktop/test.jpg")
results = model(img)
print(results)

