from ultralytics import YOLO
from ultralytics.yolo.v8.detect.predict import DetectionPredictor
import cv2

#Récupération de la version de YOLO pré-entraînée
model = YOLO("yolov8n.pt")

results = model.predict(source="0", show=True)

print(results)

