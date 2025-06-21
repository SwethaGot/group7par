# test_yolo_apriltag.py
from ultralytics import YOLO
import cv2

# Load trained model (or use yolov8n.pt for test)
model = YOLO("yolov8n.pt")  # replace with your trained model path if available

# Read test image (replace with your own)
img = cv2.imread("test_apriltag.jpg")
results = model(img)

# Plot and show results
annotated = results[0].plot()
cv2.imshow("YOLOv8 Detection", annotated)
cv2.waitKey(0)
cv2.destroyAllWindows()
