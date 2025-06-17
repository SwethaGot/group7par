# test_camera_index.py
import cv2

for index in range(6):
    cap = cv2.VideoCapture(index)
    if cap.isOpened():
        print(f"✅ Camera found at index {index}")
        cap.release()
    else:
        print(f"❌ No camera at index {index}")
