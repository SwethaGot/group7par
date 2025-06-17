import cv2
import mediapipe as mp
import numpy as np
import time
import requests
from datetime import datetime
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class UR5eHandSafetySystem(Node):
    def __init__(self, web_server_url="http://localhost:5000"):
        super().__init__('ros_hand_safety_interactive')
        self.web_server_url = web_server_url
        self.bridge = CvBridge()
        self.frame = None
        self.lock = False

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils

        self.hazard_active = False
        self.last_hazard_time = 0
        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.current_fps = 0

        print("🛡️ UR5e Hand Safety System - Group 07")
        print(f"🌐 Web Interface: {self.web_server_url}")
        print("✋ Hand Detection: Real-time safety monitoring")

    def send_to_web_interface(self, endpoint, data):
        try:
            url = f"{self.web_server_url}/api/{endpoint}"
            requests.post(url, json=data, timeout=1)
        except Exception as e:
            print(f"⚠️ Web interface error: {e}")

    def image_callback(self, msg):
        if self.lock:
            return
        self.lock = True
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame, hands_detected, hand_info = self.detect_hands(frame)

            if hands_detected:
                self.trigger_hazard_protocol(hand_info)
            else:
                self.clear_hazard_protocol()

            final_frame = self.add_clean_overlay(frame)
            self.update_fps()
            cv2.imshow('UR5e Hand Safety - Group 07', final_frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                rclpy.shutdown()
                cv2.destroyAllWindows()
            elif key == ord('s'):
                filename = f"group07_safety_{int(time.time())}.jpg"
                cv2.imwrite(filename, final_frame)
                print(f"📸 Saved: {filename}")
            elif key == ord('r'):
                self.hazard_active = False
                print(f"🔄 Hazard reset")

        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")
        finally:
            self.lock = False

    def detect_hands(self, frame):
        try:
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hands.process(rgb_frame)
            hands_detected = False
            hand_info = []

            if results.multi_hand_landmarks:
                hands_detected = True
                for idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
                    self.mp_drawing.draw_landmarks(
                        frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS,
                        self.mp_drawing.DrawingSpec(color=(255, 255, 255), thickness=2, circle_radius=2),
                        self.mp_drawing.DrawingSpec(color=(255, 0, 0), thickness=2)
                    )
                    h, w, _ = frame.shape
                    x_min = min([lm.x for lm in hand_landmarks.landmark]) * w
                    y_min = min([lm.y for lm in hand_landmarks.landmark]) * h
                    x_max = max([lm.x for lm in hand_landmarks.landmark]) * w
                    y_max = max([lm.y for lm in hand_landmarks.landmark]) * h
                    cv2.rectangle(frame, (int(x_min)-20, int(y_min)-20),
                                        (int(x_max)+20, int(y_max)+20), (255, 0, 0), 3)
                    cv2.putText(frame, "HAZARD", (int(x_min), int(y_min)-25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                    hand_info.append({
                        'label': results.multi_handedness[idx].classification[0].label,
                        'confidence': results.multi_handedness[idx].classification[0].score,
                        'bbox': [int(x_min), int(y_min), int(x_max), int(y_max)]
                    })

            return frame, hands_detected, hand_info
        except Exception as e:
            self.get_logger().error(f"Hand detection error: {e}")
            return frame, False, []

    def trigger_hazard_protocol(self, hand_info):
        now = time.time()
        if now - self.last_hazard_time < 1.0:
            return
        self.hazard_active = True
        self.last_hazard_time = now
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(f"🚨 HAZARD DETECTED at {timestamp}")
        self.send_to_web_interface('hazard', {
            'type': 'human_detected',
            'message': 'Hand detected in work area',
            'timestamp': timestamp,
            'hands': hand_info
        })

    def clear_hazard_protocol(self):
        if not self.hazard_active:
            return
        self.hazard_active = False
        timestamp = datetime.now().strftime("%H:%M:%S")
        print(f"✅ [{timestamp}] SAFETY ALL CLEAR")
        self.send_to_web_interface('clear_hazard', {
            'type': 'area_clear',
            'message': 'Work area secure',
            'timestamp': timestamp
        })

    def update_fps(self):
        self.fps_counter += 1
        now = time.time()
        if now - self.fps_start_time >= 1.0:
            self.current_fps = self.fps_counter
            self.fps_counter = 0
            self.fps_start_time = now

    def add_clean_overlay(self, frame):
        h, w = frame.shape[:2]
        overlay = frame.copy()
        color = (0, 0, 255) if self.hazard_active else (0, 255, 0)
        label = "HAZARD DETECTED" if self.hazard_active else "MONITORING ACTIVE"
        cv2.rectangle(overlay, (0, 0), (w, h), color, 8)
        cv2.putText(overlay, label, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
        cv2.putText(overlay, "GROUP 07", (w-120, h-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(overlay, f"FPS: {self.current_fps}", (20, h-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        return cv2.addWeighted(overlay, 0.7, frame, 0.3, 0)

def main():
    rclpy.init()
    node = UR5eHandSafetySystem()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()