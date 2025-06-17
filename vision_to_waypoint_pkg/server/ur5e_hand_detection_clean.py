import cv2
import mediapipe as mp
import numpy as np
import time
import requests
import json
import threading
from datetime import datetime

class UR5eHandSafetySystem:
    def __init__(self, web_server_url="http://localhost:5555"):
        """
        UR5e Hand Safety System - Clean Version for Group 07
        """
        # Web server connection
        self.web_server_url = web_server_url
        
        # Hand Detection (MediaPipe)
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils
        
        # System state
        self.monitoring_active = False
        self.hazard_active = False
        self.last_hazard_time = 0
        
        # Performance tracking
        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.current_fps = 0
        
        print("🛡️ UR5e Hand Safety System - Group 07")
        print(f"🌐 Web Interface: {self.web_server_url}")
        print("✋ Hand Detection: Real-time safety monitoring")
        print("🎯 Clean minimalistic interface")
    
    def send_to_web_interface(self, endpoint, data):
        """Send data to web interface"""
        try:
            url = f"{self.web_server_url}/api/{endpoint}"
            response = requests.post(url, json=data, timeout=1)
            return response.status_code == 200
        except Exception as e:
            print(f"⚠️ Web interface error: {e}")
            return False
    
    def detect_hands(self, frame):
        """Detect hands and trigger safety protocols"""
        try:
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hands.process(rgb_frame)
            
            hands_detected = False
            hand_info = []
            
            if results.multi_hand_landmarks:
                hands_detected = True
                
                for idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
                    # Draw hand landmarks in white/red for clean look
                    self.mp_drawing.draw_landmarks(
                        frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS,
                        self.mp_drawing.DrawingSpec(color=(255, 255, 255), thickness=2, circle_radius=2),
                        self.mp_drawing.DrawingSpec(color=(255, 0, 0), thickness=2)
                    )
                    
                    # Get hand info
                    hand_label = results.multi_handedness[idx].classification[0].label
                    hand_score = results.multi_handedness[idx].classification[0].score
                    
                    # Calculate bounding box
                    h, w, _ = frame.shape
                    x_min = min([lm.x for lm in hand_landmarks.landmark]) * w
                    y_min = min([lm.y for lm in hand_landmarks.landmark]) * h
                    x_max = max([lm.x for lm in hand_landmarks.landmark]) * w
                    y_max = max([lm.y for lm in hand_landmarks.landmark]) * h
                    
                    # Draw clean danger zone
                    cv2.rectangle(frame, 
                                (int(x_min)-20, int(y_min)-20), 
                                (int(x_max)+20, int(y_max)+20), 
                                (255, 0, 0), 3)
                    
                    # Add clean warning label
                    cv2.putText(frame, f"HAZARD: {hand_label}", 
                               (int(x_min), int(y_min)-25),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                    
                    hand_info.append({
                        'label': hand_label,
                        'confidence': hand_score,
                        'bbox': [int(x_min), int(y_min), int(x_max), int(y_max)]
                    })
            
            return frame, hands_detected, hand_info
            
        except Exception as e:
            print(f"Hand detection error: {e}")
            return frame, False, []
    
    def trigger_hazard_protocol(self, hand_info):
        """Trigger hazard safety protocol"""
        current_time = time.time()
        
        # Avoid spam alerts
        if current_time - self.last_hazard_time < 1.0:
            return
        
        self.hazard_active = True
        self.last_hazard_time = current_time
        
        # Clean console alerts
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(f"\n🚨 ===== HAZARD PROTOCOL ===== 🚨")
        print(f"⏰ {timestamp}")
        print(f"🛑 HAZARD.. STOPPING")
        print(f"🛑 HAZARD.. STOPPING") 
        print(f"🛑 HAZARD.. STOPPING")
        print(f"✋ HAND DETECTED IN UR5E WORK AREA")
        print(f"🔴 ROBOT OPERATIONS STOPPED")
        
        for i, hand in enumerate(hand_info):
            print(f"   👤 {hand['label']} hand ({hand['confidence']:.2f})")
        
        print(f"🚨 ========================= 🚨\n")
        
        # Send to web interface
        self.send_to_web_interface('hazard', {
            'type': 'human_detected',
            'message': 'Hand detected in work area',
            'timestamp': timestamp,
            'hands': hand_info
        })
    
    def clear_hazard_protocol(self):
        """Clear hazard when area is safe"""
        if not self.hazard_active:
            return
        
        self.hazard_active = False
        
        timestamp = datetime.now().strftime("%H:%M:%S")
        print(f"✅ [{timestamp}] SAFETY ALL CLEAR")
        print(f"🟢 Work area secure")
        print(f"🤖 Operations may continue\n")
        
        # Send to web interface
        self.send_to_web_interface('clear_hazard', {
            'type': 'area_clear',
            'message': 'Work area secure',
            'timestamp': timestamp
        })
    
    def update_fps(self):
        """Update FPS counter"""
        self.fps_counter += 1
        current_time = time.time()
        
        if current_time - self.fps_start_time >= 1.0:
            self.current_fps = self.fps_counter
            self.fps_counter = 0
            self.fps_start_time = current_time
    
    def add_clean_overlay(self, frame):
        """Add clean minimalistic overlay"""
        height, width = frame.shape[:2]
        overlay = frame.copy()
        
        # Status panel
        if self.hazard_active:
            # Red border for hazard
            cv2.rectangle(overlay, (0, 0), (width, height), (0, 0, 255), 8)
            # Status text
            cv2.putText(overlay, "HAZARD DETECTED", (20, 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 2)
        else:
            # Green border for safe
            cv2.rectangle(overlay, (0, 0), (width, height), (0, 255, 0), 3)
            # Status text
            cv2.putText(overlay, "MONITORING ACTIVE", (20, 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        # Group 07 watermark
        cv2.putText(overlay, "GROUP 07", (width-120, height-20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # FPS counter
        cv2.putText(overlay, f"FPS: {self.current_fps}", (20, height-20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Blend overlay
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
        
        return frame
    
    def run_safety_system(self, camera_index=0):
        """Run the UR5e hand safety system"""
        print(f"\n🤖 UR5E HAND SAFETY SYSTEM - GROUP 07")
        print(f"💻 MacBook Pro M1 Camera")
        print(f"🛡️ Real-time hand detection")
        print(f"🌐 Web Interface: {self.web_server_url}")
        print(f"🎯 Clean minimalistic design")
        print("-" * 50)
        
        # Test web server connection
        try:
            response = requests.get(f"{self.web_server_url}/api/status", timeout=2)
            print(f"✅ Web server connected")
        except:
            print(f"⚠️ Web server not available")
            print(f"💡 Start: python ur5e_web_server_clean.py")
        
        # Initialize camera
        cap = cv2.VideoCapture(camera_index)
        
        if not cap.isOpened():
            print(f"❌ Error: Could not open camera")
            return
        
        # Configure camera
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        
        print(f"✅ Camera initialized: 640x480")
        print(f"🟢 Safety monitoring active")
        print(f"📱 Web interface: http://localhost:5555")
        print(f"⚠️  Wave hand to test detection")
        
        self.monitoring_active = True
        
        while self.monitoring_active:
            ret, frame = cap.read()
            if not ret:
                print("❌ Camera frame error")
                break
            
            # Hand safety detection
            frame, hands_detected, hand_info = self.detect_hands(frame)
            
            if hands_detected:
                self.trigger_hazard_protocol(hand_info)
            else:
                self.clear_hazard_protocol()
            
            # Add clean overlay
            final_frame = self.add_clean_overlay(frame)
            
            # Update FPS
            self.update_fps()
            
            # Display frame
            cv2.imshow('UR5e Hand Safety - Group 07', final_frame)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q') or key == ord('Q'):
                print("\n🛑 Shutting down safety system...")
                break
            elif key == ord('s') or key == ord('S'):
                filename = f'group07_safety_{int(time.time())}.jpg'
                cv2.imwrite(filename, final_frame)
                print(f"📸 Saved: {filename}")
            elif key == ord('r') or key == ord('R'):
                self.hazard_active = False
                print(f"🔄 Hazard reset")
        
        # Cleanup
        self.monitoring_active = False
        cap.release()
        cv2.destroyAllWindows()
        print("🔒 Safety system shutdown complete")


def main():
    """Main function"""
    print("🤖 UR5E BOARD GAME HAND SAFETY - GROUP 07")
    print("=" * 50)
    print("🎯 Programming Autonomous Robots Assignment")
    
    print("\n🔧 FEATURES:")
    print("   • Real-time hand detection")
    print("   • Live camera feed in web UI")
    print("   • Clean black & white design") 
    print("   • Game board image display")
    print("   • Emergency stop protocols")
    
    print("\n💡 USAGE:")
    print("   1. Start web server: python ur5e_web_server_clean.py")
    print("   2. Start this system: python ur5e_hand_detection_clean.py")
    print("   3. Open: http://localhost:5555")
    print("   4. Click START to begin monitoring")
    print("   5. Wave hand to test safety detection")
    
    # Get web server URL
    web_url = input(f"\nWeb server URL (default: http://localhost:5555): ").strip()
    if not web_url:
        web_url = "http://localhost:5555"
    
    confirm = input(f"\nStart Group 07 hand safety system? (y/N): ").strip().lower()
    
    if confirm in ['y', 'yes']:
        system = UR5eHandSafetySystem(web_url)
        
        try:
            system.run_safety_system()
        except KeyboardInterrupt:
            print("\n⚠️ System interrupted")
        except Exception as e:
            print(f"❌ System error: {e}")
    else:
        print("👋 Group 07 system cancelled")


if __name__ == "__main__":
    main()