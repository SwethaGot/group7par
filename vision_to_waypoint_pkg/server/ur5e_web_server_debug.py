from flask import Flask, render_template_string, jsonify, request, Response, send_file
import time
import threading
import cv2
import os
import numpy as np
from datetime import datetime
from ultralytics import YOLO

app = Flask(__name__)

# Global system state
system_state = {
    'detection_enabled': False,
    'hazard_detected': False,
    'safety_status': 'idle',
    'camera_fps': 0,
    'hazards_detected': 0,
    'last_update': datetime.now().isoformat(),
    'logs': []
}

class HandDetectionSystem:
    def __init__(self):
        self.frame = None
        self.processed_frame = None
        self.lock = threading.Lock()
        self.cap = None
        self.running = False
        self.fps_counter = 0
        self.fps_start = time.time()
        
        # Detection state
        self.model = None
        self.detection_enabled = False
        self.hazard_active = False
        self.last_alert_time = 0
        
        self.load_yolo_model()
        self.start_camera()

    def load_yolo_model(self):
        """Load YOLOv8s model"""
        try:
            model_path = 'yolov8s.pt'
            if os.path.exists(model_path):
                print(f"✅ Loading YOLOv8s model from {model_path}")
                self.model = YOLO(model_path)
                print("✅ YOLOv8s model loaded successfully")
            else:
                print("📥 Downloading YOLOv8s model...")
                self.model = YOLO('yolov8s.pt')
                print("✅ YOLOv8s model downloaded and loaded")
        except Exception as e:
            print(f"❌ Failed to load YOLOv8s model: {e}")
            self.model = None

    def start_camera(self):
        """Start camera capture"""
        try:
            self.cap = cv2.VideoCapture(0)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            
            if self.cap.isOpened():
                self.running = True
                self.thread = threading.Thread(target=self.camera_loop, daemon=True)
                self.thread.start()
                print("✅ Camera started")
            else:
                print("❌ Camera failed to open")
        except Exception as e:
            print(f"❌ Camera error: {e}")

    def detect_hands(self, frame):
        """Detect hands using YOLO person detection"""
        if not self.model or not self.detection_enabled:
            return frame, []
        
        try:
            results = self.model(frame, verbose=False)
            hazards = []
            
            for result in results:
                if result.boxes is not None:
                    for box in result.boxes:
                        class_id = int(box.cls[0])
                        confidence = float(box.conf[0])
                        
                        # Person class = 0, treat as hand hazard if confidence > 0.5 (lowered for better model)
                        if class_id == 0 and confidence > 0.5:  # Lowered from 0.6 since better model
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                            
                            # Draw RED detection box for hand hazard
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 4)
                            cv2.putText(frame, f"HAZARD: HANDS {confidence:.2f}", 
                                       (x1, y1-15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                            
                            hazards.append({
                                'type': 'hands',
                                'confidence': confidence,
                                'bbox': [x1, y1, x2, y2]
                            })
            
            return frame, hazards
            
        except Exception as e:
            print(f"Detection error: {e}")
            return frame, []

    def add_status_overlay(self, frame):
        """Add status overlay to frame"""
        height, width = frame.shape[:2]
        
        if not self.detection_enabled:
            # Detection disabled - blue border
            cv2.rectangle(frame, (0, 0), (width, height), (255, 255, 0), 3)
            cv2.putText(frame, "DETECTION DISABLED - CLICK START", (20, 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        elif self.hazard_active:
            # Hazard detected - red border
            cv2.rectangle(frame, (0, 0), (width, height), (0, 0, 255), 8)
            cv2.putText(frame, "HAND HAZARD DETECTED", (20, 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
        else:
            # Monitoring active - green border
            cv2.rectangle(frame, (0, 0), (width, height), (0, 255, 0), 3)
            cv2.putText(frame, "HAND MONITORING ACTIVE", (20, 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        
        # Info overlay
        cv2.putText(frame, "GROUP 07 - HAND DETECTION", (width-280, height-50),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"FPS: {system_state['camera_fps']}", (20, height-50),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"Hazards: {system_state['hazards_detected']}", (20, height-25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return frame

    def trigger_hazard(self, hazards):
        """Trigger hazard protocol"""
        current_time = time.time()
        
        if current_time - self.last_alert_time < 2.0:
            return
        
        self.hazard_active = True
        self.last_alert_time = current_time
        
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        print(f"\n🚨 ===== HAND HAZARD DETECTED ===== 🚨")
        print(f"⏰ {timestamp}")
        print(f"🛑 ROBOT EMERGENCY STOP")
        print(f"✋ {len(hazards)} HAND HAZARD(S) DETECTED")
        
        for i, hazard in enumerate(hazards):
            print(f"   Hazard {i+1}: {hazard['confidence']:.2f} confidence")
        
        print(f"🚨 ========================= 🚨\n")
        
        # Update system state
        system_state['hazard_detected'] = True
        system_state['safety_status'] = 'hazard'
        system_state['hazards_detected'] = len(hazards)
        system_state['last_update'] = datetime.now().isoformat()
        
        # Add to logs
        self.add_log('🚨 HAND HAZARD', f'{len(hazards)} hand hazard(s) detected', 'error')

    def clear_hazard(self):
        """Clear hazard when safe"""
        if not self.hazard_active:
            return
        
        self.hazard_active = False
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        print(f"✅ [{timestamp}] AREA CLEAR - SAFE TO CONTINUE")
        
        # Update system state
        system_state['hazard_detected'] = False
        system_state['safety_status'] = 'safe'
        system_state['hazards_detected'] = 0
        system_state['last_update'] = datetime.now().isoformat()
        
        # Add to logs
        self.add_log('✅ Area Clear', 'Work area secure - no hands detected', 'success')

    def start_detection(self):
        """Start detection monitoring"""
        self.detection_enabled = True
        system_state['detection_enabled'] = True
        system_state['safety_status'] = 'monitoring'
        system_state['last_update'] = datetime.now().isoformat()
        print("🛡️ Hand detection monitoring STARTED")
        self.add_log('🛡️ Detection Started', 'Hand detection monitoring activated', 'success')
        
    def stop_detection(self):
        """Stop detection monitoring"""
        self.detection_enabled = False
        self.hazard_active = False
        system_state['detection_enabled'] = False
        system_state['hazard_detected'] = False
        system_state['safety_status'] = 'idle'
        system_state['hazards_detected'] = 0
        system_state['last_update'] = datetime.now().isoformat()
        print("🛑 Hand detection monitoring STOPPED")
        self.add_log('🛑 Detection Stopped', 'Hand detection monitoring disabled', 'warning')

    def add_log(self, title, message, type_='info'):
        """Add log entry"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = {
            'timestamp': timestamp,
            'title': title,
            'message': message,
            'type': type_
        }
        system_state['logs'].insert(0, log_entry)
        
        # Keep only last 50 logs
        if len(system_state['logs']) > 50:
            system_state['logs'] = system_state['logs'][:50]

    def camera_loop(self):
        """Main camera loop"""
        while self.running and self.cap and self.cap.isOpened():
            try:
                ret, frame = self.cap.read()
                if ret:
                    # Store original frame
                    with self.lock:
                        self.frame = frame.copy()
                    
                    # Run detection
                    detected_frame, hazards = self.detect_hands(frame.copy())
                    
                    # Handle hazard state
                    if self.detection_enabled:
                        if len(hazards) > 0:
                            self.trigger_hazard(hazards)
                        else:
                            self.clear_hazard()
                    
                    # Add status overlay
                    final_frame = self.add_status_overlay(detected_frame)
                    
                    # Encode for streaming
                    ret_encode, jpeg = cv2.imencode('.jpg', final_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    if ret_encode:
                        with self.lock:
                            self.processed_frame = jpeg.tobytes()
                    
                    # Update FPS
                    self.fps_counter += 1
                    if time.time() - self.fps_start >= 1.0:
                        system_state['camera_fps'] = self.fps_counter
                        self.fps_counter = 0
                        self.fps_start = time.time()
                            
                time.sleep(0.033)  # ~30 FPS
                
            except Exception as e:
                print(f"⚠️ Camera error: {e}")
                time.sleep(0.1)

    def get_frame(self):
        """Get latest processed frame"""
        with self.lock:
            return self.processed_frame

    def stop(self):
        """Stop camera"""
        self.running = False
        if self.cap:
            self.cap.release()

# Initialize detection system
detector = HandDetectionSystem()

@app.route('/gameboard-<int:num>.png')
def serve_gameboard(num):
    """Serve gameboard images"""
    try:
        filename = f'gameboard-{num}.png'
        if os.path.exists(filename):
            return send_file(filename, mimetype='image/png')
        return Response('Image not found', status=404)
    except:
        return Response('Error', status=404)

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    def generate():
        while True:
            frame = detector.get_frame()
            if frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')
            time.sleep(0.05)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/status')
def get_status():
    """Get current system status"""
    return jsonify(system_state)

@app.route('/api/start_detection', methods=['POST'])
def start_detection():
    """Start detection via HTTP"""
    detector.start_detection()
    return jsonify({'status': 'started', 'message': 'Hand detection started'})

@app.route('/api/stop_detection', methods=['POST'])
def stop_detection():
    """Stop detection via HTTP"""
    detector.stop_detection()
    return jsonify({'status': 'stopped', 'message': 'Hand detection stopped'})

@app.route('/api/emergency_stop', methods=['POST'])
def emergency_stop():
    """Emergency stop via HTTP"""
    detector.stop_detection()
    system_state['safety_status'] = 'emergency_stop'
    detector.add_log('🛑 EMERGENCY STOP', 'Manual emergency stop activated', 'error')
    return jsonify({'status': 'emergency_stopped', 'message': 'Emergency stop activated'})

@app.route('/')
def index():
    """Main interface"""
    return render_template_string("""
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>UR5e Hand Detection - Group 07</title>
        <style>
            * { margin: 0; padding: 0; box-sizing: border-box; }
            body { 
                font-family: -apple-system, BlinkMacSystemFont, sans-serif;
                background: #000; color: #fff; min-height: 100vh;
            }
            .header { 
                text-align: center; padding: 20px; background: #111;
                border-bottom: 2px solid #333;
            }
            .header h1 { 
                font-size: 2.2rem; font-weight: 300; margin-bottom: 8px;
                color: #fff; letter-spacing: 2px;
            }
            .subtitle { 
                font-size: 1rem; color: #888; font-weight: 300;
            }
            .group-badge {
                position: absolute; top: 20px; right: 20px;
                background: #fff; color: #000; padding: 10px 20px;
                border-radius: 25px; font-weight: 700; font-size: 0.9rem;
            }
            .main-grid {
                display: grid; grid-template-columns: 1fr 1fr 350px; gap: 2px;
                min-height: calc(100vh - 200px); background: #333;
            }
            .panel {
                background: #000; padding: 30px; display: flex;
                flex-direction: column; align-items: center; justify-content: center;
            }
            .panel-title {
                font-size: 1.3rem; font-weight: 300; margin-bottom: 25px;
                color: #fff; letter-spacing: 1px; text-align: center;
            }
            
            .gameboard-frame {
                width: 100%; max-width: 420px; 
                background: #fff; border-radius: 15px; padding: 25px;
                box-shadow: 0 8px 30px rgba(255,255,255,0.1);
            }
            .gameboard-container {
                background: #000; border-radius: 10px; overflow: hidden;
            }
            .gameboard-half {
                width: 100%; height: auto; display: block;
            }
            .gameboard-spacer {
                height: 10px; background: #fff;
            }
            .image-error {
                height: 150px; background: #222; border: 1px solid #444;
                display: flex; align-items: center; justify-content: center;
                color: #666; font-size: 0.8rem; margin: 3px 0;
            }
            
            .camera-container {
                width: 100%; max-width: 420px; background: #111;
                border-radius: 15px; overflow: hidden; border: 2px solid #333;
                position: relative;
            }
            .camera-feed {
                width: 100%; height: auto; display: block;
            }
            .camera-status {
                position: absolute; top: 15px; left: 15px;
                background: rgba(255, 255, 0, 0.8); padding: 8px 15px;
                border-radius: 8px; font-size: 0.85rem; font-weight: 600;
            }
            .camera-status.monitoring {
                background: rgba(0, 255, 0, 0.8);
            }
            .camera-status.hazard {
                background: rgba(255, 0, 0, 0.8);
                animation: pulse 1s infinite;
            }
            @keyframes pulse {
                0%, 100% { opacity: 1; }
                50% { opacity: 0.6; }
            }
            
            .logs-panel {
                background: #000; padding: 25px; display: flex;
                flex-direction: column; max-height: calc(100vh - 200px); overflow: hidden;
            }
            .logs-header {
                font-size: 1.1rem; font-weight: 400; margin-bottom: 20px;
                color: #fff; text-align: center; border-bottom: 1px solid #333;
                padding-bottom: 15px;
            }
            .logs-container {
                flex: 1; overflow-y: auto;
            }
            .log-entry {
                margin-bottom: 12px; padding: 15px; border-radius: 10px;
                border-left: 4px solid; font-size: 0.9rem; line-height: 1.5;
                animation: slideIn 0.4s ease-out;
            }
            .log-entry.success {
                background: rgba(0, 255, 0, 0.1); border-color: #0f0; color: #ccffcc;
            }
            .log-entry.error {
                background: rgba(255, 0, 0, 0.1); border-color: #f00; color: #ffcccc;
            }
            .log-entry.warning {
                background: rgba(255, 255, 0, 0.1); border-color: #ff0; color: #ffffcc;
            }
            .log-entry.info {
                background: rgba(0, 150, 255, 0.1); border-color: #09f; color: #ccddff;
            }
            .log-timestamp {
                font-size: 0.75rem; opacity: 0.7; float: right;
            }
            .log-title {
                font-weight: 600; margin-bottom: 5px;
            }
            @keyframes slideIn {
                from { opacity: 0; transform: translateX(20px); }
                to { opacity: 1; transform: translateX(0); }
            }
            
            .status-bar {
                position: fixed; bottom: 0; left: 0; right: 0;
                background: #111; border-top: 2px solid #333; padding: 20px;
                display: flex; justify-content: space-between; align-items: center;
                z-index: 1000;
            }
            .status-left, .status-right {
                display: flex; align-items: center; gap: 25px;
            }
            .status-indicator {
                width: 12px; height: 12px; border-radius: 50%;
                background: #ff0; animation: blink 2s infinite;
            }
            .status-indicator.monitoring {
                background: #0f0;
            }
            .status-indicator.hazard {
                background: #f00; animation: fastBlink 0.5s infinite;
            }
            @keyframes blink { 0%, 100% { opacity: 1; } 50% { opacity: 0.4; } }
            @keyframes fastBlink { 0%, 100% { opacity: 1; } 50% { opacity: 0.2; } }
            .status-text {
                font-size: 1rem; color: #fff; font-weight: 400;
            }
            .control-btn {
                background: #fff; color: #000; border: none;
                padding: 12px 20px; border-radius: 8px; cursor: pointer;
                font-size: 0.9rem; font-weight: 600; letter-spacing: 0.5px;
                transition: all 0.3s ease;
            }
            .control-btn:hover { background: #ddd; transform: scale(1.05); }
            .control-btn.danger {
                background: #f00; color: #fff;
            }
            .control-btn.danger:hover { background: #c00; }
            
            @media (max-width: 1200px) {
                .main-grid { grid-template-columns: 1fr 1fr; }
                .logs-panel { display: none; }
            }
            @media (max-width: 768px) {
                .main-grid { grid-template-columns: 1fr; }
                .status-bar { flex-direction: column; gap: 15px; }
            }
        </style>
    </head>
    <body>
        <div class="group-badge">GROUP 07</div>
        
        <div class="header">
            <h1>UR5e SEQUENCE BOARD GAME</h1>
            <div class="subtitle">HTTP-Based Hand Detection System</div>
        </div>

        <div class="main-grid">
            <div class="panel">
                <div class="panel-title">GAME BOARD</div>
                <div class="gameboard-frame">
                    <div class="gameboard-container">
                        <img src="/gameboard-1.png" alt="Board Top" class="gameboard-half" 
                             onerror="this.style.display='none'; document.getElementById('err1').style.display='flex';">
                        <div id="err1" class="image-error" style="display:none;">gameboard-1.png missing</div>
                        
                        <div class="gameboard-spacer"></div>
                        
                        <img src="/gameboard-2.png" alt="Board Bottom" class="gameboard-half"
                             onerror="this.style.display='none'; document.getElementById('err2').style.display='flex';">
                        <div id="err2" class="image-error" style="display:none;">gameboard-2.png missing</div>
                    </div>
                </div>
            </div>

            <div class="panel">
                <div class="panel-title">LIVE HAND DETECTION</div>
                <div class="camera-container">
                    <img src="/video_feed" class="camera-feed" alt="Live Hand Detection" />
                    <div class="camera-status" id="cameraStatus">● DETECTION DISABLED</div>
                </div>
            </div>

            <div class="logs-panel">
                <div class="logs-header">SYSTEM LOGS</div>
                <div class="logs-container" id="logsContainer">
                    <!-- Logs will be populated by JavaScript -->
                </div>
            </div>
        </div>

        <div class="status-bar">
            <div class="status-left">
                <div class="status-indicator" id="safetyIndicator"></div>
                <div class="status-text" id="safetyStatus">Detection Disabled</div>
                <div class="status-text">FPS: <span id="cameraFps">0</span></div>
                <div class="status-text">Hazards: <span id="hazardsCount">0</span></div>
            </div>
            
            <div class="status-right">
                <button class="control-btn" id="startBtn" onclick="startDetection()">START DETECTION</button>
                <button class="control-btn" id="stopBtn" onclick="stopDetection()" style="display:none;">STOP DETECTION</button>
                <button class="control-btn danger" onclick="emergencyStop()">EMERGENCY STOP</button>
            </div>
        </div>

        <script>
            let logsContainer = document.getElementById('logsContainer');

            // Update system status every second
            function updateStatus() {
                fetch('/api/status')
                    .then(response => response.json())
                    .then(data => {
                        // Update FPS and hazards count
                        document.getElementById('cameraFps').textContent = data.camera_fps || 0;
                        document.getElementById('hazardsCount').textContent = data.hazards_detected || 0;
                        
                        // Update status indicator and text
                        const indicator = document.getElementById('safetyIndicator');
                        const statusText = document.getElementById('safetyStatus');
                        const cameraStatus = document.getElementById('cameraStatus');
                        
                        indicator.className = 'status-indicator';
                        cameraStatus.className = 'camera-status';
                        
                        if (!data.detection_enabled) {
                            statusText.textContent = 'Detection Disabled';
                            cameraStatus.textContent = '● DETECTION DISABLED';
                            document.getElementById('startBtn').style.display = 'inline-block';
                            document.getElementById('stopBtn').style.display = 'none';
                        } else if (data.hazard_detected) {
                            indicator.classList.add('hazard');
                            cameraStatus.classList.add('hazard');
                            statusText.textContent = 'HAND HAZARD DETECTED';
                            cameraStatus.textContent = '🚨 HAND HAZARD';
                        } else {
                            indicator.classList.add('monitoring');
                            cameraStatus.classList.add('monitoring');
                            statusText.textContent = 'Hand Detection Active';
                            cameraStatus.textContent = '● HAND DETECTION';
                            document.getElementById('startBtn').style.display = 'none';
                            document.getElementById('stopBtn').style.display = 'inline-block';
                        }
                        
                        // Update logs
                        updateLogs(data.logs || []);
                    })
                    .catch(error => {
                        console.error('Status update failed:', error);
                    });
            }

            function updateLogs(logs) {
                logsContainer.innerHTML = '';
                logs.forEach(log => {
                    const logEntry = document.createElement('div');
                    logEntry.className = `log-entry ${log.type}`;
                    logEntry.innerHTML = `
                        <div class="log-timestamp">${log.timestamp}</div>
                        <div class="log-title">${log.title}</div>
                        <div class="log-message">${log.message}</div>
                    `;
                    logsContainer.appendChild(logEntry);
                });
            }

            function startDetection() {
                console.log('🔄 Starting detection...');
                fetch('/api/start_detection', { method: 'POST' })
                    .then(response => response.json())
                    .then(data => {
                        console.log('✅ Detection started:', data);
                    })
                    .catch(error => {
                        console.error('Start detection failed:', error);
                    });
            }

            function stopDetection() {
                console.log('🔄 Stopping detection...');
                fetch('/api/stop_detection', { method: 'POST' })
                    .then(response => response.json())
                    .then(data => {
                        console.log('✅ Detection stopped:', data);
                    })
                    .catch(error => {
                        console.error('Stop detection failed:', error);
                    });
            }

            function emergencyStop() {
                console.log('🚨 Emergency stop...');
                fetch('/api/emergency_stop', { method: 'POST' })
                    .then(response => response.json())
                    .then(data => {
                        console.log('🚨 Emergency stop activated:', data);
                    })
                    .catch(error => {
                        console.error('Emergency stop failed:', error);
                    });
            }

            // Start status updates
            setInterval(updateStatus, 1000);
            updateStatus(); // Initial update

            console.log('🤖 UR5e Hand Detection System - Group 07');
            console.log('✅ HTTP-based system loaded successfully');
        </script>
    </body>
    </html>
    """)

def main():
    print("🚀 UR5e HTTP-Based Hand Detection - Group 07")
    print("🎮 Web Interface: http://localhost:5555")
    print("✋ Hand Detection: YOLOv8s (Small) Model")
    print("📹 Reliable communication without WebSocket issues")
    
    if os.path.exists('yolov8s.pt'):
        print("✅ Found local yolov8s.pt model")
    else:
        print("📥 Will download YOLOv8s model on first run")
    
    for i in [1, 2]:
        filename = f'gameboard-{i}.png'
        if os.path.exists(filename):
            print(f"✅ Found: {filename}")
        else:
            print(f"⚠️ Missing: {filename}")
    
    print("-" * 60)
    print("💡 USAGE:")
    print("1. Run: python ur5e_hand_detection_http.py")
    print("2. Open: http://localhost:5555")
    print("3. Click 'START DETECTION'")
    print("4. Wave hands to see RED bounding boxes")
    print("-" * 60)
    
    try:
        app.run(host='0.0.0.0', port=5555, debug=False)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
        detector.stop()
    except Exception as e:
        print(f"❌ Error: {e}")
        detector.stop()

if __name__ == '__main__':
    main()
