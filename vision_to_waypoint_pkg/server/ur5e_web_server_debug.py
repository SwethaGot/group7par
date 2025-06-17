from flask import Flask, render_template_string, jsonify, request, Response, send_file
from flask_socketio import SocketIO, emit
import json
import time
import threading
import cv2
import mediapipe as mp
import numpy as np
from datetime import datetime
import base64
import os

app = Flask(__name__)
app.config['SECRET_KEY'] = 'ur5e_boardgame_secret'
socketio = SocketIO(app, cors_allowed_origins="*", logger=True, engineio_logger=True)

# Global state variables
robot_state = {
    'safety_status': 'safe',
    'hazard_detected': False,
    'robot_mode': 'idle',
    'game_state': 'waiting',
    'camera_fps': 0
}

# Hand detection setup
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=2,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.5
)

class HandSafetyMonitor:
    def __init__(self):
        self.hazard_active = False
        self.last_detection_time = 0
        self.camera_active = False
        self.cap = None
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.frame_count = 0
        
    def start_monitoring(self):
        """Start the hand detection in a separate thread"""
        if not self.camera_active:
            self.camera_active = True
            self.monitoring_thread = threading.Thread(target=self._monitor_loop, daemon=True)
            self.monitoring_thread.start()
            print("🛡️ Hand safety monitoring started")
            self._emit_log("System Started", "Monitoring activated", "success")
    
    def stop_monitoring(self):
        """Stop the hand detection"""
        self.camera_active = False
        if self.cap:
            self.cap.release()
        print("🛑 Hand safety monitoring stopped")
    
    def _emit_log(self, title, message, type="info"):
        """Emit log message to web interface"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        socketio.emit('log_message', {
            'timestamp': timestamp,
            'title': title,
            'message': message,
            'type': type  # success, warning, error, info
        })
    
    def _monitor_loop(self):
        """Main monitoring loop with better debugging"""
        try:
            self.cap = cv2.VideoCapture(0)
            
            # Optimize camera settings
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            fps_counter = 0
            fps_start = time.time()
            
            print("✅ Camera initialized - streaming via WebSocket")
            self._emit_log("Camera Ready", "Live feed streaming started", "success")
            
            while self.camera_active:
                ret, frame = self.cap.read()
                if not ret:
                    print("⚠️ Failed to read camera frame")
                    continue
                
                # Store latest frame
                with self.frame_lock:
                    self.latest_frame = frame.copy()
                    
                # Detect hands
                hands_detected = self._detect_hands(frame)
                
                # Update safety status
                if hands_detected and not self.hazard_active:
                    self._trigger_hazard()
                elif not hands_detected and self.hazard_active:
                    self._clear_hazard()
                
                # Emit frame EVERY frame for debugging (we'll reduce later if needed)
                self.frame_count += 1
                if self.frame_count % 2 == 0:  # Every 2nd frame
                    success = self._emit_frame(frame)
                    if not success and self.frame_count % 60 == 0:  # Log error every 60 frames
                        print(f"⚠️ Frame emission failed at frame {self.frame_count}")
                
                # Calculate FPS
                fps_counter += 1
                if time.time() - fps_start >= 1.0:
                    robot_state['camera_fps'] = fps_counter
                    fps_counter = 0
                    fps_start = time.time()
                    
                    # Emit status update
                    socketio.emit('status_update', robot_state)
                    
                    # Debug log every second
                    if self.frame_count % 30 == 0:
                        print(f"📹 Streaming: {robot_state['camera_fps']} FPS, Frame #{self.frame_count}")
                
                time.sleep(0.033)  # ~30 FPS
                
        except Exception as e:
            print(f"⚠️ Camera monitoring error: {e}")
            self._emit_log("Camera Error", str(e), "error")
        finally:
            if self.cap:
                self.cap.release()
    
    def _emit_frame(self, frame):
        """Emit camera frame to web interface via WebSocket"""
        try:
            # Add overlay if hazard
            display_frame = frame.copy()
            if self.hazard_active:
                overlay = display_frame.copy()
                cv2.rectangle(overlay, (0, 0), (640, 480), (0, 0, 255), 10)
                cv2.putText(overlay, "HAZARD DETECTED", (50, 250), 
                           cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 3)
                display_frame = cv2.addWeighted(overlay, 0.3, display_frame, 0.7, 0)
            
            # Encode to base64
            ret, buffer = cv2.imencode('.jpg', display_frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ret:
                frame_b64 = base64.b64encode(buffer).decode('utf-8')
                # Emit to all connected clients
                socketio.emit('camera_frame', {'frame': frame_b64})
                return True
            else:
                print("❌ Failed to encode frame")
                return False
            
        except Exception as e:
            print(f"Frame emission error: {e}")
            return False
    
    def _detect_hands(self, frame):
        """Detect hands in frame"""
        try:
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = hands.process(rgb_frame)
            return results.multi_hand_landmarks is not None
        except:
            return False
    
    def _trigger_hazard(self):
        """Trigger hazard alert"""
        current_time = time.time()
        if current_time - self.last_detection_time < 1.0:
            return
            
        self.hazard_active = True
        self.last_detection_time = current_time
        
        # Update global state
        robot_state['hazard_detected'] = True
        robot_state['safety_status'] = 'hazard'
        robot_state['robot_mode'] = 'stopped'
        robot_state['game_state'] = 'hazard_stop'
        
        # Log hazard
        timestamp = datetime.now().strftime("%H:%M:%S")
        print(f"\n🚨 HAZARD DETECTED at {timestamp}")
        print("🛑 HAZARD.. STOPPING")
        print("✋ Human hand detected in work area!")
        
        # Emit to web interface
        socketio.emit('hazard_alert', {
            'type': 'hazard_detected',
            'message': 'Hand detected - Robot stopped',
            'timestamp': timestamp
        })
        
        # Emit beautiful log
        self._emit_log("⚠️ HAZARD DETECTED", "Human hand in work area - Robot stopped", "error")
    
    def _clear_hazard(self):
        """Clear hazard when area is safe"""
        if not self.hazard_active:
            return
            
        self.hazard_active = False
        
        # Update global state
        robot_state['hazard_detected'] = False
        robot_state['safety_status'] = 'safe'
        robot_state['robot_mode'] = 'idle'
        robot_state['game_state'] = 'waiting'
        
        # Log clearance
        timestamp = datetime.now().strftime("%H:%M:%S")
        print(f"✅ [{timestamp}] SAFETY ALL CLEAR")
        print("🟢 Work area secure")
        
        # Emit to web interface
        socketio.emit('hazard_clear', {
            'type': 'hazard_cleared',
            'message': 'Area secure',
            'timestamp': timestamp
        })
        
        # Emit beautiful log
        self._emit_log("✅ ALL CLEAR", "Work area secure - Operations resumed", "success")

# Initialize hand safety monitor
safety_monitor = HandSafetyMonitor()

# Route to serve gameboard images
@app.route('/gameboard-<int:num>.png')
def serve_gameboard(num):
    """Serve gameboard images from current directory"""
    try:
        filename = f'gameboard-{num}.png'
        if os.path.exists(filename):
            return send_file(filename, mimetype='image/png')
        else:
            print(f"⚠️ Image not found: {filename}")
            return Response('', status=404)
    except Exception as e:
        print(f"❌ Error serving gameboard image: {e}")
        return Response('', status=404)

# HTML template with LIVE LOGS and fixed camera
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>UR5e Board Game Safety System - Group 07</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        
        body { 
            font-family: 'SF Pro Display', -apple-system, BlinkMacSystemFont, sans-serif;
            background: #000000; color: #ffffff; min-height: 100vh; overflow-x: hidden;
        }
        
        .header { 
            text-align: center; padding: 20px; background: #111111;
            border-bottom: 1px solid #333333; position: relative;
        }
        
        .header h1 { 
            font-size: 2rem; font-weight: 300; margin-bottom: 5px;
            color: #ffffff; letter-spacing: 2px;
        }
        
        .header .subtitle { 
            font-size: 0.9rem; color: #888888; font-weight: 300;
        }
        
        .group-badge {
            position: absolute; top: 20px; right: 20px;
            background: #ffffff; color: #000000; padding: 8px 16px;
            border-radius: 20px; font-weight: 600; font-size: 0.8rem;
            letter-spacing: 1px;
        }
        
        .main-container {
            display: grid; grid-template-columns: 1fr 1fr 300px; gap: 1px;
            min-height: calc(100vh - 150px); background: #333333;
        }
        
        .section {
            background: #000000; padding: 30px; display: flex;
            flex-direction: column; align-items: center; justify-content: center;
        }
        
        .section-title {
            font-size: 1.2rem; font-weight: 300; margin-bottom: 20px;
            color: #ffffff; letter-spacing: 1px; text-align: center;
        }
        
        /* Game Board Section */
        .gameboard-frame {
            width: 100%; max-width: 400px; 
            background: #ffffff;
            border-radius: 15px; 
            padding: 20px;
            box-shadow: 0 4px 20px rgba(255, 255, 255, 0.1);
        }
        
        .gameboard-container {
            width: 100%; background: #000000;
            border-radius: 10px; overflow: hidden;
            display: flex; flex-direction: column;
        }
        
        .gameboard-half {
            width: 100%; height: auto; display: block;
        }
        
        .gameboard-spacer {
            height: 8px;
            background: #ffffff;
        }
        
        .image-error {
            width: 100%; height: 150px; background: #222222;
            border: 1px solid #444444; border-radius: 5px;
            display: flex; align-items: center; justify-content: center;
            color: #666666; font-size: 0.8rem; margin: 2px 0;
        }
        
        /* Camera Feed Section */
        .camera-container {
            width: 100%; max-width: 400px; background: #111111;
            border-radius: 10px; overflow: hidden; border: 1px solid #333333;
            position: relative;
        }
        
        .camera-feed {
            width: 100%; height: auto; display: block;
        }
        
        .camera-status {
            position: absolute; top: 10px; left: 10px;
            background: rgba(0, 0, 0, 0.7); padding: 5px 10px;
            border-radius: 5px; font-size: 0.8rem; color: #ffffff;
        }
        
        .camera-status.live {
            background: rgba(0, 255, 0, 0.7);
        }
        
        .camera-status.hazard {
            background: rgba(255, 0, 0, 0.7);
            animation: pulse 1s infinite;
        }
        
        .camera-placeholder {
            width: 100%; height: 300px; background: #111111;
            border: 1px solid #333333; border-radius: 10px;
            display: flex; align-items: center; justify-content: center;
            color: #666666; font-size: 0.9rem; flex-direction: column; gap: 10px;
        }
        
        /* LIVE LOGS SECTION */
        .logs-section {
            background: #000000; padding: 20px; display: flex;
            flex-direction: column; max-height: calc(100vh - 150px); overflow: hidden;
        }
        
        .logs-header {
            font-size: 1rem; font-weight: 300; margin-bottom: 15px;
            color: #ffffff; letter-spacing: 1px; text-align: center;
            border-bottom: 1px solid #333333; padding-bottom: 10px;
        }
        
        .logs-container {
            flex: 1; overflow-y: auto; display: flex; flex-direction: column-reverse;
            scrollbar-width: thin; scrollbar-color: #333333 #000000;
        }
        
        .logs-container::-webkit-scrollbar {
            width: 6px;
        }
        
        .logs-container::-webkit-scrollbar-track {
            background: #000000;
        }
        
        .logs-container::-webkit-scrollbar-thumb {
            background: #333333;
            border-radius: 3px;
        }
        
        .log-entry {
            margin-bottom: 10px; padding: 12px; border-radius: 8px;
            border-left: 3px solid; font-size: 0.85rem; line-height: 1.4;
            animation: slideIn 0.3s ease-out;
        }
        
        .log-entry.success {
            background: rgba(0, 255, 0, 0.1); border-color: #00ff00; color: #ccffcc;
        }
        
        .log-entry.error {
            background: rgba(255, 0, 0, 0.1); border-color: #ff0000; color: #ffcccc;
        }
        
        .log-entry.warning {
            background: rgba(255, 255, 0, 0.1); border-color: #ffff00; color: #ffffcc;
        }
        
        .log-entry.info {
            background: rgba(0, 150, 255, 0.1); border-color: #0096ff; color: #ccddff;
        }
        
        .log-timestamp {
            font-size: 0.75rem; opacity: 0.7; float: right;
        }
        
        .log-title {
            font-weight: 600; margin-bottom: 2px;
        }
        
        .log-message {
            opacity: 0.9;
        }
        
        @keyframes slideIn {
            from { opacity: 0; transform: translateX(20px); }
            to { opacity: 1; transform: translateX(0); }
        }
        
        @keyframes pulse { 0%, 100% { opacity: 0.7; } 50% { opacity: 1; } }
        
        /* Status Panel */
        .status-panel {
            position: fixed; bottom: 0; left: 0; right: 0;
            background: #111111; border-top: 1px solid #333333;
            padding: 15px; display: flex; justify-content: space-between;
            align-items: center; z-index: 1000;
        }
        
        .status-left, .status-right {
            display: flex; align-items: center; gap: 20px;
        }
        
        .status-indicator {
            width: 10px; height: 10px; border-radius: 50%;
            background: #00ff00; animation: blink 2s infinite;
        }
        
        .status-indicator.hazard {
            background: #ff0000; animation: fastBlink 0.5s infinite;
        }
        
        @keyframes blink { 0%, 100% { opacity: 1; } 50% { opacity: 0.3; } }
        @keyframes fastBlink { 0%, 100% { opacity: 1; } 50% { opacity: 0.1; } }
        
        .status-text {
            font-size: 0.9rem; color: #ffffff; font-weight: 300;
        }
        
        .control-button {
            background: #ffffff; color: #000000; border: none;
            padding: 8px 16px; border-radius: 5px; cursor: pointer;
            font-size: 0.8rem; font-weight: 600; letter-spacing: 0.5px;
            transition: all 0.2s ease;
        }
        
        .control-button:hover { background: #cccccc; }
        
        .control-button.danger {
            background: #ff0000; color: #ffffff;
        }
        
        .control-button.danger:hover { background: #cc0000; }
        
        /* Responsive */
        @media (max-width: 1200px) {
            .main-container { grid-template-columns: 1fr 1fr; }
            .logs-section { display: none; }
        }
        
        @media (max-width: 768px) {
            .main-container { grid-template-columns: 1fr; }
            .group-badge { position: relative; top: 0; right: 0; margin: 10px auto; }
            .status-panel { flex-direction: column; gap: 10px; }
        }
        
        .loading-spinner {
            border: 2px solid #333333;
            border-top: 2px solid #ffffff;
            border-radius: 50%;
            width: 20px; height: 20px;
            animation: spin 1s linear infinite;
        }
        
        @keyframes spin {
            0% { transform: rotate(0deg); }
            100% { transform: rotate(360deg); }
        }
    </style>
</head>
<body>
    <div class="group-badge">GROUP 07</div>
    
    <div class="header">
        <h1>UR5e SEQUENCE BOARD GAME</h1>
        <div class="subtitle">Hand Safety System | Programming Autonomous Robots</div>
    </div>

    <div class="main-container">
        <!-- Game Board Section -->
        <div class="section">
            <div class="section-title">GAME BOARD</div>
            <div class="gameboard-frame">
                <div class="gameboard-container">
                    <img src="/gameboard-1.png" alt="Game Board Top Half" class="gameboard-half" 
                         onerror="this.style.display='none'; document.getElementById('board-error-1').style.display='flex';">
                    <div id="board-error-1" class="image-error" style="display: none;">
                        gameboard-1.png not found
                    </div>
                    
                    <div class="gameboard-spacer"></div>
                    
                    <img src="/gameboard-2.png" alt="Game Board Bottom Half" class="gameboard-half"
                         onerror="this.style.display='none'; document.getElementById('board-error-2').style.display='flex';">
                    <div id="board-error-2" class="image-error" style="display: none;">
                        gameboard-2.png not found
                    </div>
                </div>
            </div>
        </div>

        <!-- Camera Feed Section -->
        <div class="section">
            <div class="section-title">LIVE CAMERA FEED</div>
            <div class="camera-container">
                <img id="cameraFeed" class="camera-feed" style="display: none;" alt="Live Camera Feed">
                <div id="cameraPlaceholder" class="camera-placeholder">
                    <div class="loading-spinner"></div>
                    <div>Waiting for camera frames...</div>
                    <div style="font-size: 0.8rem; color: #555;">Check browser console for errors</div>
                </div>
                <div id="cameraStatus" class="camera-status live">● LIVE</div>
            </div>
        </div>

        <!-- Live Logs Section -->
        <div class="logs-section">
            <div class="logs-header">LIVE SYSTEM LOGS</div>
            <div class="logs-container" id="logsContainer">
                <div class="log-entry info">
                    <div class="log-timestamp">--:--:--</div>
                    <div class="log-title">System Ready</div>
                    <div class="log-message">Waiting for initialization...</div>
                </div>
            </div>
        </div>
    </div>

    <!-- Status Panel -->
    <div class="status-panel">
        <div class="status-left">
            <div class="status-indicator" id="safetyIndicator"></div>
            <div class="status-text" id="safetyStatus">System Ready</div>
            <div class="status-text">FPS: <span id="cameraFps">0</span></div>
            <div class="status-text">Mode: <span id="robotMode">Idle</span></div>
        </div>
        
        <div class="status-right">
            <button class="control-button" onclick="startMonitoring()">START</button>
            <button class="control-button danger" onclick="emergencyStop()">STOP</button>
        </div>
    </div>

    <script>
        // WebSocket connection
        const socket = io();
        let cameraFeed = document.getElementById('cameraFeed');
        let cameraPlaceholder = document.getElementById('cameraPlaceholder');
        let logsContainer = document.getElementById('logsContainer');
        let frameReceived = false;

        // Debug logging
        console.log('🤖 UR5e Board Game System - Group 07');
        console.log('🔗 Connecting to WebSocket...');

        // Socket event handlers
        socket.on('connect', function() {
            console.log('✅ Connected to UR5e system');
            updateStatus('Connected');
            addLog('🔗 Connected', 'WebSocket connection established', 'success');
        });

        socket.on('disconnect', function() {
            console.log('❌ Disconnected from system');
            addLog('❌ Disconnected', 'Connection lost', 'error');
        });

        socket.on('camera_frame', function(data) {
            console.log('📹 Received camera frame');
            
            // Show camera feed and hide placeholder
            if (!frameReceived) {
                cameraPlaceholder.style.display = 'none';
                cameraFeed.style.display = 'block';
                frameReceived = true;
                console.log('✅ Camera feed activated');
                addLog('📹 Camera Active', 'Live video feed streaming', 'success');
            }
            
            // Update camera feed
            cameraFeed.src = 'data:image/jpeg;base64,' + data.frame;
        });

        socket.on('status_update', function(data) {
            document.getElementById('cameraFps').textContent = data.camera_fps || 0;
            document.getElementById('robotMode').textContent = data.robot_mode || 'Idle';
        });

        socket.on('hazard_alert', function(data) {
            // Show hazard status
            document.getElementById('safetyIndicator').classList.add('hazard');
            document.getElementById('safetyStatus').textContent = 'HAZARD DETECTED';
            document.getElementById('cameraStatus').textContent = '⚠️ HAZARD';
            document.getElementById('cameraStatus').className = 'camera-status hazard';
            console.log('🚨 HAZARD:', data.message);
        });

        socket.on('hazard_clear', function(data) {
            // Clear hazard status
            document.getElementById('safetyIndicator').classList.remove('hazard');
            document.getElementById('safetyStatus').textContent = 'Area Secure';
            document.getElementById('cameraStatus').textContent = '● LIVE';
            document.getElementById('cameraStatus').className = 'camera-status live';
            console.log('✅ CLEAR:', data.message);
        });

        socket.on('log_message', function(data) {
            addLog(data.title, data.message, data.type, data.timestamp);
        });

        // Add log entry to the live logs
        function addLog(title, message, type = 'info', timestamp = null) {
            if (!timestamp) {
                timestamp = new Date().toLocaleTimeString();
            }
            
            const logEntry = document.createElement('div');
            logEntry.className = `log-entry ${type}`;
            logEntry.innerHTML = `
                <div class="log-timestamp">${timestamp}</div>
                <div class="log-title">${title}</div>
                <div class="log-message">${message}</div>
            `;
            
            // Add to top of logs (newest first)
            logsContainer.insertBefore(logEntry, logsContainer.firstChild);
            
            // Keep only last 50 logs
            while (logsContainer.children.length > 50) {
                logsContainer.removeChild(logsContainer.lastChild);
            }
        }

        // Control functions
        function startMonitoring() {
            socket.emit('start_monitoring');
            updateStatus('Monitoring Active');
            console.log('🛡️ Monitoring started');
            addLog('🛡️ Monitoring Started', 'Hand detection activated', 'success');
        }

        function emergencyStop() {
            socket.emit('emergency_stop');
            console.log('🛑 EMERGENCY STOP');
            addLog('🛑 EMERGENCY STOP', 'Manual emergency stop activated', 'error');
        }

        function updateStatus(status) {
            document.getElementById('safetyStatus').textContent = status;
        }

        // Debug camera issues
        setTimeout(() => {
            if (!frameReceived) {
                console.log('⚠️ No camera frames received after 5 seconds');
                console.log('🔍 Check if camera is being used by another app');
                addLog('⚠️ Camera Issue', 'No frames received - check camera permissions', 'warning');
            }
        }, 5000);

        // Auto-start monitoring
        setTimeout(() => {
            console.log('🚀 Auto-starting monitoring...');
            socket.emit('start_monitoring');
        }, 1000);
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    """Serve the main interface"""
    return render_template_string(HTML_TEMPLATE)

@app.route('/api/status')
def get_status():
    """Get current robot status"""
    return jsonify(robot_state)

# WebSocket event handlers
@socketio.on('connect')
def handle_connect():
    print('🔗 Web client connected')
    emit('status_update', robot_state)

@socketio.on('start_monitoring')
def handle_start_monitoring():
    print('🛡️ Starting hand safety monitoring...')
    safety_monitor.start_monitoring()
    robot_state['game_state'] = 'monitoring_active'
    robot_state['robot_mode'] = 'monitoring'
    emit('status_update', robot_state)

@socketio.on('emergency_stop')
def handle_emergency_stop():
    print('🚨 EMERGENCY STOP ACTIVATED')
    robot_state['safety_status'] = 'emergency_stop'
    robot_state['robot_mode'] = 'emergency_stopped'
    
    socketio.emit('hazard_alert', {
        'type': 'emergency_stop',
        'message': 'Emergency stop activated',
        'timestamp': datetime.now().strftime("%H:%M:%S")
    })

def start_server():
    """Start the web server on port 5555"""
    print("🚀 UR5e Board Game System - Group 07")
    print("🎮 Interface: http://localhost:5555")
    print("🛡️ Hand Safety: Debug mode with live logs")
    print("📷 Camera: Enhanced frame emission debugging")
    print("🎯 Gameboards: Styled with white frame and spacing")
    
    # Check if gameboard images exist
    for i in [1, 2]:
        filename = f'gameboard-{i}.png'
        if os.path.exists(filename):
            print(f"✅ Found: {filename}")
        else:
            print(f"⚠️  Missing: {filename} (place in same directory)")
    
    print("-" * 60)
    print("🔍 DEBUG: Watch browser console for camera frame logs")
    print("📊 LOGS: Live system logs now visible in web interface")
    
    # Start the server on port 5555
    socketio.run(app, host='0.0.0.0', port=5555, debug=False, allow_unsafe_werkzeug=True)

if __name__ == '__main__':
    start_server()