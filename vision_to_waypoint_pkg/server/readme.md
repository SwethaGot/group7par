# Install dependencies
pip install -r requirements.txt

# Terminal 1: Start web server
python3 ur5e_web_server_debug.py

# Terminal 2: Start hand detection  
python3 ur5e_hand_detection_clean.py

# Browser: Open interface
http://localhost:5000
