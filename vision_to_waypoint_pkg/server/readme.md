# Install dependencies
pip install -r requirements.txt

# Terminal 1: Start web server
python ur5e_web_server_simple.py

# Terminal 2: Start hand detection  
python ur5e_hand_safety_only.py

# Browser: Open interface
http://localhost:5000