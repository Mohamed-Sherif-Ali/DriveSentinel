# 🛠️ DriveSentinel - Installation Guide

Complete setup instructions for Raspberry Pi, ESP32, and development environment.

---

## 📋 Table of Contents

1. [Hardware Setup](#hardware-setup)
2. [Raspberry Pi Configuration](#raspberry-pi-configuration)
3. [ESP32 Configuration](#esp32-configuration)
4. [Software Installation](#software-installation)
5. [Testing & Verification](#testing--verification)
6. [Troubleshooting](#troubleshooting)

---

## 🔌 Hardware Setup

### Required Components

**Computing:**
- Raspberry Pi 5 (or Pi 4 with 4GB+ RAM)
- ESP32 Development Board (38-pin version recommended)
- MicroSD Card (32GB minimum, Class 10)

**Sensors & Actuators:**
- USB/CSI Camera Module (720p minimum)
- 3× HC-SR04 Ultrasonic Sensors
- 4× DC Motors (12V with encoders recommended)
- 2× L298N Motor Driver Boards
- Piezo Buzzer (active, 5V)

**Power:**
- 12V LiPo Battery (3000mAh minimum)
- 5V Voltage Regulator (for Raspberry Pi)
- Jumper Wires (male-to-male, male-to-female)

### Wiring Diagram

```
┌────────────────────────────────────────────────────────────────┐
│                     RASPBERRY PI 5                             │
│                                                                │
│  GPIO14 (TX) ──────────────────────────────> GPIO16 (RX)       │
│  GPIO15 (RX) <────────────────────────────── GPIO17 (TX)       │
│  GND ─────────────────────────────────────────> GND            │
│                                                                │
│  USB ────> Camera Module                                       │
│                                                                │
└────────────────────────────────────────────────────────────────┘
                                    │
                                    │ Serial Communication
                                    │
                                    ▼
┌────────────────────────────────────────────────────────────────┐
│                         ESP32                                  │
│                                                                │
│  GPIO2 ──────> Ultrasonic Front (Trigger)                      │
│  GPIO17 <────── Ultrasonic Front (Echo)                        │
│  GPIO18 ──────> Ultrasonic Right (Trigger)                     │
│  GPIO34 <────── Ultrasonic Right (Echo)                        │
│  GPIO21 ──────> Ultrasonic Back (Trigger)                      │
│  GPIO5 <─────── Ultrasonic Back (Echo)                         │
│                                                                │
│  GPIO13 ──────> Buzzer (+)                                     │
│  GND ──────────> Buzzer (-)                                    │
│                                                                │
│  GPIO4,12,22,15,23,16 ──> Motor Driver 2 (Rear)                │
│  GPIO32,33,25,26,27,14 ──> Motor Driver 1 (Front)              │
│                                                                │
│  VIN ────> 5V from Regulator                                   │
│  GND ────> Common Ground                                       │
└────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌────────────────────────────────────────────────────────────────┐
│                    MOTOR DRIVERS                               │
│                                                                │
│  Motor Driver 1 (L298N) ──> Front Left Motor                   │
│                         ──> Front Right Motor                  │
│                                                                │
│  Motor Driver 2 (L298N) ──> Rear Left Motor                    │
│                         ──> Rear Right Motor                   │
│                                                                │
│  12V+ ────> From Battery                                       │
│  GND ─────> Common Ground                                      │
└────────────────────────────────────────────────────────────────┘
```

### Pin Connection Tables

**ESP32 to Ultrasonic Sensors:**

| ESP32 Pin | Sensor | Function |
|-----------|--------|----------|
| GPIO2 | Front | Trigger |
| GPIO17 | Front | Echo |
| GPIO18 | Right | Trigger |
| GPIO34 | Right | Echo |
| GPIO21 | Back | Trigger |
| GPIO5 | Back | Echo |

**ESP32 to Motor Drivers:**

| ESP32 Pin | Motor Driver | Function |
|-----------|--------------|----------|
| GPIO4 | Driver 2 | Rear Left Speed (EnA) |
| GPIO12 | Driver 2 | Rear Right Speed (EnB) |
| GPIO22 | Driver 2 | Rear Left In1 |
| GPIO15 | Driver 2 | Rear Left In2 |
| GPIO23 | Driver 2 | Rear Right In3 |
| GPIO16 | Driver 2 | Rear Right In4 |
| GPIO32 | Driver 1 | Front Left Speed (EnA) |
| GPIO33 | Driver 1 | Front Right Speed (EnB) |
| GPIO25 | Driver 1 | Front Left In1 |
| GPIO26 | Driver 1 | Front Left In2 |
| GPIO27 | Driver 1 | Front Right In3 |
| GPIO14 | Driver 1 | Front Right In4 |

---

## 🥧 Raspberry Pi Configuration

### 1. Install Raspberry Pi OS

**Download & Flash:**
```bash
# Download Raspberry Pi Imager
# https://www.raspberrypi.com/software/

# Recommended: Raspberry Pi OS (64-bit) Lite or Desktop
# Flash to microSD card using Imager
```

**Initial Setup:**
```bash
# Boot Raspberry Pi and complete initial setup
# Enable SSH: sudo raspi-config → Interface Options → SSH

# Update system
sudo apt update
sudo apt full-upgrade -y
sudo reboot
```

### 2. Enable Serial Communication

```bash
# Open configuration
sudo raspi-config

# Navigate to:
# 3 Interface Options
# → I6 Serial Port
# → "Would you like a login shell accessible over serial?" → NO
# → "Would you like the serial port hardware enabled?" → YES

# Reboot to apply changes
sudo reboot
```

**Verify Serial Port:**
```bash
# Check serial port exists
ls -l /dev/serial0
# Should show: /dev/serial0 -> ttyAMA0 (or similar)
```

### 3. Install System Dependencies

```bash
# Install build tools and libraries
sudo apt install -y \
    python3-pip \
    python3-opencv \
    python3-dev \
    libatlas-base-dev \
    libjasper-dev \
    libqt4-test \
    libhdf5-dev \
    libhdf5-serial-dev \
    libharfbuzz0b \
    libwebp6 \
    libtiff5 \
    libilmbase25 \
    libopenexr25 \
    libgstreamer1.0-0 \
    libavcodec58 \
    libavformat58 \
    libswscale5 \
    libqtgui4 \
    libqt4-test
```

### 4. Install Python Packages

```bash
# Navigate to project directory
cd ~/drivessentinel

# Install from requirements.txt
pip3 install -r requirements.txt

# Alternative: Install packages individually
pip3 install opencv-python==4.8.1.78
pip3 install mediapipe==0.10.8
pip3 install numpy==1.24.3
pip3 install pyserial==3.5
```

**Note:** On Raspberry Pi, you may get better performance using system OpenCV:
```bash
# Remove pip-installed OpenCV
pip3 uninstall opencv-python

# Use system package (already installed above)
# python3-opencv
```

### 5. Configure Camera

**For USB Camera:**
```bash
# No additional configuration needed
# Camera will be /dev/video0
```

**For CSI Camera Module:**
```bash
# Enable camera interface
sudo raspi-config
# → 3 Interface Options → I1 Legacy Camera → Enable

# Reboot
sudo reboot

# Verify camera
vcgencmd get_camera
# Should show: supported=1 detected=1
```

### 6. Set Permissions

```bash
# Add user to dialout group (for serial access)
sudo usermod -a -G dialout $USER

# Add user to video group (for camera access)
sudo usermod -a -G video $USER

# Log out and back in for changes to take effect
```

---

## 🔧 ESP32 Configuration

### 1. Install Arduino IDE

**Option A: Arduino IDE 2.x (Recommended)**
```bash
# Download from: https://www.arduino.cc/en/software
# Or use snap:
sudo snap install arduino
```

**Option B: PlatformIO (Advanced)**
```bash
# Install VSCode extension: PlatformIO IDE
# https://platformio.org/install/ide?install=vscode
```

### 2. Add ESP32 Board Support

**In Arduino IDE:**
```
1. Open: File → Preferences
2. Additional Board Manager URLs:
   https://dl.espressif.com/dl/package_esp32_index.json
3. Tools → Board → Boards Manager
4. Search: "esp32"
5. Install: "esp32 by Espressif Systems"
```

### 3. Configure Board Settings

```
Tools → Board → ESP32 Arduino → ESP32 Dev Module

Settings:
- Upload Speed: 921600
- CPU Frequency: 240MHz
- Flash Frequency: 80MHz
- Flash Mode: QIO
- Flash Size: 4MB
- Partition Scheme: Default
- Port: /dev/ttyUSB0 (or similar)
```

### 4. Upload Firmware

```bash
# Open PWM.ino in Arduino IDE

# Connect ESP32 via USB

# Click Upload button (or Ctrl+U)
# Wait for "Done uploading" message

# Open Serial Monitor (Tools → Serial Monitor)
# Set baud rate: 115200
# Should see initialization message
```

**Expected Output:**
```
═══════════════════════════════════════════
    DRIVESSENTINEL SYSTEM INITIALIZATION    
═══════════════════════════════════════════
✓ Ultrasonic sensors initialized
✓ Motor control pins initialized
✓ Buzzer initialized
✓ PWM channels configured (5 kHz, 8-bit)

═══════════════════════════════════════════
         INITIALIZATION COMPLETE           
═══════════════════════════════════════════
Status: READY
Autopilot: DISABLED
Initial lane: LEFT
Waiting for commands...
```

---

## 📦 Software Installation

### Clone Repository

```bash
# On Raspberry Pi
cd ~
git clone https://github.com/Mohamed-Sherif-Ali/drivessentinel.git
cd drivessentinel
```

### Directory Structure

```
drivessentinel/
├── PI_Code.py              # Main drowsiness detection
├── lane_detector.py        # Lane detection module
├── PWM.ino                 # ESP32 firmware
├── requirements.txt        # Python dependencies
├── README.md               # Project documentation
├── INSTALLATION.md         # This file
└── docs/
    └── TECHNICAL_DEEP_DIVE.md
```

### Install Dependencies

```bash
pip3 install -r requirements.txt
```

---

## ✅ Testing & Verification

### 1. Test Camera

```bash
# Simple camera test
python3 << EOF
import cv2
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
if ret:
    print("✓ Camera working! Resolution:", frame.shape)
else:
    print("✗ Camera failed!")
cap.release()
EOF
```

**Expected output:**
```
✓ Camera working! Resolution: (480, 640, 3)
```

### 2. Test Serial Connection

**On Raspberry Pi:**
```bash
# Install screen if needed
sudo apt install screen

# Open serial monitor
screen /dev/serial0 115200

# You should see ESP32 output
# Press Ctrl+A then K to exit
```

### 3. Test MediaPipe

```bash
python3 << EOF
import mediapipe as mp
face_mesh = mp.solutions.face_mesh.FaceMesh()
print("✓ MediaPipe initialized successfully!")
face_mesh.close()
EOF
```

### 4. Test Complete System

**Terminal 1 (Drowsiness Detection):**
```bash
cd ~/drivessentinel
python3 PI_Code.py
```

**Expected output:**
```
✅ Connected to ESP32
🚀 Driver monitoring started
[LOG] SYSTEM: Monitoring started
[LOG] EAR_UPDATE: 0.287
```

**Terminal 2 (Lane Detection - Optional):**
```bash
cd ~/drivessentinel
python3 lane_detector.py
```

**Expected output:**
```
🚀 Prototype-scale lane detection running...
Frame size: 320x240
Center: 160
Press Ctrl+C to stop

Frame 0002 | Detected: True | Position: Centered
```

### 5. Test Autopilot Engagement

**Trigger drowsiness:**
1. Close your eyes for 2+ seconds while facing camera
2. Watch terminal output:

```
[LOG] EAR_UPDATE: 0.189
[LOG] EAR_UPDATE: 0.165
[LOG] AUTOPILOT: Mode ON sent
[LOG] ESP_RECV: ACK:autopilot enabled
```

3. Open eyes for 3+ seconds:

```
[LOG] ALERT: Driver appears awake, monitoring...
[LOG] AUTOPILOT: Mode OFF sent
[LOG] ESP_RECV: ACK:autopilot disabled
```

---

## 🔍 Troubleshooting

### Camera Issues

**Problem:** "Could not open camera 0"
```bash
# List video devices
ls -l /dev/video*

# Try different camera index
# Edit PI_Code.py: WEBCAM_IDX = 1 (or 2)

# Check permissions
groups $USER
# Should include 'video' group
```

**Problem:** "Permission denied" on camera
```bash
sudo usermod -a -G video $USER
# Log out and back in
```

### Serial Communication Issues

**Problem:** "Could not connect to ESP32"
```bash
# Check serial port
ls -l /dev/serial0
ls -l /dev/ttyAMA0

# Check permissions
groups $USER
# Should include 'dialout' group

# If missing:
sudo usermod -a -G dialout $USER
# Log out and back in
```

**Problem:** Garbage data on serial
```bash
# Verify baud rate matches (115200)
# Check wiring: TX→RX, RX→TX (crossed!)
# Ensure common ground between Pi and ESP32
```

### MediaPipe Issues

**Problem:** "No module named 'mediapipe'"
```bash
# Check Python version (needs 3.7+)
python3 --version

# Reinstall
pip3 uninstall mediapipe
pip3 install mediapipe==0.10.8
```

**Problem:** MediaPipe crashes on Raspberry Pi
```bash
# Use 32-bit OS instead of 64-bit
# Or try older version:
pip3 install mediapipe==0.8.10
```

### ESP32 Issues

**Problem:** Upload failed
```bash
# Hold BOOT button while uploading
# Check USB cable (must support data, not just power)
# Try different USB port
# Lower upload speed: 460800
```

**Problem:** ESP32 constantly resets
```bash
# Power issue - use external 5V supply
# Don't power motors from USB!
# Check voltage regulator output
```

### Performance Issues

**Problem:** Low FPS, laggy detection
```bash
# Reduce camera resolution
# Edit PI_Code.py: 
#   cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
#   cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# Reduce processing frequency
# Edit PI_Code.py: time.sleep(0.1)  # 10 FPS instead of 20
```

**Problem:** High CPU usage
```bash
# Monitor CPU
htop

# If MediaPipe using too much:
# - Close other programs
# - Reduce camera FPS
# - Consider Raspberry Pi 5 (faster)
```

### Database Issues

**Problem:** Database locked
```bash
# Close all instances of PI_Code.py
pkill -f PI_Code.py

# Remove database and restart
rm security.db
python3 PI_Code.py
```

---

## 🚀 Running at Startup (Optional)

### Create Systemd Service

**Drowsiness Detection Service:**
```bash
sudo nano /etc/systemd/system/drivessentinel.service
```

```ini
[Unit]
Description=DriveSentinel Drowsiness Detection
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/drivessentinel
ExecStart=/usr/bin/python3 /home/pi/drivessentinel/PI_Code.py
Restart=on-failure
RestartSec=5s

[Install]
WantedBy=multi-user.target
```

**Enable and start:**
```bash
sudo systemctl daemon-reload
sudo systemctl enable drivessentinel.service
sudo systemctl start drivessentinel.service

# Check status
sudo systemctl status drivessentinel.service

# View logs
sudo journalctl -u drivessentinel.service -f
```

---

## 📝 Quick Reference

**Start System:**
```bash
cd ~/drivessentinel
python3 PI_Code.py
```

**Stop System:**
```bash
Ctrl+C
```

**View Logs:**
```bash
sqlite3 security.db "SELECT datetime(ts, 'unixepoch'), type, details FROM logs ORDER BY ts DESC LIMIT 20;"
```

**Update Code:**
```bash
cd ~/drivessentinel
git pull
pip3 install -r requirements.txt
```

---

## 🆘 Getting Help

If you encounter issues not covered here:

1. Check `security.db` logs for error messages
2. Review ESP32 serial output
3. Verify all connections match wiring diagram
4. Ensure software versions match requirements.txt
5. Search GitHub issues or create new one

---

**Installation complete! Your DriveSentinel system is ready.** 🎉
