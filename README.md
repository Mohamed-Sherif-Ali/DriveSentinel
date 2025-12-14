# 🚗 DriveSentinel

**Autonomous Driver Monitoring & Vehicle Control System**

An intelligent safety system that monitors driver alertness using computer vision and controls vehicle autopilot based on drowsiness detection. Combines edge AI, embedded systems, and real-time decision-making for autonomous vehicle safety.

---

## 📋 Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Stack](#software-stack)
- [Installation](#installation)
- [Usage](#usage)
- [How It Works](#how-it-works)
- [Project Structure](#project-structure)
- [Technical Details](#technical-details)
- [Future Enhancements](#future-enhancements)
- [License](#license)

---

## 🎯 Overview

DriveSentinel is a prototype autonomous vehicle safety system designed to prevent accidents caused by driver drowsiness. The system continuously monitors the driver's face using computer vision, detects when they become drowsy, and automatically engages vehicle autopilot until the driver is alert again.

**Key Capabilities:**
- Real-time drowsiness detection using facial landmark analysis
- Automatic autopilot engagement/disengagement
- Lane detection and position tracking
- Obstacle avoidance with ultrasonic sensors
- Comprehensive event logging and diagnostics

---

## 🏗️ System Architecture

```
┌───────────────────────────────────────────────────────────────┐
│                        DRIVESSENTINEL                         │
│                                                               │
│  ┌──────────────────┐         ┌──────────────────┐            │
│  │  Raspberry Pi 5  │ Serial  │     ESP32        │            │
│  │  ─────────────   │ ◄─────► │  ─────────────   │            │
│  │  • Face Detect   │         │  • Motor Control │            │
│  │  • Drowsiness    │         │  • Sensors       │            │
│  │  • Lane Detect   │         │  • Safety Logic  │            │
│  └────────┬─────────┘         └────────┬─────────┘            │
│           │                            │                      │
│      ┌────▼────┐                  ┌────▼──────┐               │
│      │ Camera  │                  │  Sensors  │               │
│      │ Module  │                  │  + Motors │               │
│      └─────────┘                  └───────────┘               │
└───────────────────────────────────────────────────────────────┘
```

### Component Roles

| Component | Responsibility | Technology |
|-----------|---------------|------------|
| **Raspberry Pi** | AI processing, computer vision, decision-making | Python, OpenCV, MediaPipe |
| **ESP32** | Motor control, sensor reading, real-time safety logic | C++ (Arduino framework) |
| **Camera** | Video capture for face and lane detection | USB/CSI Camera Module |
| **Ultrasonic Sensors** | Obstacle detection (front, right, back) | HC-SR04 or similar |
| **Motors** | Vehicle movement (4-wheel drive) | DC motors with PWM control |

---

## ✨ Features

### 🧠 Driver Monitoring
- **Facial Landmark Detection**: Tracks 468 facial points using MediaPipe
- **Eye Aspect Ratio (EAR)**: Calculates eye openness in real-time
- **Drowsiness Algorithm**: Triggers after 1 second of closed eyes
- **False Positive Prevention**: 1.5-second grace period before triggering

### 🚘 Vehicle Control
- **Automatic Autopilot**: Engages when driver is drowsy
- **Lane Detection**: Computer vision-based lane tracking
- **Obstacle Avoidance**: 3-sensor array (front, side, rear)
- **State Machine**: 5-state control logic for safe operation

### 📊 Safety Features
- **Emergency Stop**: Immediate halt on obstacle detection
- **Gradual Recovery**: 2-second awake period before disengaging autopilot
- **Event Logging**: SQLite database tracks all system events
- **Thread-Safe Communication**: Prevents data corruption

---

## 🔧 Hardware Requirements

### Core Components
- **Raspberry Pi 5** (or Pi 4 4GB+)
- **ESP32 Development Board**
- **USB/CSI Camera Module** (minimum 720p)
- **3× HC-SR04 Ultrasonic Sensors**
- **4× DC Motors** (12V with encoders recommended)
- **2× L298N Motor Drivers** (or equivalent)
- **12V Battery Pack**
- **Chassis** (4WD platform)

### Wiring Overview
```
Raspberry Pi ─UART─> ESP32
                      │
                      ├─PWM─> Motor Driver 1 ─> Front Motors
                      ├─PWM─> Motor Driver 2 ─> Rear Motors
                      ├─GPIO─> Ultrasonic Front
                      ├─GPIO─> Ultrasonic Right
                      └─GPIO─> Ultrasonic Back
```

---

## 💻 Software Stack

### Raspberry Pi
```
Python 3.8+
├── OpenCV (cv2)           # Computer vision
├── MediaPipe              # Face mesh detection
├── NumPy                  # Numerical operations
├── pySerial               # Serial communication
└── SQLite3                # Event logging
```

### ESP32
```
Arduino Framework
├── ESP32 Core Libraries
├── Serial Communication
└── PWM Motor Control
```

---

## 📥 Installation

### Raspberry Pi Setup

1. **Install Dependencies**
```bash
sudo apt update
sudo apt install python3-opencv python3-pip

pip3 install mediapipe numpy pyserial
```

2. **Enable Serial Port**
```bash
sudo raspi-config
# Navigate to: Interface Options > Serial Port
# Disable serial console, enable serial hardware
```

3. **Clone Repository**
```bash
git clone https://github.com/Mohamed-Sherif-Ali/drivessentinel.git
cd drivessentinel
```

### ESP32 Setup

1. **Install Arduino IDE** or PlatformIO

2. **Install ESP32 Board Support**
   - Arduino IDE: File → Preferences → Additional Board Manager URLs
   - Add: `https://dl.espressif.com/dl/package_esp32_index.json`

3. **Upload Firmware**
   - Open `PWM.ino`
   - Select Board: ESP32 Dev Module
   - Upload to ESP32

---

## 🚀 Usage

### Running the System

1. **Start Driver Monitoring** (Raspberry Pi)
```bash
python3 PI_Code.py
```

2. **Start Lane Detection** (Raspberry Pi - separate terminal)
```bash
python3 lane_detector.py
```

3. **Monitor Serial Output** (Optional)
```bash
screen /dev/serial0 115200
```

### Expected Output

**Normal Operation:**
```
🚀 Driver monitoring started
[LOG] EAR_UPDATE: 0.287
[LOG] EAR_UPDATE: 0.291
Frame 0042 | Detected: True | Position: Centered
```

**Drowsiness Detected:**
```
[LOG] EAR_UPDATE: 0.189
[LOG] AUTOPILOT: Mode ON sent
[LOG] ESP_RECV: ACK:autopilot enabled
```

**Driver Awake:**
```
[LOG] ALERT: Driver appears awake, monitoring...
[LOG] AUTOPILOT: Mode OFF sent
[LOG] ESP_RECV: ACK:autopilot disabled
```

---

## 🔬 How It Works

### Drowsiness Detection Pipeline

```
1. Camera Capture (20 FPS)
   ↓
2. MediaPipe Face Mesh (468 landmarks)
   ↓
3. Extract Eye Landmarks (6 per eye)
   ↓
4. Calculate Eye Aspect Ratio (EAR)
   ↓
5. Compare to Threshold (EAR < 0.25?)
   ↓
6. Count Consecutive Frames (20 frames = 1 second)
   ↓
7. Send Autopilot Command to ESP32
```

### Eye Aspect Ratio Formula

```
EAR = (||p2 - p6|| + ||p3 - p5||) / (2 × ||p1 - p4||)

Where p1-p6 are the 6 eye landmark points:
- p1, p4: Horizontal eye corners
- p2, p3, p5, p6: Vertical eyelid points

Typical values:
- Eyes open: 0.25 - 0.40
- Eyes closed: < 0.20
```

### Lane Detection Pipeline

```
1. Grayscale Conversion
   ↓
2. Gaussian Blur (reduce noise)
   ↓
3. Canny Edge Detection
   ↓
4. Region of Interest Mask (bottom half)
   ↓
5. Hough Line Transform
   ↓
6. Separate Left/Right Lanes (by slope)
   ↓
7. Calculate Lane Midpoint
   ↓
8. Determine Position (Left/Center/Right)
```

### ESP32 State Machine

```
STATE_DRIVING
    ↓ (Right clearance > 20cm)
STATE_CHECKING_LANE (1 second)
    ↓ (Still clear)
STATE_SWITCHING_LANE (1.5 seconds - gentle turn)
    ↓ (Complete)
STATE_SAFE_STOP
    ↓ (Rear clear)
STATE_DRIVING

Emergency Override: Any state → STATE_EMERGENCY_STOP
```

---

## 📁 Project Structure

```
drivessentinel/
│
├── PI_Code.py              # Main drowsiness detection system
├── lane_detector.py        # Lane detection module
├── PWM.ino                 # ESP32 firmware
│
├── security.db             # SQLite event logs (generated at runtime)
├── prototype_lane_*.jpg    # Debug images (generated at runtime)
│
├── README.md               # This file
```

---

## 🔍 Technical Details

### Communication Protocol

**Raspberry Pi → ESP32:**
```
autopilot mode:on\n
autopilot mode:off\n
```

**ESP32 → Raspberry Pi:**
```
ACK:autopilot enabled
ACK:autopilot disabled
Sensors (cm): F=45, R=30, B=50
```

### Database Schema

```sql
CREATE TABLE logs (
    ts      REAL,      -- Unix timestamp
    type    TEXT,      -- Event type (AUTOPILOT, EAR_UPDATE, etc.)
    details TEXT       -- Event details
);
```

### Safety Thresholds

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| EAR Threshold | 0.25 | Below this = eyes closed |
| Consecutive Frames | 20 | 1 second at 20 FPS |
| Alert Release | 2.0s | Ensure driver is fully awake |
| Emergency Front | 25cm | Safe stopping distance |
| Emergency Side | 15cm | Lane boundary clearance |
| Emergency Back | 10cm | Rear collision prevention |

---

## 🎓 Learning Outcomes

This project demonstrates:

- **Computer Vision**: Face detection, landmark tracking, edge detection
- **Embedded Systems**: Real-time motor control, sensor fusion
- **Concurrent Programming**: Multi-threading, thread synchronization
- **Serial Communication**: UART protocol, data integrity
- **State Machines**: Finite state automata for control logic
- **Safety-Critical Design**: Fail-safes, timeout mechanisms
- **System Integration**: Multiple platforms working together

---

## 🚀 Future Enhancements

### Planned Features
- [ ] Machine learning drowsiness classifier (scikit-learn)
- [ ] Deep learning lane detection (YOLO/LaneNet)
- [ ] Wireless communication (WiFi/Bluetooth)
- [ ] Mobile app for monitoring
- [ ] GPS integration for route tracking
- [ ] Voice alerts for driver warnings

### Performance Improvements
- [ ] Multi-processing for parallel CV tasks
- [ ] Hardware acceleration (Neural Compute Stick)
- [ ] Optimized lane detection (reduce CPU usage)
- [ ] Adaptive thresholds based on lighting

---

## 📊 Performance Metrics

**Current Specifications:**
- **Detection Latency**: ~50ms per frame
- **Autopilot Engagement**: 1 second after drowsiness
- **Lane Detection**: 10 FPS (every 2nd frame)
- **Database Growth**: ~2 MB/hour
- **False Positive Rate**: <5% (with grace period)

---

## 🤝 Contributing

This is a graduation project, but suggestions are welcome:

1. Fork the repository
2. Create a feature branch
3. Submit pull request with detailed description

---

## 📄 License

This project is licensed under the MIT License - see LICENSE file for details.

---

## 👤 Author

**Mohamed Sherif** - AI & Computer Vision Engineer 

**Project Context:**  
Graduation project demonstrating integration of computer vision, embedded systems, and autonomous vehicle control. Developed as a prototype autonomous safety system.

---

## 🙏 Acknowledgments

- **MediaPipe** by Google for facial landmark detection
- **OpenCV** community for computer vision tools
- **ESP32** community for embedded development resources

---

## 📞 Contact

For questions or collaboration:
- GitHub: [@Mohamed-Sherif-Ali](https://github.com/Mohamed-Sherif-Ali)
- Email: mohamedshrif456@gmail.com.com

---

**⚠️ Safety Disclaimer**: This is a prototype system for educational purposes. Not intended for use on public roads or in real vehicles without extensive testing and certification.
