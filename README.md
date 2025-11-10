
# 🚗 DriveSentinel  
**AI-Powered Driver Safety & Autopilot Platform**

> “Your car’s second pair of eyes — always alert, always safe.”

---

## 🧠 Overview
**DriveSentinel** is a full-stack **AI + IoT automotive safety system** that unites **Raspberry Pi 5** (vision & decision brain) and **ESP32** (real-time motor & sensor controller).  
It detects driver drowsiness, tracks lane position, avoids obstacles, and secures vehicle ignition via fingerprint authentication.

The project merges **embedded control**, **computer vision**, and **human-aware automation** into one cohesive research-grade prototype.

---

## ⚙️ Hardware & Software Stack

| Layer | Components |
|-------|-------------|
| **AI Brain** | Raspberry Pi 5 + Python 3 + MediaPipe + OpenCV |
| **Actuation** | ESP32 (WROOM-32) + L298N motor drivers |
| **Sensing** | R307 fingerprint sensor • Ultrasonic (HC-SR04) modules |
| **Interface** | Flask dashboard + SSE telemetry API |
| **Storage / Logs** | SQLite + JSONL loggers |
| **Deployment** | Systemd services + udev rules + venv environment |
| **Language** | Python / Arduino C++ |
| **License** | MIT |

---

## 🧩 System Architecture

```
             ┌───────────────────────────────┐
             │        Raspberry Pi 5         │
             │  (AI Brain + Flask Dashboard) │
             │   • Face / Eye Detection      │
             │   • Lane Detection (CV)       │
             │   • Autopilot Logic & Logs    │
             │   • Fingerprint Logging       │
             └─────────────┬─────────────────┘
                           │ Serial (115200 baud)
                           ▼
             ┌───────────────────────────────┐
             │            ESP32              │
             │   • PWM Motor Control         │
             │   • Ultrasonic Sensors        │
             │   • Buzzer & Status LEDs      │
             │   • Unified Protocol          │
             └─────────────┬─────────────────┘
                           ▼
             ┌───────────────────────────────┐
             │      Motors & Environment     │
             │  (Obstacle Detection + Lane)  │
             └───────────────────────────────┘
```

---

## 🔧 Installation & Quick Start

```bash
# Clone & prepare environment
git clone https://github.com/<your-username>/DriveSentinel.git
cd DriveSentinel
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

### 🧠 Run AI Modules (v1)
```bash
python pc/launch_driver_monitor.py          # Drowsiness monitor
python pc/lane_detector_cli.py --show       # Lane visualization
python pc/fingerprint_logger_cli.py         # Fingerprint log listener
(cd dashboard && python app.py)             # Flask telemetry UI
```

### 🧩 Deploy as Services (on Pi 5)
```bash
sudo cp deploy/*.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now driver-monitor fingerprint-logger
```

### 🧰 Run Future v2 Modules
```bash
pip install -r v2/requirements-v2.txt
python -m v2.pc_v2.driver_monitor_v2 --port /dev/ttyESP32 --baud 115200 --cam 0
(cd v2/dashboard_v2 && python app.py)
```

---

## 🧠 AI Vision Features
- **Face & Eye Detection** using MediaPipe FaceMesh.  
  → Triggers *Autopilot ON* when eyes stay closed > threshold.  
- **Lane Detection (CV)** with Canny + HoughLines + ROI masking.  
- **Dynamic Thresholds** (`ear_thresh`, `ear_frames`, `release_seconds`) configurable in `config/config.toml`.

---

## 🔒 Security & Fingerprint Access
- R307 fingerprint sensor connected via UART 2 (ESP32 GPIO16/17).  
- Arduino console handles **enroll / verify / delete**, emitting `[SAVE] <name>` to Pi.  
- Pi app (`fingerprint_logger_cli.py`) logs enrollments to `fingerprints.txt` / `fingerprints.jsonl`.

---

## 📊 Dashboard & Telemetry
- **v1 Dashboard:** Flask + AJAX polling (port 8080).  
- **v2 Dashboard:** Flask + Server-Sent Events (SSE) live stream (port 8181).  
- Displays:
  - Current STATE, LANE, DIST values
  - Warnings & errors
  - Real-time event timeline

---

## 🧩 Unified Serial Protocol
```
Pi → ESP32:
  CMD:MODE:ON | CMD:MODE:OFF
  CMD:SPEED:<0–100>
  CMD:BUZZER:ON | OFF

ESP32 → Pi:
  ACK:MODE:ON | OFF
  STATE:DRIVING | SAFE_STOP | EMERGENCY
  DIST:F=<cm> R=<cm> B=<cm>
  LANE:LEFT | CENTER | RIGHT
  WARN:OBSTACLE:<side>:<cm>
  ERR:<message>
```

---

## 🧰 Development, CI & Testing
```bash
pip install -r requirements-dev.txt
pre-commit install
pre-commit run --all-files
pytest v2/tests
```
> GitHub Actions (`.github/workflows/ci.yml`) runs ruff + black + pytest on push / PR.

---

## Versions
This repository contains both **v1 (stable)** and **v2 (future)** in the same folder structure. See **[VERSIONS.md](VERSIONS.md)** for a quick matrix and how to run each.

---

## 📜 License & Credits
**License:** MIT  
**Developed by:** Mohamed Sherif Ali  
**Affiliation (research):** The British University in Egypt — Computer Engineering Programme  

---

## 🏁 Version
**DriveSentinel v0.1.0 — Initial Integrated Release**
