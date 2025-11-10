# Versions Overview

This repository ships **two maintained tracks** in a single folder, ready for GitHub:

- **v1 (Stable)** — Root of the repo. Ready for deployment on the Pi 5 and ESP32.
  - Arduino (unified protocol): `arduino/esp32_autopilot/esp32_autopilot_unified.ino`
  - Pi apps: `pc/driver_monitor_cli.py`, `pc/lane_detector_cli.py`, `pc/fingerprint_logger_cli.py`
  - Dashboard: `dashboard/`
  - Config & services: `config/`, `deploy/`
  - Use this for **production demos** today.

- **v2 (Future)** — Next iteration, lives in `v2/` to avoid breaking v1.
  - Modular Python package in `v2/pc_v2/` with protocol parser and telemetry store
  - SSE dashboard in `v2/dashboard_v2/`
  - ESP32 sketch with **optional checksum** ideas in `v2/esp32_v2/`
  - Additional systemd units and tests

## When to use which?

| Scenario | Use v1 | Use v2 |
|---|---|---|
| Quick deploy on Pi + ESP32 | ✅ | |
| Stable serial protocol right now | ✅ | |
| Experiment with checksum framing/SSE | | ✅ |
| Unit testing for protocol | | ✅ |
| Keep demo stable while iterating | ✅ | ✅ (in `v2/`) |

## How to run

### v1 (root)
```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python pc/launch_driver_monitor.py
python pc/lane_detector_cli.py --show
python pc/fingerprint_logger_cli.py
# Dashboard
(cd dashboard && python app.py)
```

### v2
```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt -r v2/requirements-v2.txt
python -m v2.pc_v2.driver_monitor_v2 --port /dev/ttyESP32 --baud 115200 --cam 0
(cd v2/dashboard_v2 && python app.py)
```

> Both versions can run side-by-side if you use different ports or machines.
