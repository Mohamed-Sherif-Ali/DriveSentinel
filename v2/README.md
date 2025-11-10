# v2 (Future Improvements)

This folder contains a clean, modularized second iteration ("v2") of the project. It keeps the **v1** code intact, and introduces a more formal architecture, better testability, and an upgraded dashboard with Server‑Sent Events (SSE).

## Highlights
- **Modular Python package** (`pc_v2/`) with clear separation of concerns:
  - `serial_protocol.py`: strict parser/formatter for `CMD/ACK/STATE/DIST/LANE/WARN/ERR`.
  - `telemetry_store.py`: SQLite store with WAL, pruning, and typed records.
  - `driver_monitor_v2.py`: drowsiness detection logic (MediaPipe) wired through the protocol.
  - `lane_detector_v2.py`: improved smoothing and lane confidence (moving-average window).
  - `fingerprint_logger_v2.py`: deterministic logger with optional JSONL format.
- **Dashboard v2 (Flask + SSE)** (`dashboard_v2/`):
  - Streams telemetry in real‑time using Server‑Sent Events.
  - Minimal front-end that updates without polling.
- **ESP32 v2 (esp32_v2/)**:
  - Unified protocol with a simple checksum `*XX` at end of line (optional).
  - Robust line reader and token validator.
  - Same pins and PWM model from v1 for compatibility.
- **Tests** (`tests/`):
  - Unit tests for protocol parsing (pytest).
- **Deployment** (`deploy/`):
  - Hardened services for the v2 apps (`driver-monitor-v2.service`, `dashboard-v2.service`).

> v2 is deliberately separate so you can iterate without affecting v1 stability.
