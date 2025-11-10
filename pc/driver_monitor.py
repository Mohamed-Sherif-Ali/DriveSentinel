#!/usr/bin/env python3

import cv2
import mediapipe as mp
import numpy as np
import time
import sqlite3
import serial
import threading

# ─────────────────────────────
# Configuration
WEBCAM_IDX = 0
ESP32_PORT = "/dev/serial0"
ESP32_BAUD = 115200
EAR_THRESH = 0.25
EAR_CONSEC_FRAMES = 20
ALERT_RELEASE = 2.0  # seconds eyes must stay open before turning off
DB_FILE = "security.db"

# Thread-coordination
DB_LOCK = threading.Lock()
running = True  # flag to stop the ESP reader thread

# ─────────────────────────────
# Main thread DB setup
conn = sqlite3.connect(DB_FILE, check_same_thread=False)
c = conn.cursor()
c.execute(
    """
CREATE TABLE IF NOT EXISTS logs (
    ts      REAL,
    type    TEXT,
    details TEXT
)
"""
)
conn.commit()


def log_event(evt_type, details=""):
    """Log to SQLite and stdout."""
    ts = time.time()
    with DB_LOCK:
        c.execute("INSERT INTO logs VALUES (?, ?, ?)", (ts, evt_type, details))
        conn.commit()
    print(f"[LOG] {evt_type}: {details}")


# ─────────────────────────────
# Serial / ESP32 setup
try:
    ser = serial.Serial(ESP32_PORT, ESP32_BAUD, timeout=1)
    time.sleep(2)
    print("✅ Connected to ESP32")
except Exception as e:
    print(f"⚠️ Could not connect to ESP32: {e}")
    ser = None


def send_esp(cmd: str):
    """Thread-safe send to ESP32."""
    if not ser:
        return
    with threading.Lock():
        ser.write(cmd.encode())
    log_event("ESP_SEND", cmd.strip())


# ─────────────────────────────
# ESP reader thread (its own DB conn)
def esp_reader():
    local_conn = sqlite3.connect(DB_FILE, check_same_thread=False)
    local_c = local_conn.cursor()
    while running and ser:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                ts = time.time()
                with DB_LOCK:
                    local_c.execute(
                        "INSERT INTO logs VALUES (?, ?, ?)", (ts, "ESP_RECV", line)
                    )
                    local_conn.commit()
                print(f"[LOG] ESP_RECV: {line}")
        except Exception as e:
            ts = time.time()
            with DB_LOCK:
                local_c.execute(
                    "INSERT INTO logs VALUES (?, ?, ?)", (ts, "ESP_ERROR", str(e))
                )
                local_conn.commit()
            break
    local_conn.close()


reader_thread = threading.Thread(target=esp_reader)
reader_thread.start()

# ─────────────────────────────
# MediaPipe face-mesh setup
LEFT_EYE_INDICES = [362, 385, 387, 263, 373, 380]
RIGHT_EYE_INDICES = [33, 160, 158, 133, 153, 144]
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(
    static_image_mode=False,
    max_num_faces=1,
    refine_landmarks=True,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
)


def eye_aspect_ratio(lm, idxs):
    pts = np.array([(lm[i].x, lm[i].y) for i in idxs])
    A = np.linalg.norm(pts[1] - pts[5])
    B = np.linalg.norm(pts[2] - pts[4])
    C = np.linalg.norm(pts[0] - pts[3])
    return (A + B) / (2.0 * C)


# ─────────────────────────────
# Main monitoring loop
def monitor():
    cap = cv2.VideoCapture(WEBCAM_IDX)
    if not cap.isOpened():
        print(f"⚠️ Could not open camera {WEBCAM_IDX}")
        return

    print("🚀 Driver monitoring started")
    ear_counter = 0
    in_autopilot = False
    awake_start = None

    while True:
        ret, frame = cap.read()
        if not ret:
            log_event("ERROR", "Frame capture failed")
            time.sleep(0.1)
            continue

        # Compute EAR
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = face_mesh.process(rgb)
        if results.multi_face_landmarks:
            lm = results.multi_face_landmarks[0].landmark
            ear = (
                eye_aspect_ratio(lm, LEFT_EYE_INDICES)
                + eye_aspect_ratio(lm, RIGHT_EYE_INDICES)
            ) / 2.0
        else:
            ear = 0.0  # no face = treat as closed

        log_event("EAR_UPDATE", f"{ear:.3f}")

        # Eyes closed?
        if ear < EAR_THRESH:
            ear_counter += 1
            awake_start = None
            if ear_counter >= EAR_CONSEC_FRAMES and not in_autopilot:
                in_autopilot = True
                send_esp("autopilot mode:on\n")
                log_event("AUTOPILOT", "Mode ON sent")

        else:
            # Eyes open
            ear_counter = 0
            if in_autopilot:
                if awake_start is None:
                    awake_start = time.time()
                elif time.time() - awake_start >= ALERT_RELEASE:
                    send_esp("autopilot mode:off\n")
                    log_event("AUTOPILOT", "Mode OFF sent")
                    in_autopilot = False
                    awake_start = None

        time.sleep(0.05)

    cap.release()


# ─────────────────────────────
if __name__ == "__main__":
    try:
        monitor()
    except KeyboardInterrupt:
        print("\n[!] Exiting monitoring")
    finally:
        running = False
        reader_thread.join()
        conn.close()
        if ser:
            ser.close()
        print("[+] Resources cleaned up")
