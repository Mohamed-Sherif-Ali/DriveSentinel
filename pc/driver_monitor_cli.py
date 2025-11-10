#!/usr/bin/env python3
import argparse, sys, time, threading, sqlite3, logging
from logging.handlers import RotatingFileHandler

try:
    import tomllib as tomli
except Exception:
    import tomli  # type: ignore

import cv2
import numpy as np

try:
    import serial
except Exception:
    serial = None

SERIAL_LOCK = threading.Lock()

def setup_logger(path):
    logger = logging.getLogger("driver_monitor")
    if logger.handlers:
        return logger
    logger.setLevel(logging.INFO)
    fh = RotatingFileHandler(path, maxBytes=1_000_000, backupCount=3)
    fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logger.addHandler(fh)
    return logger

def open_db(path):
    db = sqlite3.connect(path, check_same_thread=False)
    db.execute("PRAGMA journal_mode=WAL")
    db.execute("PRAGMA synchronous=NORMAL")
    db.execute("CREATE TABLE IF NOT EXISTS events (ts REAL, kind TEXT, data TEXT)")
    return db

def prune_db(db, max_rows=200000):
    n = db.execute("SELECT COUNT(*) FROM events").fetchone()[0]
    if n > max_rows:
        to_delete = n - max_rows
        db.execute("DELETE FROM events WHERE rowid IN (SELECT rowid FROM events ORDER BY rowid ASC LIMIT ?)", (to_delete,))
        db.commit()

def parse_args():
    p = argparse.ArgumentParser(description="Driver Drowsiness Monitor (Pi5)")
    p.add_argument("--port", default="/dev/ttyESP32", help="ESP32 serial port")
    p.add_argument("--baud", type=int, default=115200, help="ESP32 baud rate")
    p.add_argument("--cam", type=int, default=0, help="Camera index")
    p.add_argument("--ear-thresh", type=float, default=0.25, help="EAR threshold")
    p.add_argument("--ear-frames", type=int, default=20, help="Frames below thresh to trigger")
    p.add_argument("--release", type=float, default=2.0, help="Seconds eyes-open to disable autopilot")
    p.add_argument("--db", default="security.db", help="SQLite DB path")
    p.add_argument("--log", default="monitor.log", help="Log file")
    p.add_argument("--config", default=None, help="Optional TOML config path (overrides flags)")
    return p.parse_args()

def load_config(path):
    with open(path, "rb") as f:
        return tomli.load(f)

def merge_args_with_config(args):
    if not args.config:
        return args
    cfg = load_config(args.config)
    args.port = cfg.get("serial", {}).get("esp32", {}).get("port", args.port)
    args.baud = cfg.get("serial", {}).get("esp32", {}).get("baud", args.baud)
    args.cam = cfg.get("camera", {}).get("index", args.cam)
    d = cfg.get("drowsiness", {})
    args.ear_thresh = d.get("ear_thresh", args.ear_thresh)
    args.ear_frames = d.get("ear_frames", args.ear_frames)
    args.release = d.get("release_seconds", args.release)
    return args

def safe_open_camera(index, retries=5, delay=1.0):
    cap = None
    for _ in range(retries):
        cap = cv2.VideoCapture(index)
        if cap.isOpened():
            return cap
        time.sleep(delay)
    raise RuntimeError(f"Camera {index} not available")

def send_esp(ser_conn, cmd: str):
    if not ser_conn: 
        return
    msg = (cmd.rstrip() + "\n").encode()
    with SERIAL_LOCK:
        ser_conn.write(msg)

def reader_thread(ser_conn, db, log):
    if not ser_conn:
        return
    while True:
        try:
            line = ser_conn.readline().decode(errors="ignore").strip()
            if not line:
                continue
            db.execute("INSERT INTO events VALUES (?,?,?)", (time.time(), "ESP32", line))
            db.commit()
            if line.startswith("ERROR") or line.startswith("ERR"):
                log.warning(line)
        except Exception as e:
            log.error(f"Serial read error: {e}")
            time.sleep(0.5)

def main():
    args = parse_args()
    args = merge_args_with_config(args)
    log = setup_logger(args.log)
    db = open_db(args.db)

    # Optional MediaPipe import happens here to avoid import cost on help
    import mediapipe as mp
    face_mesh = mp.solutions.face_mesh.FaceMesh(static_image_mode=False, max_num_faces=1, refine_landmarks=True)

    ser_conn = None
    try:
        if serial is not None:
            ser_conn = serial.Serial(args.port, args.baud, timeout=1)
    except Exception as e:
        log.error(f"Serial open failed on {args.port}: {e}")

    rt = threading.Thread(target=reader_thread, args=(ser_conn, db, log), daemon=True)
    rt.start()

    cap = None
    try:
        cap = safe_open_camera(args.cam)
        consec = 0
        autopilot_on = False
        last_open_ts = time.time()
        while True:
            ok, frame = cap.read()
            if not ok:
                log.warning("Camera read failed; retrying...")
                time.sleep(0.2)
                continue

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            res = face_mesh.process(rgb)
            ear = None
            if res.multi_face_landmarks:
                # Simple EAR using eye corner landmarks (approx indices for FaceMesh: left eye [33, 133, 159, 145], right eye [263, 362, 386, 374])
                h, w = frame.shape[:2]
                def p(idx): 
                    lm = res.multi_face_landmarks[0].landmark[idx]
                    return np.array([lm.x*w, lm.y*h], dtype=np.float32)
                def eye_ear(a, b, c, d):
                    # vertical distances / horizontal distance
                    return (np.linalg.norm(b - c) + np.linalg.norm(a - d)) / (2.0 * np.linalg.norm(a - c))
                try:
                    L = eye_ear(p(159), p(145), p(33), p(133))
                    R = eye_ear(p(386), p(374), p(263), p(362))
                    ear = float((L + R) / 2.0)
                except Exception:
                    ear = None

            ts = time.time()
            if ear is not None:
                if ear < args.ear_thresh:
                    consec += 1
                    if consec >= args.ear_frames and not autopilot_on:
                        send_esp(ser_conn, "CMD:MODE:ON")
                        db.execute("INSERT INTO events VALUES (?,?,?)", (ts, "CMD", "MODE:ON")); db.commit()
                        log.info("Autopilot ON (drowsy)")
                        autopilot_on = True
                else:
                    consec = 0
                    if autopilot_on:
                        # Count open-eye duration
                        if (ts - last_open_ts) >= args.release:
                            send_esp(ser_conn, "CMD:MODE:OFF")
                            db.execute("INSERT INTO events VALUES (?,?,?)", (ts, "CMD", "MODE:OFF")); db.commit()
                            log.info("Autopilot OFF (recovered)")
                            autopilot_on = False
                    last_open_ts = ts

            # periodic prune
            if int(ts) % 30 == 0:
                prune_db(db)

    except KeyboardInterrupt:
        log.info("Interrupted by user.")
    finally:
        try: cap and cap.release()
        except: pass
        try: face_mesh and face_mesh.close()
        except: pass
        try: ser_conn and ser_conn.close()
        except: pass

if __name__ == "__main__":
    main()
