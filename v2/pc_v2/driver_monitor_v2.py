#!/usr/bin/env python3
import argparse
import threading
import time
import logging
from logging.handlers import RotatingFileHandler

import cv2
import numpy as np
import serial

from .serial_protocol import format_cmd_mode
from .telemetry_store import open_db, add as db_add, prune as db_prune


def parse_args():
    p = argparse.ArgumentParser(description="Driver Drowsiness Monitor v2")
    p.add_argument("--port", default="/dev/ttyESP32")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--cam", type=int, default=0)
    p.add_argument("--ear-thresh", type=float, default=0.25)
    p.add_argument("--ear-frames", type=int, default=20)
    p.add_argument("--release", type=float, default=2.0)
    p.add_argument("--db", default="security_v2.db")
    p.add_argument("--log", default="monitor_v2.log")
    return p.parse_args()


def setup_logger(path):
    logger = logging.getLogger("monitor_v2")
    logger.setLevel(logging.INFO)
    fh = RotatingFileHandler(path, maxBytes=1_000_000, backupCount=3)
    fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logger.addHandler(fh)
    return logger


def serial_reader(ser, db):
    while True:
        try:
            s = ser.readline().decode(errors="ignore").strip()
            if not s:
                continue
            db_add(db, "ESP32", s)
        except Exception:
            time.sleep(0.2)


def main():
    args = parse_args()
    # log = setup_logger(args.log)
    db = open_db(args.db)
    ser = serial.Serial(args.port, args.baud, timeout=1)
    t = threading.Thread(target=serial_reader, args=(ser, db), daemon=True)
    t.start()

    import mediapipe as mp

    face_mesh = mp.solutions.face_mesh.FaceMesh(
        static_image_mode=False, max_num_faces=1, refine_landmarks=True
    )
    cap = cv2.VideoCapture(args.cam)

    consec = 0
    autopilot = False
    last_open = time.time()

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            res = face_mesh.process(rgb)
            ear = None
            if res.multi_face_landmarks:
                h, w = frame.shape[:2]

                def p(idx):
                    lm = res.multi_face_landmarks[0].landmark[idx]
                    return np.array([lm.x * w, lm.y * h], dtype=np.float32)

                def ear4(a, b, c, d):
                    return (np.linalg.norm(b - c) + np.linalg.norm(a - d)) / (
                        2.0 * np.linalg.norm(a - c)
                    )

                try:
                    L = ear4(p(159), p(145), p(33), p(133))
                    R = ear4(p(386), p(374), p(263), p(362))
                    ear = float((L + R) / 2.0)
                except Exception:
                    pass

            ts = time.time()
            if ear is not None:
                if ear < args.ear_thresh:
                    consec += 1
                    if not autopilot and consec >= args.ear_frames:
                        ser.write((format_cmd_mode(True) + "\n").encode())
                        db_add(db, "CMD", "MODE:ON")
                        autopilot = True
                else:
                    consec = 0
                    if autopilot and (ts - last_open) >= args.release:
                        ser.write((format_cmd_mode(False) + "\n").encode())
                        db_add(db, "CMD", "MODE:OFF")
                        autopilot = False
                    last_open = ts

            if int(ts) % 30 == 0:
                db_prune(db)
    finally:
        try:
            cap and cap.release()
        except Exception:
            pass
        try:
            face_mesh and face_mesh.close()
        except Exception:
            pass
        try:
            ser and ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
