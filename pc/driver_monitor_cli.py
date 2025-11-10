#!/usr/bin/env python3
import argparse
import time
import logging
from logging.handlers import RotatingFileHandler

import cv2
import numpy as np
import serial
import sqlite3


def parse_args():
    p = argparse.ArgumentParser(description="Driver Drowsiness Monitor (v1)")
    p.add_argument("--port", default="/dev/ttyESP32")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--cam", type=int, default=0)
    p.add_argument("--ear-thresh", type=float, default=0.25)
    p.add_argument("--ear-frames", type=int, default=20)
    p.add_argument("--release", type=float, default=2.0)
    p.add_argument("--db", default="security.db")
    p.add_argument("--log", default="monitor.log")
    return p.parse_args()


def setup_logger(path):
    logger = logging.getLogger("monitor")
    logger.setLevel(logging.INFO)
    fh = RotatingFileHandler(path, maxBytes=1_000_000, backupCount=3)
    fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logger.addHandler(fh)
    return logger


def open_db(path):
    db = sqlite3.connect(path, check_same_thread=False)
    db.execute("PRAGMA journal_mode=WAL")
    db.execute("PRAGMA synchronous=NORMAL")
    db.execute(
        """
        CREATE TABLE IF NOT EXISTS events (
          ts REAL,
          type TEXT,
          data TEXT
        );
        """
    )
    return db


def send_esp(ser, line: str):
    if ser:
        ser.write((line + "\n").encode())


def main():
    args = parse_args()
    log = setup_logger(args.log)
    db = open_db(args.db)

    try:
        ser_conn = serial.Serial(args.port, args.baud, timeout=1)
    except Exception:
        ser_conn = None
        log.warning("Could not open serial port; continuing without serial.")

    import mediapipe as mp

    face_mesh = mp.solutions.face_mesh.FaceMesh(
        static_image_mode=False, max_num_faces=1, refine_landmarks=True
    )
    cap = cv2.VideoCapture(args.cam)

    consec = 0
    autopilot_on = False
    last_open_ts = time.time()

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                log.warning("Camera read failed, retrying...")
                time.sleep(0.2)
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
                    # simple 2-pair average ratio
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
                    if not autopilot_on and consec >= args.ear_frames:
                        send_esp(ser_conn, "CMD:MODE:ON")
                        db.execute(
                            "INSERT INTO events VALUES (?,?,?)",
                            (ts, "CMD", "MODE:ON"),
                        )
                        db.commit()
                        log.info("Autopilot ON (drowsy)")
                        autopilot_on = True
                else:
                    consec = 0
                    if autopilot_on and (ts - last_open_ts) >= args.release:
                        send_esp(ser_conn, "CMD:MODE:OFF")
                        db.execute(
                            "INSERT INTO events VALUES (?,?,?)",
                            (ts, "CMD", "MODE:OFF"),
                        )
                        db.commit()
                        log.info("Autopilot OFF (recovered)")
                        autopilot_on = False
                    last_open_ts = ts

            # periodic prune
            if int(ts) % 30 == 0:
                try:
                    db.execute(
                        "DELETE FROM events WHERE rowid IN (SELECT rowid FROM events ORDER BY rowid ASC LIMIT 1000)"
                    )
                    db.commit()
                except Exception:
                    pass

    except KeyboardInterrupt:
        log.info("Interrupted by user.")
    finally:
        try:
            if cap:
                cap.release()
        except Exception:
            pass
        try:
            if face_mesh:
                face_mesh.close()
        except Exception:
            pass
        try:
            if ser_conn:
                ser_conn.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
