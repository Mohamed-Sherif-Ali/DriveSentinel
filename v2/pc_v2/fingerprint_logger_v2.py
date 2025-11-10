#!/usr/bin/env python3
import argparse
import json
import time
import serial


def parse_args():
    p = argparse.ArgumentParser(description="Fingerprint Logger v2 (JSONL)")
    p.add_argument("--port", default="/dev/ttyESP32")
    p.add_argument("--baud", type=int, default=57600)
    p.add_argument("--out", default="fingerprints.jsonl")
    return p.parse_args()


def main():
    args = parse_args()
    with serial.Serial(args.port, args.baud, timeout=1) as ser, open(
        args.out, "a", encoding="utf-8"
    ) as f:
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if line.startswith("[SAVE]"):
                f.write(json.dumps({"ts": time.time(), "line": line}) + "\n")
                f.flush()
                print("saved:", line, flush=True)


if __name__ == "__main__":
    main()
