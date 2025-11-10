#!/usr/bin/env python3
import argparse
import serial
import time


def parse_args():
    p = argparse.ArgumentParser(description="Fingerprint Logger (v1)")
    p.add_argument("--port", default="/dev/ttyESP32")
    p.add_argument("--baud", type=int, default=57600)
    p.add_argument("--out", default="fingerprints.txt")
    return p.parse_args()


def main():
    args = parse_args()
    with serial.Serial(args.port, args.baud, timeout=1) as ser, open(
        args.out, "a", encoding="utf-8"
    ) as f:
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if line.startswith("[SAVE]"):
                f.write(line + "\n")
                f.flush()
                print("saved:", line, flush=True)
            time.sleep(0.01)


if __name__ == "__main__":
    main()
