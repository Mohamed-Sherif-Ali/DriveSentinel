#!/usr/bin/env python3
import sys
import os
import pathlib

CONFIG_PATH = os.environ.get(
    "APP_CONFIG",
    str(pathlib.Path(__file__).resolve().parents[1] / "config" / "config.toml"),
)

try:
    import tomllib as tomli
except Exception:
    import tomli  # type: ignore

with open(CONFIG_PATH, "rb") as f:
    cfg = tomli.load(f)

port = cfg.get("fingerprint", {}).get("logger", {}).get("port", "/dev/ttyUSB0")
baud = str(cfg.get("fingerprint", {}).get("logger", {}).get("baud", 57600))

script = pathlib.Path(__file__).resolve().parent / "fingerprint_logger_cli.py"
os.execvp(sys.executable, [sys.executable, str(script), "--port", port, "--baud", baud])
