#!/usr/bin/env python3
import sys, os, pathlib

CONFIG_PATH = os.environ.get("APP_CONFIG", str(pathlib.Path(__file__).resolve().parents[1] / "config" / "config.toml"))
script = pathlib.Path(__file__).resolve().parent / "driver_monitor_cli.py"
os.execvp(sys.executable, [sys.executable, str(script), "--config", CONFIG_PATH])
