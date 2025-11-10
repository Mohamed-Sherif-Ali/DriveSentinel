from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Dict

# Simple normalized protocol:
# CMD:MODE:ON|OFF
# CMD:SPEED:<0-100>
# CMD:BUZZER:ON|OFF
# ACK:MODE:ON|OFF
# STATE:DRIVING|CHECKING_LANE|SWITCHING_LANE|SAFE_STOP|EMERGENCY
# DIST:F=<cm> R=<cm> B=<cm>
# LANE:LEFT|CENTER|RIGHT
# WARN:OBSTACLE:<FRONT|RIGHT|BACK>:<cm>
# ERR:<message>
#
# Optional checksum: "<line>*XX" where XX is 2-digit hex of xor of ASCII bytes before '*'.

def checksum(line: str) -> int:
    return 0 if not line else (sum(line.encode('ascii', 'ignore')) & 0xFF)

def append_checksum(line: str) -> str:
    return f"{line}*{checksum(line):02X}"

def validate_and_strip_checksum(raw: str) -> Optional[str]:
    if '*' not in raw:
        return raw  # no checksum used
    try:
        data, tag = raw.rsplit('*', 1)
        want = int(tag[:2], 16)
        have = checksum(data)
        return data if want == have else None
    except Exception:
        return None

def parse_line(raw: str) -> Optional[Dict[str, str]]:
    s = validate_and_strip_checksum(raw.strip())
    if s is None or not s:
        return None
    parts = s.split(':')
    head = parts[0]
    if head in ('CMD', 'ACK'):
        return {'type': head, 'key': parts[1], 'val': parts[2] if len(parts) > 2 else ''}
    if head == 'STATE':
        return {'type': 'STATE', 'val': parts[1] if len(parts) > 1 else ''}
    if head == 'LANE':
        return {'type': 'LANE', 'val': parts[1] if len(parts) > 1 else ''}
    if head == 'DIST':
        # DIST:F=123 R=88 B=999
        vals = {}
        for tok in parts[1:]:
            for kv in tok.split(' '):
                if '=' in kv:
                    k,v = kv.split('=',1)
                    vals[k] = v
        return {'type': 'DIST', **vals}
    if head == 'WARN':
        # WARN:OBSTACLE:RIGHT:7
        return {'type': 'WARN', 'what': parts[1] if len(parts)>1 else '', 'side': parts[2] if len(parts)>2 else '', 'val': parts[3] if len(parts)>3 else ''}
    if head == 'ERR':
        return {'type': 'ERR', 'msg': ':'.join(parts[1:])}
    return {'type': 'RAW', 'raw': s}

def format_cmd_mode(on: bool) -> str:
    return append_checksum(f"CMD:MODE:{'ON' if on else 'OFF'}")

def format_cmd_speed(pct: int) -> str:
    pct = max(0, min(100, int(pct)))
    return append_checksum(f"CMD:SPEED:{pct}")

def format_cmd_buzzer(on: bool) -> str:
    return append_checksum(f"CMD:BUZZER:{'ON' if on else 'OFF'}")
