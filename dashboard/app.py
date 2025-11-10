from flask import Flask, jsonify, send_from_directory
import queue, threading, time, os

try:
    import serial
except Exception:
    serial = None

app = Flask(__name__, static_url_path="", static_folder="static")
q = queue.Queue(maxsize=500)

def parse_line(s: str):
    # crude parser for protocol tokens
    parts = s.split(":")
    if not parts: return None
    return {"raw": s, "t": time.time()}

@app.get("/api/telemetry")
def api_telemetry():
    items = []
    try:
        while True:
            items.append(q.get_nowait())
    except Exception:
        pass
    return jsonify(items)

@app.get("/")
def index():
    return send_from_directory(app.static_folder, "index.html")

def serial_reader(port="/dev/ttyESP32", baud=115200):
    if serial is None:
        return
    try:
        with serial.Serial(port, baud, timeout=1) as ser:
            for line in ser:
                s = line.decode(errors="ignore").strip()
                data = parse_line(s)
                if data:
                    try: q.put_nowait(data)
                    except: pass
    except Exception as e:
        while True:
            time.sleep(1)

if __name__ == "__main__":
    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()
    app.run(host="0.0.0.0", port=8080)
