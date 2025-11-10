from flask import Flask, Response, send_from_directory
import queue
import threading
import time
import serial

app = Flask(__name__, static_url_path="", static_folder="static")
q = queue.Queue(maxsize=1000)


def parse_line(s: str):
    # crude parser for protocol tokens
    parts = s.split(":")
    if not parts:
        return None
    return {"raw": s, "t": time.time()}


def serial_reader(port="/dev/ttyESP32", baud=115200):
    while True:
        try:
            with serial.Serial(port, baud, timeout=1) as ser:
                for raw in ser:
                    s = raw.decode(errors="ignore").strip()
                    if not s:
                        continue
                    data = parse_line(s)
                    if data:
                        try:
                            q.put_nowait(data)
                        except Exception:
                            # queue full; drop oldest
                            try:
                                q.get_nowait()
                            except Exception:
                                pass
                            try:
                                q.put_nowait(data)
                            except Exception:
                                pass
        except Exception:
            # If serial open fails, backoff a bit and retry
            time.sleep(1)


@app.get("/events")
def events():
    def gen():
        while True:
            item = q.get()
            yield f"data: {item}\n\n"

    return Response(gen(), mimetype="text/event-stream")


@app.get("/")
def index():
    return send_from_directory(app.static_folder, "index.html")


if __name__ == "__main__":
    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()
    app.run(host="0.0.0.0", port=8080, threaded=True)
