from flask import Flask, Response, send_from_directory
import queue, threading, time
import serial

from pc_v2.serial_protocol import parse_line

app = Flask(__name__, static_url_path="", static_folder="static")
q = queue.Queue(maxsize=1000)

def serial_reader(port="/dev/ttyESP32", baud=115200):
    with serial.Serial(port, baud, timeout=1) as ser:
        for raw in ser:
            s = raw.decode(errors="ignore").strip()
            data = parse_line(s) or {"type":"RAW","raw":s}
            try: q.put_nowait(data)
            except Exception:
pass

@app.get("/events")
def events():
    def gen():
        while True:
            item = q.get()
            yield f"data: {item}\n\n"
    return Response(gen(), mimetype="text/event-stream" )

@app.get("/")
def index():
    return send_from_directory(app.static_folder, "index.html")

if __name__ == "__main__":
    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()
    app.run(host="0.0.0.0", port=8181, threaded=True)
