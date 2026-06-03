"""
Rescue Drone — laptop controller (XIAO ESP32S3 Sense)
======================================================
Architecture:
  Thread 1 (main)      — cv2.imshow + waitKey only  → never blocks UI
  Thread 2 (stream)    — reads MJPEG, decodes frames
  Thread 3 (inference) — runs YOLOv8, draws boxes
  Thread 4 (alert)     — email + Telegram (spawned on detection)

Set XIAO_IP in config.py, then:  python rescue_drone.py
"""

from __future__ import annotations

import smtplib
import sys
import threading
import time
from collections import deque
from email.mime.image import MIMEImage
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from pathlib import Path

import cv2
import numpy as np
import requests
from ultralytics import YOLO

try:
    import config
except ImportError:
    print("[error] config.py not found")
    sys.exit(1)

# ── Config ────────────────────────────────────────────────────────────────────
ROOT       = Path(__file__).resolve().parent.parent
ALERTS_DIR = ROOT / "alerts"
XIAO_IP    = getattr(config, "XIAO_IP", "192.168.1.100")
STREAM_URL = f"http://{XIAO_IP}/stream"

CONFIDENCE       = 0.50
TARGET_CLASS     = "person"
ALERT_COOLDOWN   = 15          # seconds between alerts
STREAM_TIMEOUT   = 10          # seconds for HTTP connect
INFER_SIZE       = 320         # YOLO input size — 320 is faster than 640, still accurate

CLR_PERSON  = (0, 0, 255)
CLR_OTHER   = (0, 200, 100)
CLR_OVERLAY = (0, 229, 255)
FONT        = cv2.FONT_HERSHEY_SIMPLEX


# ── Shared state (thread-safe) ────────────────────────────────────────────────
class SharedState:
    def __init__(self):
        self._lock       = threading.Lock()
        self.display     = None   # latest annotated frame ready to show
        self.raw_queue   = deque(maxlen=2)  # raw frames for inference (drop old ones)
        self.fps         = 0
        self.person_count = 0
        self.running     = True

    def push_raw(self, frame):
        with self._lock:
            self.raw_queue.append(frame)

    def pop_raw(self):
        with self._lock:
            return self.raw_queue.popleft() if self.raw_queue else None

    def set_display(self, frame, fps, persons):
        with self._lock:
            self.display      = frame
            self.fps          = fps
            self.person_count = persons

    def get_display(self):
        with self._lock:
            return self.display, self.fps, self.person_count

    def stop(self):
        with self._lock:
            self.running = False

    def is_running(self):
        with self._lock:
            return self.running


# ── MJPEG stream reader thread ────────────────────────────────────────────────
def stream_thread(state: SharedState):
    while state.is_running():
        try:
            resp = requests.get(STREAM_URL, stream=True, timeout=STREAM_TIMEOUT)
            resp.raise_for_status()
            buf = b""
            for chunk in resp.iter_content(chunk_size=8192):
                if not state.is_running():
                    break
                buf += chunk
                while True:
                    start = buf.find(b"\xff\xd8")
                    end   = buf.find(b"\xff\xd9", start)
                    if start == -1 or end == -1:
                        break
                    jpg = buf[start : end + 2]
                    buf = buf[end + 2 :]
                    frame = cv2.imdecode(
                        np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR
                    )
                    if frame is not None:
                        state.push_raw(frame)
        except Exception as e:
            if state.is_running():
                print(f"[stream] {e} — retrying in 2 s")
                time.sleep(2)


# ── YOLO inference thread ─────────────────────────────────────────────────────
def inference_thread(state: SharedState, model: YOLO, last_alert: list):
    fps_time  = time.time()
    fps_count = 0
    fps_val   = 0

    while state.is_running():
        frame = state.pop_raw()
        if frame is None:
            time.sleep(0.005)   # no frame yet — yield CPU
            continue

        fps_count += 1
        now = time.time()
        if now - fps_time >= 1.0:
            fps_val   = fps_count
            fps_count = 0
            fps_time  = now

        # Run YOLO at reduced input size for speed
        results      = model(frame, verbose=False, conf=CONFIDENCE, imgsz=INFER_SIZE)
        annotated    = frame.copy()
        person_count = draw_detections(annotated, model, results)

        # Overlay HUD
        overlay_hud(annotated, fps_val, person_count)

        if person_count > 0:
            alert_label = "ALERT SENT" if (now - last_alert[0]) < 3 else "PERSON!"
            cv2.putText(annotated, alert_label,
                        (annotated.shape[1] // 2 - 80, 80),
                        FONT, 1.0, CLR_PERSON, 3)
            # Snapshot for alert (clean copy without HUD flash text)
            snap = frame.copy()
            draw_detections(snap, model, results)
            trigger_alerts(snap, last_alert)

        state.set_display(annotated, fps_val, person_count)


# ── Drawing helpers ───────────────────────────────────────────────────────────
def draw_detections(frame, model, results) -> int:
    person_count = 0
    for result in results:
        for box in result.boxes:
            cls   = int(box.cls[0])
            conf  = float(box.conf[0])
            label = model.names[cls]
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            is_target = label == TARGET_CLASS
            color     = CLR_PERSON if is_target else CLR_OTHER
            thickness = 3 if is_target else 2
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
            tag = f"{label} {conf:.0%}"
            tw, th = cv2.getTextSize(tag, FONT, 0.6, 2)[0]
            cv2.rectangle(frame, (x1, y1 - th - 8), (x1 + tw + 4, y1), color, -1)
            cv2.putText(frame, tag, (x1 + 2, y1 - 4), FONT, 0.6, (255, 255, 255), 2)
            if is_target:
                person_count += 1
    return person_count


def overlay_hud(frame, fps_val: int, person_count: int) -> None:
    h, w = frame.shape[:2]
    cv2.rectangle(frame, (0, 0), (w, 36), (15, 20, 30), -1)
    cv2.putText(frame, f"FPS: {fps_val}",         (8,   24), FONT, 0.6, CLR_OVERLAY, 2)
    cv2.putText(frame, f"Person: {person_count}", (120, 24), FONT, 0.6,
                CLR_PERSON if person_count else CLR_OVERLAY, 2)
    cv2.putText(frame, XIAO_IP, (w - 180, 24),   FONT, 0.45, CLR_OVERLAY, 1)


# ── Alert helpers ─────────────────────────────────────────────────────────────
def trigger_alerts(frame, last_alert: list) -> None:
    now = time.time()
    if now - last_alert[0] < ALERT_COOLDOWN:
        return
    last_alert[0] = now

    ALERTS_DIR.mkdir(parents=True, exist_ok=True)
    ts   = time.strftime("%Y%m%d_%H%M%S")
    path = ALERTS_DIR / f"person_{ts}.jpg"
    ok, jpeg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    if not ok:
        return
    jpeg_bytes = jpeg.tobytes()
    path.write_bytes(jpeg_bytes)
    print(f"[alert] Saved {path}")

    threading.Thread(
        target=lambda: (send_email(jpeg_bytes), send_telegram(jpeg_bytes)),
        name="alert", daemon=True,
    ).start()


def send_email(jpeg_bytes: bytes) -> None:
    try:
        msg            = MIMEMultipart()
        msg["From"]    = f"{config.FROM_NAME} <{config.FROM_EMAIL}>"
        msg["To"]      = config.TO_EMAIL
        msg["Subject"] = "Rescue Drone — Person Detected"
        msg.attach(MIMEText("Person detected by Rescue Drone.\n", "plain"))
        img = MIMEImage(jpeg_bytes, _subtype="jpeg")
        img.add_header("Content-Disposition", "attachment", filename="detection.jpg")
        msg.attach(img)
        with smtplib.SMTP_SSL("smtp.gmail.com", 465, timeout=30) as smtp:
            smtp.login(config.FROM_EMAIL, config.APP_PASSWORD)
            smtp.sendmail(config.FROM_EMAIL, [config.TO_EMAIL], msg.as_string())
        print("[email] Sent OK")
    except Exception as e:
        print(f"[email] FAILED: {e}")


def send_telegram(jpeg_bytes: bytes) -> None:
    token    = config.TG_BOT_TOKEN
    chat_ids = getattr(config, "TG_CHAT_ID", None) or getattr(config, "TG_CHAT_IDs", None)
    if not chat_ids:
        return
    if isinstance(chat_ids, (str, int)):
        chat_ids = [chat_ids]
    base = f"https://api.telegram.org/bot{token}"
    try:
        for cid in chat_ids:
            requests.post(f"{base}/sendMessage",
                          json={"chat_id": cid, "text": "*Rescue Drone*\nPerson detected",
                                "parse_mode": "Markdown"}, timeout=15)
            requests.post(f"{base}/sendPhoto",
                          data={"chat_id": cid},
                          files={"photo": ("detection.jpg", jpeg_bytes, "image/jpeg")},
                          timeout=30)
        print("[telegram] Sent OK")
    except Exception as e:
        print(f"[telegram] FAILED: {e}")


# ── Entry point ───────────────────────────────────────────────────────────────
def main() -> None:
    model_path = ROOT / "yolov8n.pt"
    if not model_path.is_file():
        model_path = Path(__file__).parent / "yolov8n.pt"

    print("=" * 50)
    print("  Rescue Drone — XIAO ESP32S3 + YOLO")
    print("=" * 50)
    print(f"  Stream : {STREAM_URL}")
    print(f"  Model  : {model_path.name}  imgsz={INFER_SIZE}")
    print(f"  Target : {TARGET_CLASS}  conf>={CONFIDENCE}")
    print("  Q = quit")
    print("=" * 50)

    print("[yolo] Loading model...")
    model = YOLO(str(model_path))
    # Warm-up pass so the first real frame isn't slow
    model(np.zeros((INFER_SIZE, INFER_SIZE, 3), dtype=np.uint8),
          verbose=False, imgsz=INFER_SIZE)
    print("[yolo] Ready")

    state      = SharedState()
    last_alert = [0.0]

    # Start background threads
    t_stream = threading.Thread(target=stream_thread,
                                args=(state,), name="stream", daemon=True)
    t_infer  = threading.Thread(target=inference_thread,
                                args=(state, model, last_alert),
                                name="inference", daemon=True)
    t_stream.start()
    t_infer.start()

    window = "Rescue Drone  (Q to quit)"
    cv2.namedWindow(window)

    # ── Main thread: display only ─────────────────────────────────────────────
    # waitKey(1) keeps Windows happy — this loop never does heavy work
    placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(placeholder, "Connecting to XIAO...", (140, 240),
                FONT, 0.9, CLR_OVERLAY, 2)

    try:
        while True:
            frame, _, _ = state.get_display()
            cv2.imshow(window, frame if frame is not None else placeholder)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        state.stop()
        cv2.destroyAllWindows()
        print("[exit] Stopped")


if __name__ == "__main__":
    main()
