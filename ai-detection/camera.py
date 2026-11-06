import ctypes
import threading
from pathlib import Path
import time

import cv2
import numpy as np
import requests
from ultralytics import YOLO

try:
    from pynput import keyboard as pynput_keyboard
except ImportError:
    pynput_keyboard = None

ESP32 = "http://192.168.4.1"
STREAM_URL = ESP32 + ":81/stream"
MODEL_NAME = Path(__file__).resolve().parent / "models" / "yolo11n-seg.pt"
WINDOW = "GarbageBot - Segmentation and Control"

KEY_HOLD_SECONDS = 0.6   # only used in the fallback mode (no pynput)
CONTROL_KEYS = {"w", "a", "s", "d", "x", "space"}

running = True
lock = threading.Lock()

latest_frame = None
latest_frame_id = 0
annotated = None
video_ok = False
gps_text = "GPS: waiting for data"

pressed = set()     # real key state (pynput mode)
last_seen = {}      # key -> last time seen (fallback mode)


def window_focused():
    """Only drive the robot while the video window is in front."""
    try:
        hwnd = ctypes.windll.user32.GetForegroundWindow()
        buf = ctypes.create_unicode_buffer(256)
        ctypes.windll.user32.GetWindowTextW(hwnd, buf, 256)
        return buf.value == WINDOW
    except Exception:
        return True


# ---------------- Video grabber (always keeps the newest frame) ----------------
def grab_loop():
    global latest_frame, latest_frame_id, video_ok
    cap = None

    while running:
        if cap is None or not cap.isOpened():
            cap = cv2.VideoCapture(STREAM_URL, cv2.CAP_FFMPEG)
            if not cap.isOpened():
                video_ok = False
                cap = None
                time.sleep(1)
                continue

        ok, frame = cap.read()

        if not ok:
            video_ok = False
            cap.release()
            cap = None
            time.sleep(0.5)
            continue

        with lock:
            latest_frame = frame
            latest_frame_id += 1
        video_ok = True

    if cap is not None:
        cap.release()


# ---------------- YOLO inference (runs on newest frame only) ----------------
def inference_loop(model):
    global annotated
    last_id = 0

    while running:
        with lock:
            frame = latest_frame
            fid = latest_frame_id

        if frame is None or fid == last_id:
            time.sleep(0.01)
            continue

        last_id = fid

        try:
            result = model.predict(frame, imgsz=320, verbose=False)[0]
            out = result.plot()
        except Exception as error:
            print("Segmentation error:", error)
            out = frame

        with lock:
            annotated = out


# ---------------- Robot commands ----------------
def current_command():
    if not video_ok or not window_focused():
        return "S"

    now = time.monotonic()

    with lock:
        if pynput_keyboard is not None:
            keys = set(pressed)
        else:
            keys = {k for k, t in last_seen.items()
                    if now - t < KEY_HOLD_SECONDS}

    if "space" in keys or "x" in keys:
        return "S"
    if "w" in keys:
        return "F"
    if "s" in keys:
        return "B"
    if "a" in keys:
        return "L"
    if "d" in keys:
        return "R"
    return "S"


def command_loop():
    session = requests.Session()

    while running:
        command = current_command()
        try:
            session.get(ESP32 + "/cmd", params={"c": command}, timeout=0.4)
        except requests.RequestException:
            pass
        time.sleep(0.15)

    session.close()


def gps_loop():
    global gps_text
    session = requests.Session()

    while running:
        try:
            response = session.get(ESP32 + "/gps", timeout=1.0)
            text = response.text.strip()
        except requests.RequestException:
            text = "GPS: ESP32 unavailable"

        with lock:
            gps_text = text
        time.sleep(1)

    session.close()


# ---------------- Keyboard (true key down / key up) ----------------
def key_name(key):
    try:
        if key.char:
            return key.char.lower()
    except AttributeError:
        pass
    if key == pynput_keyboard.Key.space:
        return "space"
    return None


def on_press(key):
    name = key_name(key)
    if name in CONTROL_KEYS:
        with lock:
            pressed.add(name)


def on_release(key):
    name = key_name(key)
    if name in CONTROL_KEYS:
        with lock:
            pressed.discard(name)


def stop_robot():
    for _ in range(3):
        try:
            requests.get(ESP32 + "/cmd", params={"c": "S"}, timeout=0.4)
        except requests.RequestException:
            pass
        time.sleep(0.05)


def main():
    global running

    print("Loading model...")
    model = YOLO(MODEL_NAME)

    if pynput_keyboard is None:
        print("pynput not installed: using fallback keyboard mode.")
        print("Install it with: python -m pip install pynput")

    threading.Thread(target=grab_loop, daemon=True).start()
    threading.Thread(target=inference_loop, args=(model,), daemon=True).start()
    threading.Thread(target=command_loop, daemon=True).start()
    threading.Thread(target=gps_loop, daemon=True).start()

    listener = None
    if pynput_keyboard is not None:
        listener = pynput_keyboard.Listener(on_press=on_press,
                                            on_release=on_release)
        listener.start()

    cv2.namedWindow(WINDOW)
    print("Controls: W=forward, S=back, A=left, D=right")
    print("Space/X=stop, ESC=exit")
    print("Keep the video window focused to control the robot.")

    try:
        while running:
            with lock:
                frame = annotated if annotated is not None else latest_frame
                text = gps_text

            if frame is None:
                display = np.zeros((240, 320, 3), dtype=np.uint8)
                cv2.putText(display, "Waiting for video...", (20, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 255, 255), 2)
            else:
                display = frame.copy()

            status = "VIDEO OK" if video_ok else "VIDEO LOST - robot stopped"
            cv2.putText(display, "WASD: Move | SPACE/X: Stop | ESC: Exit",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 2)
            cv2.putText(display, text[:60], (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.putText(display, status, (10, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            cv2.imshow(WINDOW, display)
            key = cv2.waitKey(15)

            if key != -1:
                key &= 0xFF
                if key == 27:
                    break
                if pynput_keyboard is None:
                    name = "space" if key == 32 else chr(key).lower()
                    if name in CONTROL_KEYS:
                        with lock:
                            last_seen[name] = time.monotonic()

            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break

    finally:
        running = False
        if listener is not None:
            listener.stop()
        stop_robot()
        time.sleep(0.2)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()