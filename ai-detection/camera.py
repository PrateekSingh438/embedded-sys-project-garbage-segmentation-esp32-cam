import csv
import ctypes
import math
import os
import threading
import time
from collections import Counter
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import requests
from ultralytics import YOLO

try:
    from pynput import keyboard as pynput_keyboard
except ImportError:
    pynput_keyboard = None

# Folder that contains this script (ai-detection). All paths are built from it,
# so the script works no matter which folder you start it from.
BASE_DIR = Path(__file__).resolve().parent

# =============================== SETTINGS ===============================
ESP32 = "http://192.168.4.1"
STREAM_URL = ESP32 + ":81/stream"
WINDOW = "GarbageBot - Segmentation and Control"
ROBOT_ID = "GB-01"

# ---- Model ----
MODEL_NAME = str(BASE_DIR / "models" / "garbage-seg.pt")      # from train_taco.py
FALLBACK_MODEL = str(BASE_DIR / "models" / "yolo11n-seg.pt")  # generic COCO model
FALLBACK_COCO_FILTER = {"bottle", "cup", "wine glass", "bowl", "handbag",
                        "backpack", "fork", "knife", "spoon"}
INFER_IMGSZ = 416                      # 320 = faster, 480 = more accurate
CONF_MIN = 0.40                        # ignore detections below this confidence
CLASS_FILTER = None                    # None = log every class of the model

# ---- Window size ----
CAM_DISPLAY_WIDTH = 800                # camera picture width on screen (pixels)
MAP_PANEL_WIDTH = 440                  # width of the garbage map panel
# The window can also be dragged larger with the mouse.

# ---- ORIGIN: used as the position whenever there is no GPS fix ----
ORIGIN_LAT = 25.495572                 # set to the place where you start the robot
ORIGIN_LNG = 81.867973
DEAD_RECKON_WITHOUT_GPS = True         # True: robot position = origin + estimated movement
                                       # False: robot position stays exactly at the origin

# ---- Dead-reckoning calibration (drive 2 s forward / spin 2 s and adjust) ----
ROBOT_SPEED_MPS = 0.20                 # metres per second at motor speed 150
TURN_RATE_DPS = 90.0                   # degrees per second while turning in place

# ---- Camera geometry for estimating where the garbage is ----
CAMERA_HFOV_DEG = 62.0                 # horizontal field of view of the OV2640
HORIZON_FRAC = 0.40                    # picture height fraction where the floor ends (far)
NEAR_DIST_M = 0.30                     # distance of an object at the bottom of the picture
FAR_DIST_M = 2.50                      # distance of an object at the horizon line

# ---- Logging ----
LOG_DIR = str(BASE_DIR / "detections")
CSV_PATH = os.path.join(LOG_DIR, "detections.csv")
SNAP_DIR = os.path.join(LOG_DIR, "snapshots")
MAP_PNG = os.path.join(LOG_DIR, "map_latest.png")
LOGGING_ON = True                      # toggle at runtime with the L key
DEDUP_RADIUS_M = 0.6                   # same class within this distance = same garbage
LOAD_PREVIOUS_ON_START = False         # True: redraw earlier detections on the map
MAP_MIN_HALF_M = 3.0                   # smallest map half-width in metres

# ---- Size estimate ----
SIZE_SMALL_MAX_PCT = 2.0               # % of frame covered by the mask
SIZE_MEDIUM_MAX_PCT = 10.0
PIXELS_PER_CM = None                   # optional calibration -> area_cm2

KEY_HOLD_SECONDS = 0.6                 # only for the fallback keyboard mode (no pynput)
CONTROL_KEYS = {"w", "a", "s", "d", "x", "space"}
# ========================================================================

M_PER_DEG_LAT = 111320.0

running = True
lock = threading.Lock()

latest_frame = None
latest_frame_id = 0
annotated = None
video_ok = False

gps_raw = "GPS: waiting for data"
gps_info = {"status": "UNAVAILABLE", "lat": None, "lng": None, "sats": 0}
fps_value = 0.0
logged_total = 0
last_logged_text = "No detections logged yet"

origin_lat = ORIGIN_LAT
origin_lng = ORIGIN_LNG
origin_from_gps = False

# Robot pose in metres from the origin. Heading: degrees clockwise from north.
# There is no compass, so heading comes from the turn commands only.
pose = {"x": 0.0, "y": 0.0, "heading": 0.0}
trail = []
garbage_points = []      # dicts: x, y, cls, lat, lng
class_colors = {}

pressed = set()
last_seen = {}

CSV_FIELDS = [
    "timestamp", "robot_id", "garbage_type", "confidence",
    "latitude", "longitude", "satellites", "gps_source",
    "garbage_lat", "garbage_lng", "garbage_x_m", "garbage_y_m",
    "distance_m", "bearing_deg",
    "area_px", "area_percent", "size_class", "area_cm2",
    "bbox_w_px", "bbox_h_px", "snapshot",
]

PALETTE = [(60, 76, 231), (113, 204, 46), (219, 152, 52), (15, 196, 241),
           (182, 89, 155), (156, 188, 26), (34, 126, 230), (166, 166, 127),
           (94, 73, 200), (180, 119, 31)]


# ----------------------------- geo helpers -----------------------------
def latlng_to_xy(lat, lng):
    x = (lng - origin_lng) * M_PER_DEG_LAT * math.cos(math.radians(origin_lat))
    y = (lat - origin_lat) * M_PER_DEG_LAT
    return x, y


def xy_to_latlng(x, y):
    lat = origin_lat + y / M_PER_DEG_LAT
    lng = origin_lng + x / (M_PER_DEG_LAT * math.cos(math.radians(origin_lat)))
    return lat, lng


def add_trail_point_locked():
    if not trail or math.hypot(pose["x"] - trail[-1][0],
                               pose["y"] - trail[-1][1]) > 0.05:
        trail.append((pose["x"], pose["y"]))
        if len(trail) > 3000:
            del trail[:500]


def recompute_points_locked():
    for p in garbage_points:
        p["x"], p["y"] = latlng_to_xy(p["lat"], p["lng"])


def set_origin_locked(lat, lng, from_gps):
    """Make (lat, lng) the new origin and re-place everything around it."""
    global origin_lat, origin_lng, origin_from_gps
    origin_lat, origin_lng = lat, lng
    origin_from_gps = from_gps
    pose["x"], pose["y"] = 0.0, 0.0
    trail.clear()
    trail.append((0.0, 0.0))
    recompute_points_locked()


def set_origin_here():
    with lock:
        if gps_info["status"] == "FIX":
            set_origin_locked(gps_info["lat"], gps_info["lng"], True)
            print("Origin set to current GPS position.")
        else:
            print("Cannot set origin: no GPS fix.")


# ----------------------------- GPS parsing -----------------------------
def parse_gps(text):
    """Convert the ESP32 /gps text into a dictionary.

    GPS,25.495572,81.867973,5      -> FIX
    GPS,NO_FIX,SAT=0               -> NO_FIX
    GPS,NO_DATA,CHECK_WIRING       -> NO_DATA
    anything else                  -> UNAVAILABLE
    """
    info = {"status": "UNAVAILABLE", "lat": None, "lng": None, "sats": 0}

    if not text.startswith("GPS,"):
        return info

    parts = text.split(",")

    if len(parts) >= 2 and parts[1] == "NO_FIX":
        info["status"] = "NO_FIX"
        if len(parts) >= 3 and parts[2].startswith("SAT="):
            try:
                info["sats"] = int(parts[2][4:])
            except ValueError:
                pass
        return info

    if len(parts) >= 2 and parts[1] == "NO_DATA":
        info["status"] = "NO_DATA"
        return info

    try:
        info["lat"] = float(parts[1])
        info["lng"] = float(parts[2])
        info["sats"] = int(parts[3]) if len(parts) > 3 else 0
        info["status"] = "FIX"
    except (ValueError, IndexError):
        pass

    return info


def gps_display_text(info):
    if info["status"] == "FIX":
        return "GPS FIX {:.6f}, {:.6f} | SAT {}".format(
            info["lat"], info["lng"], info["sats"])
    if info["status"] == "NO_FIX":
        return "GPS searching - using ORIGIN as position"
    if info["status"] == "NO_DATA":
        return "GPS module silent (check wiring) - using ORIGIN"
    return "GPS link down - using ORIGIN"


# ------------------------- window / focus helper -------------------------
def window_focused():
    try:
        hwnd = ctypes.windll.user32.GetForegroundWindow()
        buf = ctypes.create_unicode_buffer(256)
        ctypes.windll.user32.GetWindowTextW(hwnd, buf, 256)
        return buf.value == WINDOW
    except Exception:
        return True


# --------------------------- detection records ---------------------------
def size_class_from_percent(pct):
    if pct < SIZE_SMALL_MAX_PCT:
        return "Small"
    if pct < SIZE_MEDIUM_MAX_PCT:
        return "Medium"
    return "Large"


def ensure_log_files():
    os.makedirs(SNAP_DIR, exist_ok=True)

    if os.path.exists(CSV_PATH):
        with open(CSV_PATH, "r", newline="", encoding="utf-8") as f:
            header = next(csv.reader(f), [])
        if header != CSV_FIELDS:
            backup = os.path.join(
                LOG_DIR, "detections_old_{}.csv".format(
                    datetime.now().strftime("%Y%m%d_%H%M%S")))
            os.replace(CSV_PATH, backup)
            print("Old CSV had a different format, moved to", backup)

    if not os.path.exists(CSV_PATH):
        with open(CSV_PATH, "w", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow(CSV_FIELDS)


def load_previous_points():
    try:
        with open(CSV_PATH, "r", newline="", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                try:
                    lat = float(row["garbage_lat"])
                    lng = float(row["garbage_lng"])
                except (ValueError, KeyError):
                    continue
                x, y = latlng_to_xy(lat, lng)
                garbage_points.append(
                    {"x": x, "y": y, "cls": row["garbage_type"],
                     "lat": lat, "lng": lng})
        print("Loaded {} earlier detections.".format(len(garbage_points)))
    except OSError:
        pass


def extract_records(result, frame):
    """One record per detected object, with mask-based size."""
    records = []
    boxes = result.boxes

    if boxes is None or len(boxes) == 0:
        return records

    h, w = frame.shape[:2]
    frame_area = float(w * h)
    names = result.names
    masks = result.masks

    for i in range(len(boxes)):
        conf = float(boxes.conf[i])
        if conf < CONF_MIN:
            continue

        name = names[int(boxes.cls[i])]
        if CLASS_FILTER is not None and name not in CLASS_FILTER:
            continue

        x1, y1, x2, y2 = [float(v) for v in boxes.xyxy[i]]

        area_px = max(1.0, (x2 - x1) * (y2 - y1))
        if masks is not None and i < len(masks.xy):
            poly = masks.xy[i]
            if len(poly) >= 3:
                area_px = float(cv2.contourArea(poly.astype(np.float32)))

        area_pct = 100.0 * area_px / frame_area

        records.append({
            "garbage_type": name,
            "confidence": conf,
            "x1": x1, "y1": y1, "x2": x2, "y2": y2,
            "frame_w": w, "frame_h": h,
            "area_px": area_px,
            "area_pct": area_pct,
            "size_class": size_class_from_percent(area_pct),
            "area_cm2": (area_px / (PIXELS_PER_CM ** 2)) if PIXELS_PER_CM else None,
        })

    return records


def locate_object(rec, robot_pose):
    """Estimate where the garbage lies (metres from origin) using the camera view.

    Bearing comes from how far left/right the object is in the picture.
    Distance comes from how low it sits in the picture (flat-floor assumption).
    These are rough estimates, good for a relative map.
    """
    w, h = rec["frame_w"], rec["frame_h"]
    cx = (rec["x1"] + rec["x2"]) / 2.0

    offset = (cx / w - 0.5) * CAMERA_HFOV_DEG
    t = (rec["y2"] / h - HORIZON_FRAC) / (1.0 - HORIZON_FRAC)
    t = min(max(t, 0.0), 1.0)
    dist = FAR_DIST_M - t * (FAR_DIST_M - NEAR_DIST_M)

    bearing = (robot_pose["heading"] + offset) % 360.0
    b = math.radians(bearing)
    gx = robot_pose["x"] + dist * math.sin(b)
    gy = robot_pose["y"] + dist * math.cos(b)
    return gx, gy, dist, bearing


def log_detection(frame_annotated, rec):
    """Save the snapshot, append the CSV row and add the point to the map."""
    global logged_total, last_logged_text

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    safe = rec["garbage_type"].replace(" ", "_").replace("/", "-")
    snap_name = "{}_{}.jpg".format(stamp, safe)
    cv2.imwrite(os.path.join(SNAP_DIR, snap_name), frame_annotated)

    row = [
        datetime.now().isoformat(timespec="seconds"),
        ROBOT_ID,
        rec["garbage_type"],
        "{:.2f}".format(rec["confidence"]),
        "{:.6f}".format(rec["robot_lat"]),
        "{:.6f}".format(rec["robot_lng"]),
        rec["sats"],
        rec["gps_source"],
        "{:.6f}".format(rec["g_lat"]),
        "{:.6f}".format(rec["g_lng"]),
        "{:.2f}".format(rec["gx"]),
        "{:.2f}".format(rec["gy"]),
        "{:.2f}".format(rec["dist"]),
        "{:.0f}".format(rec["bearing"]),
        int(rec["area_px"]),
        "{:.2f}".format(rec["area_pct"]),
        rec["size_class"],
        "" if rec["area_cm2"] is None else "{:.1f}".format(rec["area_cm2"]),
        int(rec["x2"] - rec["x1"]),
        int(rec["y2"] - rec["y1"]),
        snap_name,
    ]

    with open(CSV_PATH, "a", newline="", encoding="utf-8") as f:
        csv.writer(f).writerow(row)

    with lock:
        garbage_points.append({"x": rec["gx"], "y": rec["gy"],
                               "cls": rec["garbage_type"],
                               "lat": rec["g_lat"], "lng": rec["g_lng"]})
        logged_total += 1
        last_logged_text = "Logged: {} {:.0f}% ({})".format(
            rec["garbage_type"], rec["confidence"] * 100, rec["gps_source"])


# ----------------------------- video grabber -----------------------------
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


# --------------------- YOLO inference + GPS-tagged logging ---------------------
def inference_loop(model):
    global annotated, fps_value
    last_id = 0
    last_time = time.monotonic()

    while running:
        with lock:
            frame = latest_frame
            fid = latest_frame_id

        if frame is None or fid == last_id:
            time.sleep(0.01)
            continue

        last_id = fid

        try:
            result = model.predict(frame, imgsz=INFER_IMGSZ, verbose=False)[0]
            out = result.plot()
            records = extract_records(result, frame)
        except Exception as error:
            print("Segmentation error:", error)
            out = frame
            records = []

        now = time.monotonic()
        dt = now - last_time
        last_time = now
        if dt > 0:
            fps_value = 0.8 * fps_value + 0.2 * (1.0 / dt)

        with lock:
            annotated = out
            logging_on = LOGGING_ON
            robot_pose = dict(pose)
            info = dict(gps_info)
            known = [(p["x"], p["y"], p["cls"]) for p in garbage_points]

        if not (logging_on and records):
            continue

        # Robot position tag: real GPS if available, otherwise the origin
        # (plus estimated movement if DEAD_RECKON_WITHOUT_GPS is True).
        if info["status"] == "FIX":
            robot_lat, robot_lng, source = info["lat"], info["lng"], "FIX"
        else:
            robot_lat, robot_lng = xy_to_latlng(robot_pose["x"], robot_pose["y"])
            source = "ORIGIN"

        for rec in records:
            gx, gy, dist, bearing = locate_object(rec, robot_pose)

            duplicate = any(
                cls == rec["garbage_type"] and
                math.hypot(gx - kx, gy - ky) < DEDUP_RADIUS_M
                for kx, ky, cls in known)
            if duplicate:
                continue

            g_lat, g_lng = xy_to_latlng(gx, gy)
            rec.update({
                "robot_lat": robot_lat, "robot_lng": robot_lng,
                "sats": info["sats"], "gps_source": source,
                "gx": gx, "gy": gy, "dist": dist, "bearing": bearing,
                "g_lat": g_lat, "g_lng": g_lng,
            })
            known.append((gx, gy, rec["garbage_type"]))

            try:
                log_detection(out, rec)
            except Exception as error:
                print("Logging error:", error)


# ------------------------------ robot control ------------------------------
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


def integrate_motion(command, dt):
    """Dead reckoning: update heading and position from the commands sent."""
    if command not in ("F", "B", "L", "R"):
        return

    with lock:
        if command == "L":
            pose["heading"] = (pose["heading"] - TURN_RATE_DPS * dt) % 360.0
        elif command == "R":
            pose["heading"] = (pose["heading"] + TURN_RATE_DPS * dt) % 360.0
        elif gps_info["status"] != "FIX" and DEAD_RECKON_WITHOUT_GPS:
            sign = 1.0 if command == "F" else -1.0
            d = sign * ROBOT_SPEED_MPS * dt
            h = math.radians(pose["heading"])
            pose["x"] += d * math.sin(h)
            pose["y"] += d * math.cos(h)
            add_trail_point_locked()


def command_loop():
    session = requests.Session()
    previous = "S"
    last_t = time.monotonic()

    while running:
        command = current_command()
        sent = False

        try:
            session.get(ESP32 + "/cmd", params={"c": command}, timeout=0.4)
            sent = True
        except requests.RequestException:
            pass

        now = time.monotonic()
        dt = min(now - last_t, 0.5)
        last_t = now

        if sent:
            integrate_motion(previous, dt)
            previous = command

        time.sleep(0.15)

    session.close()


def gps_loop():
    global gps_raw, gps_info
    session = requests.Session()

    while running:
        try:
            response = session.get(ESP32 + "/gps", timeout=1.0)
            text = response.text.strip()
        except requests.RequestException:
            text = "GPS: ESP32 unavailable"

        info = parse_gps(text)

        with lock:
            gps_raw = text
            gps_info = info

            if info["status"] == "FIX":
                if not origin_from_gps:
                    # First real fix of this session becomes the origin.
                    set_origin_locked(info["lat"], info["lng"], True)
                    print("GPS fix acquired: origin set to", info["lat"], info["lng"])
                else:
                    pose["x"], pose["y"] = latlng_to_xy(info["lat"], info["lng"])
                    add_trail_point_locked()

        time.sleep(1)

    session.close()


def stop_robot():
    for _ in range(3):
        try:
            requests.get(ESP32 + "/cmd", params={"c": "S"}, timeout=0.4)
        except requests.RequestException:
            pass
        time.sleep(0.05)


# --------------------------------- keyboard ---------------------------------
def toggle_logging():
    global LOGGING_ON
    with lock:
        LOGGING_ON = not LOGGING_ON
        state = LOGGING_ON
    print("Logging", "ON" if state else "OFF")


def save_map_png():
    os.makedirs(LOG_DIR, exist_ok=True)
    cv2.imwrite(MAP_PNG, draw_map(900, 900))
    print("Map saved to", os.path.abspath(MAP_PNG))


def key_name(key):
    try:
        if key.char:
            return key.char.lower()
    except AttributeError:
        pass
    if key == pynput_keyboard.Key.space:
        return "space"
    return None


def handle_command_key(name):
    if name == "l":
        toggle_logging()
    elif name == "o":
        set_origin_here()
    elif name == "m":
        save_map_png()


def on_press(key):
    name = key_name(key)
    if name in CONTROL_KEYS:
        with lock:
            pressed.add(name)
    elif name in ("l", "o", "m") and window_focused():
        handle_command_key(name)


def on_release(key):
    name = key_name(key)
    if name in CONTROL_KEYS:
        with lock:
            pressed.discard(name)


# ------------------------------- map drawing -------------------------------
def class_color(name):
    if name not in class_colors:
        class_colors[name] = PALETTE[len(class_colors) % len(PALETTE)]
    return class_colors[name]


def nice_step(raw):
    for s in (0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000):
        if s >= raw:
            return s
    return 10000


def draw_map(w, h):
    """Top-down map: origin, robot trail, robot, and every logged garbage."""
    img = np.full((h, w, 3), (32, 32, 32), np.uint8)

    with lock:
        rp = dict(pose)
        pts = [dict(p) for p in garbage_points]
        tr = list(trail)
        info = dict(gps_info)
        lat0, lng0 = origin_lat, origin_lng

    top, bottom = 38, 112
    size = min(w - 20, h - top - bottom)
    pcx = w // 2
    pcy = top + size // 2

    xs = [0.0, rp["x"]] + [p["x"] for p in pts] + [t[0] for t in tr]
    ys = [0.0, rp["y"]] + [p["y"] for p in pts] + [t[1] for t in tr]
    cx = (min(xs) + max(xs)) / 2.0
    cy = (min(ys) + max(ys)) / 2.0
    half = max(max(xs) - min(xs), max(ys) - min(ys)) / 2.0 * 1.3
    half = max(half, MAP_MIN_HALF_M)
    scale = (size / 2.0) / half

    def to_px(x, y):
        return int(pcx + (x - cx) * scale), int(pcy - (y - cy) * scale)

    left_px, right_px = pcx - size // 2, pcx + size // 2
    top_px, bot_px = pcy - size // 2, pcy + size // 2

    cv2.rectangle(img, (left_px, top_px), (right_px, bot_px), (45, 45, 45), -1)

    step = nice_step(half * 2.0 / 5.0)
    gx = math.floor((cx - half) / step) * step
    while gx <= cx + half:
        px, _ = to_px(gx, cy)
        if left_px <= px <= right_px:
            cv2.line(img, (px, top_px), (px, bot_px), (70, 70, 70), 1)
        gx += step
    gy = math.floor((cy - half) / step) * step
    while gy <= cy + half:
        _, py = to_px(cx, gy)
        if top_px <= py <= bot_px:
            cv2.line(img, (left_px, py), (right_px, py), (70, 70, 70), 1)
        gy += step
    cv2.rectangle(img, (left_px, top_px), (right_px, bot_px), (120, 120, 120), 1)

    # Origin
    ox, oy = to_px(0.0, 0.0)
    cv2.line(img, (ox - 8, oy), (ox + 8, oy), (255, 255, 255), 1)
    cv2.line(img, (ox, oy - 8), (ox, oy + 8), (255, 255, 255), 1)
    cv2.putText(img, "ORIGIN", (ox + 6, oy - 6), cv2.FONT_HERSHEY_SIMPLEX,
                0.4, (255, 255, 255), 1, cv2.LINE_AA)

    # Robot trail
    if len(tr) >= 2:
        poly = np.array([to_px(x, y) for x, y in tr], np.int32)
        cv2.polylines(img, [poly], False, (160, 160, 90), 1, cv2.LINE_AA)

    # Garbage
    for p in pts:
        px, py = to_px(p["x"], p["y"])
        col = class_color(p["cls"])
        cv2.circle(img, (px, py), 7, col, -1)
        cv2.circle(img, (px, py), 7, (255, 255, 255), 1)
        cv2.putText(img, p["cls"][:12], (px + 9, py + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (230, 230, 230), 1, cv2.LINE_AA)

    # Robot (triangle pointing along its heading)
    rx, ry = to_px(rp["x"], rp["y"])
    hd = math.radians(rp["heading"])
    fx, fy = math.sin(hd), -math.cos(hd)
    sx, sy = -fy, fx
    tip = (int(rx + fx * 13), int(ry + fy * 13))
    b1 = (int(rx - fx * 8 + sx * 7), int(ry - fy * 8 + sy * 7))
    b2 = (int(rx - fx * 8 - sx * 7), int(ry - fy * 8 - sy * 7))
    cv2.fillPoly(img, [np.array([tip, b1, b2], np.int32)], (0, 220, 0))
    cv2.line(img, (rx, ry), (int(rx + fx * 30), int(ry + fy * 30)), (0, 220, 0), 1)

    # North arrow
    nx, ny = right_px - 18, top_px + 30
    cv2.arrowedLine(img, (nx, ny), (nx, ny - 22), (255, 255, 255), 2, tipLength=0.4)
    cv2.putText(img, "N", (nx - 5, ny + 14), cv2.FONT_HERSHEY_SIMPLEX,
                0.45, (255, 255, 255), 1, cv2.LINE_AA)

    # Title
    cv2.putText(img, "GARBAGE MAP (relative to origin)", (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

    # Footer text
    src = "GPS FIX" if info["status"] == "FIX" else "ORIGIN (no GPS)"
    y0 = bot_px + 20
    cv2.putText(img, "Grid {} m | Robot x={:.1f} y={:.1f} m | heading {:.0f} deg".format(
        step, rp["x"], rp["y"], rp["heading"]),
        (10, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1, cv2.LINE_AA)
    cv2.putText(img, "Position source: {} | Origin {:.5f}, {:.5f}".format(src, lat0, lng0),
                (10, y0 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1,
                cv2.LINE_AA)

    # Legend with counts
    counts = Counter(p["cls"] for p in pts)
    legend = counts.most_common(6)
    for i, (name, n) in enumerate(legend):
        col = class_color(name)
        lx = 10 + (i % 2) * (w // 2)
        ly = y0 + 40 + (i // 2) * 18
        cv2.circle(img, (lx + 5, ly - 4), 5, col, -1)
        cv2.putText(img, "{} {}".format(name[:16], n), (lx + 16, ly),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (230, 230, 230), 1, cv2.LINE_AA)
    extra = len(counts) - len(legend)
    if extra > 0:
        cv2.putText(img, "+{} more".format(extra), (w - 80, y0 + 40 + 2 * 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1, cv2.LINE_AA)

    return img


def draw_info_bar(display, lines):
    line_h = 24
    bar_h = 12 + line_h * len(lines)
    overlay = display.copy()
    cv2.rectangle(overlay, (0, 0), (display.shape[1], bar_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, display, 0.4, 0, display)

    for i, (text, color) in enumerate(lines):
        cv2.putText(display, text, (10, 22 + i * line_h),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, color, 1, cv2.LINE_AA)


# ----------------------------------- main -----------------------------------
def main():
    global running, CLASS_FILTER

    ensure_log_files()

    model_path = MODEL_NAME
    if not os.path.exists(model_path):
        model_path = FALLBACK_MODEL
        CLASS_FILTER = FALLBACK_COCO_FILTER
        print("WARNING: {} not found. Using the generic COCO model {}.".format(
            MODEL_NAME, FALLBACK_MODEL))
        print("Train the TACO model with python ai-detection/train_taco.py and copy "
              "garbage-seg.pt under ai-detection/models/.")

    print("Loading model", model_path, "...")
    model = YOLO(model_path)

    if LOAD_PREVIOUS_ON_START:
        with lock:
            load_previous_points()

    with lock:
        trail.append((0.0, 0.0))

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

    cam_h = int(CAM_DISPLAY_WIDTH * 0.75)
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, CAM_DISPLAY_WIDTH + MAP_PANEL_WIDTH, cam_h)

    print("Controls: W=forward, S=back, A=left, D=right")
    print("Space/X=stop, L=logging on/off, O=set origin to current GPS,")
    print("M=save map picture, ESC=exit")
    print("Detections are saved in:", os.path.abspath(LOG_DIR))
    print("Keep the video window focused to control the robot.")

    white = (255, 255, 255)
    green = (0, 255, 0)
    yellow = (0, 255, 255)
    red = (0, 0, 255)

    try:
        while running:
            with lock:
                frame = annotated if annotated is not None else latest_frame
                total = logged_total
                last_text = last_logged_text
                logging_on = LOGGING_ON
                info = dict(gps_info)

            if frame is None:
                cam = np.zeros((240, 320, 3), dtype=np.uint8)
                cv2.putText(cam, "Waiting for video...", (20, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, white, 2)
            else:
                cam = frame.copy()

            scale = CAM_DISPLAY_WIDTH / float(cam.shape[1])
            cam = cv2.resize(cam, (CAM_DISPLAY_WIDTH, int(cam.shape[0] * scale)),
                             interpolation=cv2.INTER_CUBIC)

            lines = [
                ("WASD move | SPACE/X stop | L log | O origin | M save map | ESC exit",
                 white),
                (gps_display_text(info), green if info["status"] == "FIX" else yellow),
                ("VIDEO OK" if video_ok else "VIDEO LOST - robot stopped",
                 green if video_ok else red),
                ("Logging {} | saved {} | {:.1f} FPS".format(
                    "ON" if logging_on else "OFF", total, fps_value), white),
                (last_text, yellow),
            ]
            draw_info_bar(cam, lines)

            map_panel = draw_map(MAP_PANEL_WIDTH, cam.shape[0])
            display = np.hstack([cam, map_panel])

            cv2.imshow(WINDOW, display)
            key = cv2.waitKey(15)

            if key != -1:
                key &= 0xFF
                if key == 27:
                    break
                if pynput_keyboard is None:
                    name = "space" if key == 32 else chr(key).lower()
                    if name in ("l", "o", "m"):
                        handle_command_key(name)
                    elif name in CONTROL_KEYS:
                        with lock:
                            last_seen[name] = time.monotonic()

            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break

    finally:
        running = False
        if listener is not None:
            listener.stop()
        stop_robot()
        try:
            save_map_png()
        except Exception as error:
            print("Could not save map:", error)
        time.sleep(0.2)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
