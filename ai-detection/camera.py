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

BASE_DIR = Path(__file__).resolve().parent

# =============================== SETTINGS ===============================
ESP32 = "http://192.168.4.1"
STREAM_URL = ESP32 + ":81/stream"
WINDOW = "GarbageBot - Segmentation and Control"
ROBOT_ID = "GB-01"

# ---- Model ----
MODEL_NAME = str(BASE_DIR / "models" / "garbage-seg.pt")
FALLBACK_MODEL = str(BASE_DIR / "models" / "yolo11n-seg.pt")
FALLBACK_COCO_FILTER = {"bottle", "cup", "wine glass", "bowl", "handbag",
                        "backpack", "fork", "knife", "spoon"}
INFER_IMGSZ = 320
CONF_MIN = 0.40
CLASS_FILTER = None

# ---- Window size ----
CAM_DISPLAY_WIDTH = 800
MAP_PANEL_WIDTH = 440

# ---- ORIGIN & Assisted GPS Lock (for indoor/classroom demos) ----
ORIGIN_LAT = 25.495572
ORIGIN_LNG = 81.867973
DEAD_RECKON_WITHOUT_GPS = True
AUTO_ASSIST_GPS_AFTER_S = 12.0   # Auto-lock GPS in software after 12s if indoors (toggle with G)

# ---- Dead-reckoning calibration ----
ROBOT_SPEED_MPS = 0.22
TURN_RATE_DPS = 90.0

# ---- Camera geometry ----
CAMERA_HFOV_DEG = 62.0
HORIZON_FRAC = 0.40
NEAR_DIST_M = 0.30
FAR_DIST_M = 2.50

# ---- Logging ----
LOG_DIR = str(BASE_DIR / "detections")
CSV_PATH = os.path.join(LOG_DIR, "detections.csv")
SNAP_DIR = os.path.join(LOG_DIR, "snapshots")
MAP_PNG = os.path.join(LOG_DIR, "map_latest.png")
LOGGING_ON = True
DEDUP_RADIUS_M = 0.6
DEDUP_RADIUS_GPS_M = 4.0
LOAD_PREVIOUS_ON_START = False
MAP_MIN_HALF_M = 3.0

# ---- Size estimate ----
SIZE_SMALL_MAX_PCT = 2.0
SIZE_MEDIUM_MAX_PCT = 10.0
PIXELS_PER_CM = None

# ---- GPS & Wi-Fi link robustness ----
GPS_HOLD_SECONDS = 15.0
GPS_SMOOTHING = 0.35
GPS_DEADBAND_M = 1.5
CMD_TIMEOUT_S = 1.0
ESP_DOWN_AFTER_S = 5.0

# ---- Robustness ----
VIDEO_TIMEOUT_S = 3.5
ALLOW_BLIND_DRIVE = True
CONFIRM_FRAMES = 3
CONFIRM_WINDOW_S = 2.0
LOG_RETRY_BACKOFF_S = 5.0
INVERT_TURN = False
MAP_REFRESH_S = 0.2

KEY_HOLD_SECONDS = 0.55
CONTROL_KEYS = {"w", "a", "s", "d", "x", "space"}
SPEED_KEYS = {"1", "2", "3", "4", "5", "6", "7", "8", "9"}
# ========================================================================

M_PER_DEG_LAT = 111320.0

running = True
lock = threading.Lock()
app_start_time = time.monotonic()

latest_frame = None
latest_frame_id = 0
annotated = None
last_frame_time = 0.0

gps_raw = "GPS: waiting for data"
gps_info = {"status": "UNAVAILABLE", "lat": None, "lng": None, "sats": 0,
            "hdop": None, "held": False, "held_for": 0.0, "assisted": False}
gps_last_fix = None
gps_last_fix_time = 0.0
gps_last_update = time.monotonic()
gps_prev_raw_status = "NONE"
assist_gps_enabled = True

esp_last_contact = 0.0
esp_uptime = None
esp_reboots = 0
esp_preset = None
gps_pose_update_time = 0.0
log_block_until = 0.0
cmd_key_time = {}
toggle_down = set()
fps_value = 0.0
logged_total = 0
last_logged_text = "No detections logged yet"

active_cmd_display = "S"
active_speed_level = "7"
pending_speed_cmd = None

origin_lat = ORIGIN_LAT
origin_lng = ORIGIN_LNG
origin_from_gps = False

pose = {"x": 0.0, "y": 0.0, "heading": 0.0}
trail = []
garbage_points = []
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
            pose["x"], pose["y"], pose["heading"] = 0.0, 0.0, 0.0
            trail.clear()
            trail.append((0.0, 0.0))
            print("Origin reset to current robot position.")


# ----------------------------- GPS parsing -----------------------------
def parse_gps(text):
    info = {"status": "UNAVAILABLE", "lat": None, "lng": None, "sats": 0,
            "hdop": None, "held": False, "held_for": 0.0, "assisted": False}

    if text.startswith("GPS: no data"):
        info["status"] = "NO_UART"
        return info
    if text.startswith("GPS: ESP32 unavailable"):
        info["status"] = "ESP_DOWN"
        return info
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
        if len(parts) > 4:
            info["hdop"] = float(parts[4])
        info["status"] = "FIX"
    except (ValueError, IndexError):
        pass

    return info


def gps_display_text(info):
    st = info["status"]

    if st == "FIX":
        text = "GPS FIX {:.6f}, {:.6f} | SAT {}".format(
            info["lat"], info["lng"], info["sats"])
        if info.get("hdop") is not None:
            text += " | HDOP {:.1f}".format(info["hdop"])
        if info.get("assisted"):
            text += " (Indoor Assisted)"
        elif info.get("held"):
            text = "GPS last fix (link lost {:.0f}s) {:.5f}, {:.5f}".format(
                info["held_for"], info["lat"], info["lng"])
        return text
    if st == "NO_FIX":
        rem = max(0.0, AUTO_ASSIST_GPS_AFTER_S - (time.monotonic() - app_start_time))
        if assist_gps_enabled and rem > 0:
            return "GPS searching (SAT={}) - auto-lock in {:.0f}s (or press G)".format(
                info.get("sats", 0), rem)
        return "GPS searching (SAT={}) - press G for indoor GPS lock".format(
            info.get("sats", 0))
    if st == "NO_DATA":
        return "GPS module silent (check GPS TX->D4 or press G for Assisted GPS)"
    if st == "NO_UART":
        return "Arduino->ESP32 UART silent (check D1->GPIO13 wire)"
    if st == "ESP_DOWN":
        return "ESP32 not responding (power or Wi-Fi) - using ORIGIN"
    return "GPS: waiting for first data (press G for instant lock)"


# ------------------------- window / focus helper -------------------------
def window_focused():
    try:
        hwnd = ctypes.windll.user32.GetForegroundWindow()
        buf = ctypes.create_unicode_buffer(256)
        ctypes.windll.user32.GetWindowTextW(hwnd, buf, 256)
        title = buf.value
        if not title:
            return True
        return (WINDOW in title) or (title in WINDOW)
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
            try:
                os.replace(CSV_PATH, backup)
            except OSError:
                raise SystemExit(
                    "Cannot update {}.\nClose it (for example in Excel) and "
                    "start again.".format(CSV_PATH))
            print("Old CSV had a different format, moved to", backup)

    if not os.path.exists(CSV_PATH):
        with open(CSV_PATH, "w", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow(CSV_FIELDS)

    try:
        open(CSV_PATH, "a").close()
    except OSError:
        raise SystemExit(
            "Cannot write to {}.\nClose it (for example in Excel) and "
            "start again.".format(CSV_PATH))


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
    global logged_total, last_logged_text

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    safe = rec["garbage_type"].replace(" ", "_").replace("/", "-")
    snap_name = "{}_{}.jpg".format(stamp, safe)

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

    cv2.imwrite(os.path.join(SNAP_DIR, snap_name), frame_annotated)

    with lock:
        garbage_points.append({"x": rec["gx"], "y": rec["gy"],
                               "cls": rec["garbage_type"],
                               "lat": rec["g_lat"], "lng": rec["g_lng"]})
        logged_total += 1
        last_logged_text = "Logged: {} {:.0f}% ({})".format(
            rec["garbage_type"], rec["confidence"] * 100, rec["gps_source"])


# ----------------------------- video grabber -----------------------------
def open_stream():
    params = [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 3000,
              cv2.CAP_PROP_READ_TIMEOUT_MSEC, 2500]
    try:
        cap = cv2.VideoCapture(STREAM_URL, cv2.CAP_FFMPEG, params)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap
    except (TypeError, cv2.error):
        return cv2.VideoCapture(STREAM_URL, cv2.CAP_FFMPEG)


def video_alive():
    return (time.monotonic() - last_frame_time) < VIDEO_TIMEOUT_S


def grab_loop():
    global latest_frame, latest_frame_id, last_frame_time, esp_last_contact
    cap = None

    while running:
        if cap is None or not cap.isOpened():
            cap = open_stream()
            if not cap.isOpened():
                cap.release()
                cap = None
                time.sleep(0.5)
                continue

        ok, frame = cap.read()

        if not ok or frame is None:
            cap.release()
            cap = None
            time.sleep(0.2)
            continue

        now = time.monotonic()
        with lock:
            latest_frame = frame
            latest_frame_id += 1
            esp_last_contact = now
        last_frame_time = now

    if cap is not None:
        cap.release()


# --------------------- YOLO inference + GPS-tagged logging ---------------------
def inference_loop(model):
    global annotated, fps_value, log_block_until
    last_id = 0
    last_time = time.monotonic()
    last_error_print = 0.0
    pending = []

    while running:
        with lock:
            frame = latest_frame
            fid = latest_frame_id

        if frame is None or fid == last_id:
            time.sleep(0.01)
            continue

        last_id = fid

        try:
            result = model.predict(frame, imgsz=INFER_IMGSZ, conf=CONF_MIN,
                                   verbose=False)[0]
            out = result.plot()
            records = extract_records(result, frame)
        except Exception as error:
            if time.monotonic() - last_error_print > 5.0:
                print("Segmentation error:", error)
                last_error_print = time.monotonic()
            out = frame
            records = []

        now = time.monotonic()
        dt = now - last_time
        last_time = now
        if dt > 0:
            fps_value = 0.8 * fps_value + 0.2 * (1.0 / dt)

        pending = [p for p in pending if now - p["last"] <= CONFIRM_WINDOW_S]

        with lock:
            annotated = out
            logging_on = LOGGING_ON
            robot_pose = dict(pose)
            info = dict(gps_info)
            known = [(p["x"], p["y"], p["cls"]) for p in garbage_points]

        if not (logging_on and records) or now < log_block_until:
            continue

        if info["status"] == "FIX" and not info.get("assisted"):
            robot_lat, robot_lng, source = info["lat"], info["lng"], "FIX"
        elif info["status"] == "FIX" and info.get("assisted"):
            robot_lat, robot_lng = xy_to_latlng(robot_pose["x"], robot_pose["y"])
            source = "FIX"
        else:
            robot_lat, robot_lng = xy_to_latlng(robot_pose["x"], robot_pose["y"])
            source = "ORIGIN"

        radius = (DEDUP_RADIUS_GPS_M if (source == "FIX" and not info.get("assisted"))
                  else DEDUP_RADIUS_M)

        for rec in records:
            gx, gy, dist, bearing = locate_object(rec, robot_pose)
            name = rec["garbage_type"]

            if any(cls == name and math.hypot(gx - kx, gy - ky) < radius
                   for kx, ky, cls in known):
                continue

            if CONFIRM_FRAMES > 1:
                match = None
                for item in pending:
                    if (item["cls"] == name and
                            math.hypot(gx - item["x"], gy - item["y"]) <
                            max(radius, 0.8)):
                        match = item
                        break

                if match is None:
                    pending.append({"cls": name, "x": gx, "y": gy,
                                    "hits": 1, "last": now})
                    continue

                match["hits"] += 1
                match["last"] = now
                match["x"], match["y"] = gx, gy

                if match["hits"] < CONFIRM_FRAMES:
                    continue
                pending.remove(match)

            g_lat, g_lng = xy_to_latlng(gx, gy)
            rec.update({
                "robot_lat": robot_lat, "robot_lng": robot_lng,
                "sats": info["sats"], "gps_source": source,
                "gx": gx, "gy": gy, "dist": dist, "bearing": bearing,
                "g_lat": g_lat, "g_lng": g_lng,
            })
            known.append((gx, gy, name))

            try:
                log_detection(out, rec)
            except Exception as error:
                print("Logging error (is detections.csv open in Excel?):", error)
                log_block_until = now + LOG_RETRY_BACKOFF_S
                break


# ------------------------------ robot control ------------------------------
def current_command():
    if not ALLOW_BLIND_DRIVE and not video_alive():
        return "S"
    if not window_focused():
        return "S"

    now = time.monotonic()

    with lock:
        keys = set(pressed) | {k for k, t in last_seen.items()
                               if now - t < KEY_HOLD_SECONDS}

    if "space" in keys or "x" in keys:
        return "S"
    # Check Left (A) and Right (D) BEFORE Forward (W) / Back (S)
    # so pressing D while holding W steers immediately!
    if "a" in keys and "d" not in keys:
        return "L"
    if "d" in keys and "a" not in keys:
        return "R"
    if "w" in keys:
        return "F"
    if "s" in keys:
        return "B"
    return "S"


def integrate_motion(command, dt):
    if command not in ("F", "B", "L", "R"):
        return

    turn = -1.0 if INVERT_TURN else 1.0

    with lock:
        if command == "L":
            pose["heading"] = (pose["heading"] - turn * TURN_RATE_DPS * dt) % 360.0
        elif command == "R":
            pose["heading"] = (pose["heading"] + turn * TURN_RATE_DPS * dt) % 360.0
        elif (gps_info["status"] != "FIX" or gps_info.get("assisted")) and DEAD_RECKON_WITHOUT_GPS:
            sign = 1.0 if command == "F" else -1.0
            d = sign * ROBOT_SPEED_MPS * dt
            h = math.radians(pose["heading"])
            pose["x"] += d * math.sin(h)
            pose["y"] += d * math.cos(h)
            add_trail_point_locked()

        # Keep assisted GPS coordinates live as the robot moves
        if gps_info.get("assisted"):
            lat, lng = xy_to_latlng(pose["x"], pose["y"])
            gps_info["lat"] = lat
            gps_info["lng"] = lng


def update_pose_from_fix_locked(info):
    global gps_pose_update_time

    if not origin_from_gps:
        if garbage_points:
            print("Note: {} item(s) logged before the first fix were placed "
                  "relative to the placeholder ORIGIN.".format(len(garbage_points)))
        set_origin_locked(info["lat"], info["lng"], True)
        gps_pose_update_time = time.monotonic()
        print("Hardware GPS fix acquired: origin set to {:.6f}, {:.6f}".format(
            info["lat"], info["lng"]))
        return

    now = time.monotonic()
    if now - gps_pose_update_time < 0.9:
        return
    gps_pose_update_time = now

    x, y = latlng_to_xy(info["lat"], info["lng"])
    dx, dy = x - pose["x"], y - pose["y"]
    dist = math.hypot(dx, dy)

    if dist > 15.0:
        pose["x"], pose["y"] = x, y
    elif dist >= GPS_DEADBAND_M:
        pose["x"] += GPS_SMOOTHING * dx
        pose["y"] += GPS_SMOOTHING * dy

    add_trail_point_locked()


def apply_gps_text(text):
    global gps_raw, gps_info, gps_last_fix, gps_last_fix_time
    global gps_last_update, gps_prev_raw_status

    info = parse_gps(text)
    now = time.monotonic()
    message = None

    with lock:
        gps_raw = text
        gps_last_update = now

        if info["status"] == "FIX":
            # Real hardware satellite fix always wins!
            info["assisted"] = False
            gps_last_fix = dict(info)
            gps_last_fix_time = now
            update_pose_from_fix_locked(info)
        elif (gps_last_fix is not None and not gps_last_fix.get("assisted") and
              now - gps_last_fix_time < GPS_HOLD_SECONDS):
            info = dict(gps_last_fix)
            info["held"] = True
            info["held_for"] = now - gps_last_fix_time
        elif (assist_gps_enabled and
              (now - app_start_time >= AUTO_ASSIST_GPS_AFTER_S) and
              info["status"] in ("NO_FIX", "NO_DATA", "NO_UART", "UNAVAILABLE")):
            # Engage software-assisted GPS lock so indoor demo works seamlessly
            lat, lng = xy_to_latlng(pose["x"], pose["y"])
            info = {
                "status": "FIX",
                "lat": lat,
                "lng": lng,
                "sats": info.get("sats", 0),
                "hdop": None,
                "held": False,
                "held_for": 0.0,
                "assisted": True,
            }

        if info["status"] != gps_prev_raw_status:
            message = "[GPS] {} -> {}   ({})".format(
                gps_prev_raw_status, info["status"], text[:70])
            gps_prev_raw_status = info["status"]

        gps_info = info

    if message:
        print(message)


def process_esp_reply(body):
    global esp_last_contact, esp_uptime, esp_reboots, esp_preset

    now = time.monotonic()
    gps_line = None
    uptime = None
    preset = None

    for line in body.splitlines():
        line = line.strip()
        if line.startswith("GPS"):
            gps_line = line
        elif line.startswith("UP="):
            try:
                uptime = int(line[3:])
            except ValueError:
                pass
        elif line.startswith("RES="):
            try:
                preset = int(line[4:])
            except ValueError:
                pass

    with lock:
        esp_last_contact = now
        if preset is not None:
            esp_preset = preset
        if uptime is not None:
            if esp_uptime is not None and uptime + 2 < esp_uptime:
                esp_reboots += 1
                print("[ESP32] Reboot detected (uptime reset). "
                      "Check the ESP32-CAM 5V supply.")
            esp_uptime = uptime

    if gps_line is not None:
        apply_gps_text(gps_line)


def command_loop():
    global active_cmd_display, pending_speed_cmd
    session = requests.Session()
    previous = "S"
    last_t = time.monotonic()
    last_http_time = 0.0

    while running:
        now = time.monotonic()

        with lock:
            spd = pending_speed_cmd
            pending_speed_cmd = None

        if spd is not None:
            try:
                session.get(ESP32 + "/cmd", params={"c": spd}, timeout=CMD_TIMEOUT_S)
                print(f"[SPEED] Motor speed level set to {spd}/9")
            except requests.RequestException:
                pass

        command = current_command()
        with lock:
            active_cmd_display = command

        interval = 0.25 if command != "S" else 0.80
        should_send = (command != previous) or ((now - last_http_time) >= interval)

        sent = False
        if should_send:
            try:
                reply = session.get(ESP32 + "/cmd", params={"c": command},
                                    timeout=CMD_TIMEOUT_S)
                if reply.status_code == 200:
                    sent = True
                    last_http_time = time.monotonic()
                    process_esp_reply(reply.text)
            except requests.RequestException:
                pass

        now = time.monotonic()
        dt = min(now - last_t, 0.5)
        last_t = now

        if previous != "S":
            integrate_motion(previous, dt)
        if sent or command != "S":
            previous = command

        time.sleep(0.04)

    session.close()


def link_watchdog_loop():
    while running:
        now = time.monotonic()
        with lock:
            since_gps = now - gps_last_update
            since_contact = now - esp_last_contact

        if since_contact > ESP_DOWN_AFTER_S and not video_alive():
            apply_gps_text("GPS: ESP32 unavailable")
        elif since_gps > 6.0 and since_contact <= ESP_DOWN_AFTER_S:
            apply_gps_text("GPS: no data from Arduino")

        time.sleep(1.0)


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


def toggle_assisted_gps():
    global assist_gps_enabled, app_start_time
    with lock:
        assist_gps_enabled = not assist_gps_enabled
        if assist_gps_enabled:
            app_start_time = time.monotonic() - AUTO_ASSIST_GPS_AFTER_S
        state = assist_gps_enabled
        raw = gps_raw
    apply_gps_text(raw)
    print("Indoor Assisted GPS Lock:", "ENABLED" if state else "DISABLED (Hardware GPS only)")


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


def cycle_resolution():
    with lock:
        current = esp_preset if esp_preset else 1
    nxt = current % 4 + 1

    def worker():
        try:
            reply = requests.get(ESP32 + "/res", params={"p": nxt}, timeout=2.0)
            print("Picture size preset ->", reply.text.strip(),
                  "(1=320x240 2=400x296 3=480x320 4=640x480)")
        except requests.RequestException:
            print("Could not change the picture size (is the ESP32 reachable?).")

    threading.Thread(target=worker, daemon=True).start()


def handle_command_key(name):
    global active_speed_level, pending_speed_cmd
    now = time.monotonic()
    with lock:
        if now - cmd_key_time.get(name, 0.0) < 0.3:
            return
        cmd_key_time[name] = now

    if name == "l":
        toggle_logging()
    elif name == "o":
        set_origin_here()
    elif name == "m":
        save_map_png()
    elif name == "r":
        cycle_resolution()
    elif name == "g":
        toggle_assisted_gps()
    elif name in SPEED_KEYS:
        with lock:
            active_speed_level = name
            pending_speed_cmd = name


def on_press(key):
    name = key_name(key)
    if name in CONTROL_KEYS:
        with lock:
            pressed.add(name)
    elif (name in ("l", "o", "m", "r", "g") or name in SPEED_KEYS) and window_focused():
        if name not in toggle_down:
            toggle_down.add(name)
            handle_command_key(name)


def on_release(key):
    name = key_name(key)
    if name in CONTROL_KEYS:
        with lock:
            pressed.discard(name)
    elif name in ("l", "o", "m", "r", "g") or name in SPEED_KEYS:
        toggle_down.discard(name)


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
    img = np.full((h, w, 3), (32, 32, 32), np.uint8)

    with lock:
        rp = dict(pose)
        pts = [dict(p) for p in garbage_points]
        tr = list(trail)
        info = dict(gps_info)

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

    ox, oy = to_px(0.0, 0.0)
    cv2.line(img, (ox - 8, oy), (ox + 8, oy), (255, 255, 255), 1)
    cv2.line(img, (ox, oy - 8), (ox, oy + 8), (255, 255, 255), 1)
    cv2.putText(img, "ORIGIN", (ox + 6, oy - 6), cv2.FONT_HERSHEY_SIMPLEX,
                0.4, (255, 255, 255), 1, cv2.LINE_AA)

    if len(tr) >= 2:
        poly = np.array([to_px(x, y) for x, y in tr], np.int32)
        cv2.polylines(img, [poly], False, (160, 160, 90), 1, cv2.LINE_AA)

    for p in pts:
        px, py = to_px(p["x"], p["y"])
        col = class_color(p["cls"])
        cv2.circle(img, (px, py), 7, col, -1)
        cv2.circle(img, (px, py), 7, (255, 255, 255), 1)
        cv2.putText(img, p["cls"][:12], (px + 9, py + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (230, 230, 230), 1, cv2.LINE_AA)

    rx, ry = to_px(rp["x"], rp["y"])
    hd = math.radians(rp["heading"])
    fx, fy = math.sin(hd), -math.cos(hd)
    sx, sy = -fy, fx
    tip = (int(rx + fx * 13), int(ry + fy * 13))
    b1 = (int(rx - fx * 8 + sx * 7), int(ry - fy * 8 + sy * 7))
    b2 = (int(rx - fx * 8 - sx * 7), int(ry - fy * 8 - sy * 7))
    cv2.fillPoly(img, [np.array([tip, b1, b2], np.int32)], (0, 220, 0))
    cv2.line(img, (rx, ry), (int(rx + fx * 30), int(ry + fy * 30)), (0, 220, 0), 1)

    nx, ny = right_px - 18, top_px + 30
    cv2.arrowedLine(img, (nx, ny), (nx, ny - 22), (255, 255, 255), 2, tipLength=0.4)
    cv2.putText(img, "N", (nx - 5, ny + 14), cv2.FONT_HERSHEY_SIMPLEX,
                0.45, (255, 255, 255), 1, cv2.LINE_AA)

    cv2.putText(img, "GARBAGE MAP (relative to origin)", (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

    cur_lat, cur_lng = xy_to_latlng(rp["x"], rp["y"])
    if info["status"] == "FIX" and not info.get("assisted"):
        src = "GPS SATELLITE FIX"
        cur_lat, cur_lng = info["lat"], info["lng"]
    elif info["status"] == "FIX" and info.get("assisted"):
        src = "GPS ASSISTED LOCK"
    else:
        src = "ORIGIN (no GPS)"

    y0 = bot_px + 20
    cv2.putText(img, "Grid {} m | Robot x={:.1f} y={:.1f} m | heading {:.0f} deg".format(
        step, rp["x"], rp["y"], rp["heading"]),
        (10, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1, cv2.LINE_AA)
    cv2.putText(img, "{} | {:.6f}, {:.6f}".format(src, cur_lat, cur_lng),
                (10, y0 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1,
                cv2.LINE_AA)

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
                    cv2.FONT_HERSHEY_SIMPLEX, 0.56, color, 1, cv2.LINE_AA)


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
        if not os.path.exists(model_path):
            raise SystemExit(
                "Model file not found: {}\nPut yolo11n-seg.pt in that folder.".format(
                    model_path))

    print("Loading model", model_path, "...")
    try:
        model = YOLO(model_path)
    except Exception as error:
        raise SystemExit("Could not load the model: {}".format(error))

    if LOAD_PREVIOUS_ON_START:
        with lock:
            load_previous_points()

    with lock:
        trail.append((0.0, 0.0))

    if pynput_keyboard is None:
        print("pynput not installed: using OpenCV keyboard mode.")

    threading.Thread(target=grab_loop, daemon=True).start()
    threading.Thread(target=inference_loop, args=(model,), daemon=True).start()
    threading.Thread(target=command_loop, daemon=True).start()
    threading.Thread(target=link_watchdog_loop, daemon=True).start()

    listener = None
    if pynput_keyboard is not None:
        listener = pynput_keyboard.Listener(on_press=on_press,
                                            on_release=on_release)
        listener.start()

    cam_h = int(CAM_DISPLAY_WIDTH * 0.75)
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, CAM_DISPLAY_WIDTH + MAP_PANEL_WIDTH, cam_h)

    print("Controls: W=forward, S=back, A=left, D=right, 1..9=motor speed")
    print("Space/X=stop, G=toggle instant GPS lock, L=logging on/off,")
    print("O=reset origin, M=save map picture, R=change camera size, ESC=exit")

    white = (255, 255, 255)
    green = (0, 255, 0)
    yellow = (0, 255, 255)
    red = (0, 0, 255)

    map_cache = None
    map_cache_time = 0.0

    try:
        while running:
            with lock:
                frame = annotated if annotated is not None else latest_frame
                total = logged_total
                last_text = last_logged_text
                logging_on = LOGGING_ON
                info = dict(gps_info)
                esp_up = esp_uptime
                reboots = esp_reboots
                preset = esp_preset
                cmd_now = active_cmd_display
                spd_now = active_speed_level

            if frame is None:
                cam = np.zeros((240, 320, 3), dtype=np.uint8)
                cv2.putText(cam, "Waiting for video...", (20, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, white, 2)
            else:
                cam = frame.copy()

            scale = CAM_DISPLAY_WIDTH / float(cam.shape[1])
            cam = cv2.resize(cam, (CAM_DISPLAY_WIDTH, int(cam.shape[0] * scale)),
                             interpolation=cv2.INTER_LINEAR)

            alive = video_alive()
            if not alive:
                cam = (cam * 0.35).astype(np.uint8)

            lines = [
                ("WASD move | 1-9 speed | G gps lock | L log | O origin | M map | R size",
                 white),
                (gps_display_text(info),
                 (green if not info["held"] else yellow) if info["status"] == "FIX"
                 else yellow),
                ("{} | CMD: {} | SPD: {}/9".format(
                    "VIDEO OK" if alive else "VIDEO LAGGING", cmd_now, spd_now),
                 green if alive else red),
                ("Log {} | saved {} | AI {:.1f} FPS | ESP up {} | size {} | reboots {}".format(
                    "ON" if logging_on else "OFF", total, fps_value,
                    "-" if esp_up is None else "{}s".format(esp_up),
                    "-" if preset is None else preset, reboots),
                 red if reboots else white),
                (last_text, yellow),
            ]
            draw_info_bar(cam, lines)

            now_r = time.monotonic()
            if (map_cache is None or now_r - map_cache_time > MAP_REFRESH_S or
                    map_cache.shape[0] != cam.shape[0]):
                map_cache = draw_map(MAP_PANEL_WIDTH, cam.shape[0])
                map_cache_time = now_r
            display = np.hstack([cam, map_cache])

            cv2.imshow(WINDOW, display)
            key = cv2.waitKey(25)

            if key != -1:
                key &= 0xFF
                if key == 27:
                    break
                name = "space" if key == 32 else chr(key).lower()
                if name in ("l", "o", "m", "r", "g") or name in SPEED_KEYS:
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
