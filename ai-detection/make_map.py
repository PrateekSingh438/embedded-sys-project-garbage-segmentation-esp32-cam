"""
Build an interactive map (HTML) from detections/detections.csv.

    python -m pip install folium
    python make_map.py

Opens detections/garbage_map.html in your browser (needs internet for map tiles).
Each marker = one piece of garbage, coloured by type. Click a marker to see
type, confidence, time, size, coordinates and whether the position came
from a real GPS fix or from the ORIGIN fallback.
"""
import csv
import os
import webbrowser
from collections import Counter
from pathlib import Path

import folium

BASE_DIR = Path(__file__).resolve().parent
DETECTIONS_DIR = BASE_DIR / "detections"
CSV_PATH = DETECTIONS_DIR / "detections.csv"
OUT_PATH = DETECTIONS_DIR / "garbage_map.html"

COLORS = ["red", "green", "blue", "orange", "purple", "darkred",
          "cadetblue", "darkgreen", "pink", "gray"]


def main():
    if not os.path.exists(CSV_PATH):
        raise SystemExit("No {} yet. Run camera.py and detect something first.".format(CSV_PATH))

    rows = []
    with open(CSV_PATH, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            try:
                r["_lat"] = float(r["garbage_lat"])
                r["_lng"] = float(r["garbage_lng"])
                rows.append(r)
            except (ValueError, KeyError):
                continue

    if not rows:
        raise SystemExit("The CSV has no rows with coordinates yet.")

    lat0 = sum(r["_lat"] for r in rows) / len(rows)
    lng0 = sum(r["_lng"] for r in rows) / len(rows)
    fmap = folium.Map(location=[lat0, lng0], zoom_start=20, max_zoom=22)

    types = sorted({r["garbage_type"] for r in rows})
    color_of = {t: COLORS[i % len(COLORS)] for i, t in enumerate(types)}

    # Robot positions (small grey dots)
    seen = set()
    for r in rows:
        try:
            rl, rn = float(r["latitude"]), float(r["longitude"])
        except ValueError:
            continue
        if (rl, rn) in seen:
            continue
        seen.add((rl, rn))
        folium.CircleMarker([rl, rn], radius=3, color="gray", fill=True,
                            tooltip="Robot position").add_to(fmap)

    # Garbage markers
    for r in rows:
        popup = (
            "<b>{type}</b><br>Confidence: {conf}<br>Time: {time}<br>"
            "Size: {size}<br>Distance from robot: {dist} m<br>"
            "Position source: {src}<br>Lat/Lng: {lat:.6f}, {lng:.6f}<br>"
            "Snapshot: {snap}"
        ).format(type=r["garbage_type"], conf=r["confidence"], time=r["timestamp"],
                 size=r["size_class"], dist=r["distance_m"], src=r["gps_source"],
                 lat=r["_lat"], lng=r["_lng"], snap=r["snapshot"])
        folium.Marker(
            [r["_lat"], r["_lng"]],
            popup=folium.Popup(popup, max_width=300),
            tooltip=r["garbage_type"],
            icon=folium.Icon(color=color_of[r["garbage_type"]], icon="trash", prefix="fa"),
        ).add_to(fmap)

    fmap.save(OUT_PATH)

    print("Garbage items:", len(rows))
    for t, n in Counter(r["garbage_type"] for r in rows).most_common():
        print("  {:<22}{}".format(t, n))
    print("Saved", os.path.abspath(OUT_PATH))
    webbrowser.open("file://" + os.path.abspath(OUT_PATH))


if __name__ == "__main__":
    main()
