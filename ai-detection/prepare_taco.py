"""
Step 1 of 2: download TACO and convert it to YOLO segmentation format.

Run this on your laptop or in Google Colab (it needs normal internet):

    python prepare_taco.py
    python prepare_taco.py --classes super      # 28 TACO super-categories
    python prepare_taco.py --classes full       # all 60 TACO categories

Output folder (default: ai-detection/taco_yolo):
    ai-detection/taco_yolo/images/train, ai-detection/taco_yolo/images/val
    ai-detection/taco_yolo/labels/train, ai-detection/taco_yolo/labels/val
    ai-detection/taco_yolo/data.yaml

Needs:  pip install pillow requests
"""
import argparse
import json
import os
import random
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from io import BytesIO
from pathlib import Path

import requests
from PIL import Image, ImageOps

ANNOTATIONS_URL = (
    "https://raw.githubusercontent.com/pedropro/TACO/master/data/annotations.json"
)
BASE_DIR = Path(__file__).resolve().parent

# "grouped" mode: 10 practical classes built from TACO super-categories.
# Anything not listed here becomes "Other litter".
GROUPS = {
    "Bottle": "Bottle",
    "Bottle cap": "Cap/Lid",
    "Lid": "Cap/Lid",
    "Pop tab": "Cap/Lid",
    "Can": "Can",
    "Carton": "Carton",
    "Cup": "Cup",
    "Paper": "Paper",
    "Paper bag": "Paper",
    "Plastic bag & wrapper": "Plastic bag/wrapper",
    "Plastic container": "Plastic container",
    "Glass jar": "Plastic container",
    "Cigarette": "Cigarette",
}
GROUPED_ORDER = [
    "Bottle", "Cap/Lid", "Can", "Carton", "Cup", "Paper",
    "Plastic bag/wrapper", "Plastic container", "Cigarette", "Other litter",
]


def load_annotations(out_dir):
    path = os.path.join(out_dir, "annotations.json")
    if not os.path.exists(path):
        print("Downloading annotations.json ...")
        r = requests.get(ANNOTATIONS_URL, timeout=60)
        r.raise_for_status()
        with open(path, "wb") as f:
            f.write(r.content)
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def build_class_map(data, mode):
    """Return (names_list, {taco_category_id: yolo_class_index})."""
    cats = {c["id"]: c for c in data["categories"]}

    if mode == "full":
        names = [cats[i]["name"] for i in sorted(cats)]
        return names, {cid: idx for idx, cid in enumerate(sorted(cats))}

    if mode == "super":
        names = sorted({c["supercategory"] for c in cats.values()})
        index = {n: i for i, n in enumerate(names)}
        return names, {cid: index[c["supercategory"]] for cid, c in cats.items()}

    names = list(GROUPED_ORDER)
    index = {n: i for i, n in enumerate(names)}
    mapping = {}
    for cid, c in cats.items():
        group = GROUPS.get(c["supercategory"], "Other litter")
        mapping[cid] = index[group]
    return names, mapping


def fetch_image(info, retries=3):
    """Download one image, upright, or return None."""
    urls = [info.get("flickr_640_url"), info.get("flickr_url")]
    for url in urls:
        if not url:
            continue
        for _ in range(retries):
            try:
                r = requests.get(url, timeout=30)
                r.raise_for_status()
                img = Image.open(BytesIO(r.content))
                img = ImageOps.exif_transpose(img).convert("RGB")
                # Annotations are in the upright orientation, so aspect must match.
                want = info["width"] / info["height"]
                got = img.width / img.height
                if abs(want - got) / want > 0.03:
                    return None
                return img
            except Exception:
                continue
    return None


def polygons_to_lines(anns, class_map, width, height):
    lines = []
    for a in anns:
        if a.get("iscrowd", 0):
            continue
        seg = a["segmentation"]
        if not isinstance(seg, list):
            continue                      # skip RLE masks (none in TACO)
        cls = class_map[a["category_id"]]
        for poly in seg:
            if len(poly) < 6:
                continue
            pts = []
            for i in range(0, len(poly) - 1, 2):
                x = min(max(poly[i] / width, 0.0), 1.0)
                y = min(max(poly[i + 1] / height, 0.0), 1.0)
                pts.append("{:.6f} {:.6f}".format(x, y))
            lines.append("{} {}".format(cls, " ".join(pts)))
    return lines


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(BASE_DIR / "taco_yolo"))
    ap.add_argument("--classes", choices=["grouped", "super", "full"],
                    default="grouped")
    ap.add_argument("--val", type=float, default=0.15, help="validation fraction")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--workers", type=int, default=8)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    data = load_annotations(args.out)
    names, class_map = build_class_map(data, args.classes)

    by_image = {}
    for a in data["annotations"]:
        by_image.setdefault(a["image_id"], []).append(a)

    images = [im for im in data["images"] if im["id"] in by_image]
    random.Random(args.seed).shuffle(images)
    n_val = max(1, int(len(images) * args.val))
    split_of = {}
    for i, im in enumerate(images):
        split_of[im["id"]] = "val" if i < n_val else "train"

    for split in ("train", "val"):
        os.makedirs(os.path.join(args.out, "images", split), exist_ok=True)
        os.makedirs(os.path.join(args.out, "labels", split), exist_ok=True)

    print("Classes ({}): {}".format(len(names), names))
    print("Downloading {} images ...".format(len(images)))

    done = skipped = 0
    counts = [0] * len(names)

    def work(info):
        split = split_of[info["id"]]
        stem = "taco_{:04d}".format(info["id"])
        img_path = os.path.join(args.out, "images", split, stem + ".jpg")
        lbl_path = os.path.join(args.out, "labels", split, stem + ".txt")

        if os.path.exists(img_path) and os.path.exists(lbl_path):
            with open(lbl_path) as f:
                return info, [l.split()[0] for l in f if l.strip()], True

        img = fetch_image(info)
        if img is None:
            return info, None, False

        lines = polygons_to_lines(
            by_image[info["id"]], class_map, info["width"], info["height"])
        if not lines:
            return info, None, False

        img.save(img_path, quality=92)
        with open(lbl_path, "w") as f:
            f.write("\n".join(lines) + "\n")
        return info, [l.split()[0] for l in lines], True

    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = [pool.submit(work, im) for im in images]
        for n, fut in enumerate(as_completed(futures), 1):
            info, cls_list, ok = fut.result()
            if ok:
                done += 1
                for c in cls_list:
                    counts[int(c)] += 1
            else:
                skipped += 1
            if n % 100 == 0:
                print("  {}/{}".format(n, len(images)))

    yaml_path = os.path.join(args.out, "data.yaml")
    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write("path: {}\n".format(os.path.abspath(args.out).replace("\\", "/")))
        f.write("train: images/train\nval: images/val\n")
        f.write("names:\n")
        for i, n in enumerate(names):
            f.write("  {}: {}\n".format(i, n))

    print("\nImages ready: {}   skipped: {}".format(done, skipped))
    print("Instances per class:")
    for n, c in zip(names, counts):
        print("  {:<22}{}".format(n, c))
    print("\nWrote", yaml_path)
    if done < 800:
        print("WARNING: fewer than 800 images downloaded. "
              "Check your internet connection and run the script again "
              "(it resumes where it stopped).")
        sys.exit(1)


if __name__ == "__main__":
    main()


