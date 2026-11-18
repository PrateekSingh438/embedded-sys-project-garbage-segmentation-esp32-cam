"""
Draw the converted TACO labels on a few images so you can SEE that the masks
line up with the litter before you spend hours training.

    python check_labels.py            # 12 random training images
    python check_labels.py --n 24

Open taco_yolo/preview/ afterwards. If the coloured outlines sit on the
objects, the dataset is good. If they are shifted, rotated or mirrored,
do not train: the images were not oriented like the annotations.
"""
import argparse
import random
from pathlib import Path

from PIL import Image, ImageDraw

BASE_DIR = Path(__file__).resolve().parent
COLORS = [(231, 76, 60), (46, 204, 113), (52, 152, 219), (241, 196, 15),
          (155, 89, 182), (26, 188, 156), (230, 126, 34), (127, 166, 166),
          (200, 73, 94), (31, 119, 180)]


def read_names(yaml_path):
    names, in_names = {}, False
    for line in yaml_path.read_text(encoding="utf-8").splitlines():
        if line.startswith("names:"):
            in_names = True
        elif in_names and ":" in line:
            k, v = line.strip().split(":", 1)
            names[int(k)] = v.strip()
    return names


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=str(BASE_DIR / "taco_yolo"))
    ap.add_argument("--n", type=int, default=12)
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()

    root = Path(args.data)
    names = read_names(root / "data.yaml")
    images = sorted((root / "images" / "train").glob("*.jpg"))
    if not images:
        raise SystemExit("No images found. Run prepare_taco.py first.")

    out = root / "preview"
    out.mkdir(exist_ok=True)

    for img_path in random.Random(args.seed).sample(images, min(args.n, len(images))):
        img = Image.open(img_path).convert("RGB")
        draw = ImageDraw.Draw(img)
        w, h = img.size
        label_file = root / "labels" / "train" / (img_path.stem + ".txt")

        for line in label_file.read_text().splitlines():
            parts = line.split()
            cls = int(parts[0])
            vals = [float(v) for v in parts[1:]]
            pts = [(vals[i] * w, vals[i + 1] * h) for i in range(0, len(vals) - 1, 2)]
            color = COLORS[cls % len(COLORS)]
            draw.line(pts + [pts[0]], fill=color, width=3)
            draw.text((pts[0][0] + 4, pts[0][1] + 4), names.get(cls, str(cls)), fill=color)

        img.save(out / img_path.name, quality=90)

    print("Saved {} preview images in {}".format(min(args.n, len(images)), out))


if __name__ == "__main__":
    main()