"""
Step 2 of 2: fine-tune YOLO11 segmentation on the prepared TACO data.

    python ai-detection/train_taco.py
    python ai-detection/train_taco.py --epochs 100 --imgsz 640 --batch 16
    python ai-detection/train_taco.py --export onnx

A GPU is strongly recommended. On a laptop CPU, training takes many hours.
Use Google Colab (Runtime -> Change runtime type -> GPU) if you have no GPU.

Output:
    models/garbage-seg.pt
    runs/taco_seg/   <- plots, confusion matrix, metrics
"""
import argparse
import os
import shutil
from pathlib import Path

from ultralytics import YOLO

BASE_DIR = Path(__file__).resolve().parent
MODELS_DIR = BASE_DIR / "models"
RUNS_DIR = BASE_DIR / "runs"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=str(BASE_DIR / "taco_yolo" / "data.yaml"))
    ap.add_argument("--model", default=str(MODELS_DIR / "yolo11n-seg.pt"),
                    help="start weights. yolo11s-seg.pt is more accurate but slower")
    ap.add_argument("--epochs", type=int, default=80)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--device", default=None, help="0 for GPU, cpu for CPU")
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--name", default="taco_seg")
    ap.add_argument("--export", choices=["onnx", "openvino", "ncnn"], default=None)
    args = ap.parse_args()

    if not os.path.exists(args.data):
        raise SystemExit("Cannot find {}. Run prepare_taco.py first.".format(args.data))

    device = args.device
    if device is None:
        try:
            import torch
            device = "0" if torch.cuda.is_available() else "cpu"
        except Exception:
            device = "cpu"

    if device == "cpu":
        print("WARNING: no GPU found. Training will be slow.")
        print("Tip: use --epochs 30 --imgsz 416 --batch 8, or train on Colab.")

    model = YOLO(args.model)

    model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=device,
        workers=args.workers,
        patience=20,             # stop early if validation stops improving
        project=str(RUNS_DIR),
        name=args.name,
        exist_ok=True,
        # Augmentation suited to outdoor litter photos
        mosaic=1.0,
        hsv_h=0.015, hsv_s=0.6, hsv_v=0.4,
        degrees=10.0,
        fliplr=0.5,
        scale=0.5,
    )

    best = os.path.join(str(RUNS_DIR), args.name, "weights", "best.pt")
    if not os.path.exists(best):
        raise SystemExit("Training did not produce best.pt")

    # ---------- Evaluation ----------
    print("\nEvaluating best.pt on the validation set ...")
    best_model = YOLO(best)
    metrics = best_model.val(data=args.data, imgsz=args.imgsz, device=device)

    print("\n=========== RESULTS ===========")
    print("Box  mAP50    : {:.3f}".format(metrics.box.map50))
    print("Box  mAP50-95 : {:.3f}".format(metrics.box.map))
    print("Mask mAP50    : {:.3f}".format(metrics.seg.map50))
    print("Mask mAP50-95 : {:.3f}".format(metrics.seg.map))
    print("\nPer-class mask mAP50-95:")
    names = best_model.names
    for i, idx in enumerate(metrics.seg.ap_class_index):
        print("  {:<22}{:.3f}".format(names[int(idx)], metrics.seg.maps[int(idx)]))
    print("Plots saved in {}/".format(RUNS_DIR / args.name))

    # ---------- Copy to the name camera.py expects ----------
    MODELS_DIR.mkdir(parents=True, exist_ok=True)
    output_model = MODELS_DIR / "garbage-seg.pt"
    shutil.copyfile(best, output_model)
    print("\nSaved", output_model)

    # ---------- Optional optimisation ----------
    if args.export:
        print("\nExporting to {} ...".format(args.export))
        out = best_model.export(format=args.export, imgsz=416)
        print("Exported:", out)
        print("Use it by setting MODEL_NAME in camera.py to this file.")


if __name__ == "__main__":
    main()   # this guard is required on Windows

