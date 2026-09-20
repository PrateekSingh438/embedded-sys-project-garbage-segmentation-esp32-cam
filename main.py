import time
from datetime import datetime
from pathlib import Path

import cv2
from ultralytics import YOLO

camera_url = "http://192.168.4.1:81/stream"

# Instance segmentation model.
model = YOLO("yolo11n-seg.pt")

output = Path("segmentation_results")
output.mkdir(exist_ok=True)

last_alert = {}

results = model.predict(
    source=camera_url,
    stream=True,
    stream_buffer=False,
    classes=[39, 41],  # Bottle and cup
    conf=0.45,
    imgsz=640,
    retina_masks=True,
    device="cpu",
    verbose=False
)

try:
    for result in results:
        # Draw coloured masks without rectangular boxes.
        frame = result.plot(
            masks=True,
            boxes=False,
            labels=False,
            probs=False
        )

        detected = {}

        if result.masks is not None:
            for box in result.boxes:
                name = model.names[int(box.cls.item())]
                confidence = float(box.conf.item())

                detected[name] = max(
                    detected.get(name, 0),
                    confidence
                )

                # Use the box position only to place the text.
                # No rectangle is drawn.
                x1, y1, x2, y2 = map(
                    int, box.xyxy[0].tolist()
                )

                label = f"{name}: {confidence:.0%}"
                position = (max(0, x1), max(25, y1 - 10))

                # Black outline keeps the text readable.
                cv2.putText(
                    frame, label, position,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 0), 4
                )

                cv2.putText(
                    frame, label, position,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 255), 2
                )

        status = (
            "POTENTIAL LITTER - SEGMENTED"
            if detected else "Scanning"
        )

        cv2.putText(
            frame, status, (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (0, 0, 0), 4
        )

        cv2.putText(
            frame, status, (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (0, 255, 255), 2
        )

        for name, confidence in detected.items():
            now = time.monotonic()

            # At most one alert per class every 10 seconds.
            if now - last_alert.get(name, -100) < 10:
                continue

            stamp = datetime.now().strftime(
                "%Y%m%d_%H%M%S_%f"
            )

            image_path = output / f"{stamp}_{name}.jpg"

            if cv2.imwrite(str(image_path), frame):
                last_alert[name] = now

                print(
                    f"SEGMENTATION ALERT: {name} | "
                    f"Confidence: {confidence:.0%} | "
                    f"Saved: {image_path}"
                )
            else:
                print("Could not save segmentation image")

        cv2.imshow("Garbage Instance Segmentation", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

except KeyboardInterrupt:
    pass
finally:
    results.close()
    cv2.destroyAllWindows()