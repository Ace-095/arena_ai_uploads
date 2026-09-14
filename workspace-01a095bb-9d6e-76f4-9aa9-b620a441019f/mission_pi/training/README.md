# QR YOLO fine-tune loop (x86 machine, not the Pi)

Target: a 1-class `qr` YOLOv8n that fires on an A3 QR at 10–140 px on a
640 input — the tiled bottom-cam regime at 6–18 m. Re-run this loop
whenever field frames show misses.

## 0. Install (x86, GPU recommended)

```
pip install ultralytics qrcode pillow numpy opencv-python
```

## 1. Generate data

```
python make_qr_dataset.py --out qr_dataset --n-train 3000 --n-val 400
```

Scales bracket the mission (`--px-min/--px-max`), lighting/blur/noise
cover "any light condition". Payloads default to 6 alnum chars — the
same shape mission-ui auto-latches over MAVLink.

## 2. Train + export

```
python train_qr.py --data qr_dataset/qr.yaml --epochs 120 --imgsz 640
```

Watch `mAP50` on val; for a 1-class high-contrast target expect >0.9.
Export leaves `runs/qr_yolov8n/weights/best.onnx` (opset 13, DFC-safe).

## 3. Compile to HEF

Follow `compile_hef.md` (Hailo Dataflow Compiler on x86) → copy the
result to `../models/qr_yolov8n.hef` on the Pi.

## 4. Bench before flight

```
python ../tools/qr_bench.py --detector hailo --hef ../models/qr_yolov8n.hef <frames-dir>
```

## 5. Close the loop with field frames

Hard cases from real flights beat more synthetic data: save miss frames
from the UI stream, label the QR box (any YOLO labeller), drop them into
`qr_dataset/images/train` + `labels/train`, retrain from the last
`best.pt` (`--model runs/.../best.pt --epochs 40`). Keep the synthetic
base — it prevents overfitting to one field.
