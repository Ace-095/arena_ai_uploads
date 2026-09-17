# Model slots

Active files (referenced by `config.yaml` → `detector.hef_path` / `model_path`):

| file | what | size | where used |
|---|---|---|---|
| `qr_yolov8n.hef` | fine-tuned 1-class QR YOLOv8n for the Hailo HAT (from `training/`) | ~? MB | Pi5 + Hailo HAT, `kind: auto` / `hailo` |
| `qr_yolov8n.pt` | YOLOv8n QR trained on synthetic A3, 500 imgs, 50 epochs, CPU — works on laptop without Hailo | 24 MB | Laptop Pop!_OS + SITL+Gazebo, `kind: yolo` or `custom` + `detectors.yolo_ultralytics` |
| `qr_yolov8n.onnx` | Same as .pt, exported ONNX opset 13, lighter, no torch needed | 12 MB | Laptop Pop!_OS, `kind: yolo` or `custom` + `detectors.yolo_onnx` (needs onnxruntime) |

We now commit .pt and .onnx (24MB+12MB) because they are needed for laptop SITL+Gazebo real test — classical FAILS at 10m/15m A3, YOLO mandatory.

Build them via `training/README.md`:
```bash
python3 training/make_qr_dataset.py --out /tmp/qr_dataset --n-train 500 --n-val 80 --img 640
python3 training/train_qr.py --data /tmp/qr_dataset/qr.yaml --model yolov8n.yaml --epochs 40 --imgsz 640 --batch 8 --device cpu
cp /tmp/runs/qr_yolov8n/weights/best.pt models/qr_yolov8n.pt
python3 -c "from ultralytics import YOLO; YOLO('models/qr_yolov8n.pt').export(format='onnx', opset=13)"
cp /tmp/runs/qr_yolov8n/weights/best.onnx models/qr_yolov8n.onnx
```

Bench (real run):
```
34px (A3@15m) classical: 0 boxes FAIL
34px yolo_full: 1 box 0.42 PASS, yolo_tiled_3x3: 1 box 0.81 PASS
50px (A3@10m) classical: 0 FAIL, yolo_tiled: 0.79 PASS
```
Proves YOLO mandatory for 10m/15m.

## Custom-model placeholder (future)

Set `detector.kind: custom` + `detector.custom_module: yourpkg.yourmod`,
where the module defines either:

```python
from detector import Detector, BBox

class Detector(Detector):          # or: def create_detector(): ...
    name = "mine"
    def detect(self, frame_bgr):   # BGR numpy, any size
        ...                        # -> [BBox(x, y, w, h, conf, "mine")]
        ...                        # coords in FULL-frame pixels, never raises
```

Budget to respect: 2 cameras × (bottom tiled 3x3 + front single) ≈
5 inferences/frame-cycle at ~10 Hz → keep single inference under ~15 ms
on the HAT, or narrow the tile grid in config. If the fine-tuned YOLO
holds up, this slot stays empty by design.
