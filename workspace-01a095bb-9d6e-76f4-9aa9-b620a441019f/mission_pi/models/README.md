# Model slots

Active file (referenced by `config.yaml` → `detector.hef_path`):

| file | what |
|---|---|
| `qr_yolov8n.hef` | fine-tuned 1-class QR YOLOv8n for the Hailo HAT (from `training/`) |
| `qr_yolov8n.pt` / `.onnx` | training archive copies (optional, same folder) |

No weights are committed — build them via `training/README.md`.

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
