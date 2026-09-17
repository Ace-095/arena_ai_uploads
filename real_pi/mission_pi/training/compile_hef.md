# best.onnx → qr_yolov8n.hef (Hailo Dataflow Compiler, x86)

DFC runs on x86 Linux only — never on the Pi. One-time setup, then each
retrain is parse → optimize → compile.

## 0. Install DFC + Model Zoo (x86 Ubuntu)

From hailo.ai/developer-zone: `hailo_dataflow_compiler-<ver>.whl` +
`hailo_model_zoo` wheel for the same version, in a venv with
`onnx`, `onnxruntime`, `numpy`. Verify: `hailo --help`.

Target hardware: Hailo-8 (the 26 TOPS AI HAT+). The flow below also
works for Hailo-8L (`--hw-arch hailo8l`) if you ever swap hats.

## 1. Calibration set (≥128 images, DIVERSE)

```
mkdir -p calib && cp qr_dataset/images/train/*.jpg calib/ | head -200
```

Mix scales/lighting — "quantization params" failures almost always mean
a monotonous calib set. Include real field frames once you have them.

## 2. Model script (.alls) — start from the Zoo's yolov8n

```
hailo_model_zoo/yolov8n.alls            # base (has nms_postprocess)
```

Append a line lowering the NMS floor if the bench shows dropped weak
boxes (field default: keep 0.25 first, drop to 0.1 only if needed):

```
nms_postprocess(score_threshold=0.1, iou_threshold=0.5, ... )
```

(Keep the rest of the zoo .alls untouched — it matches the runtime
parser in `detector.py`, which reads the NMS tensor.)

## 3. Parse → optimize → compile

```
hailo parse --hw-arch hailo8 --ckpt runs/qr_yolov8n/weights/best.onnx --yaml yolov8n.yaml
hailo optimize --hw-arch hailo8 --har best.har --calib-path calib/ --model-script yolov8n.alls --output-har-path qr.har
hailo compile --hw-arch hailo8 --har qr.har --output qr_yolov8n.hef
```

(`yolov8n.yaml` = input shape 640x640x3; copy the zoo's and change only
`classes: 1`.)

## 4. Verify on x86, then ship

```
hailortcli run qr_yolov8n.hef --input test_15m.jpg   # expect boxes
```

`scp qr_yolov8n.hef pi@<pi>:/home/pi/mission_pi/models/` and set
`detector.hef_path` in config. On the Pi: `python main.py --check`
should show the Hailo runtime OK.

## Troubleshooting (from the field guides)

| symptom | fix |
|---|---|
| no boxes at runtime, compile fine | lower `score_threshold` in .alls; check letterbox matches training |
| ReduceMax/SoftplusGrad warnings | re-export ONNX with `opset=13` |
| "Cannot find suitable quantization params" | more diverse calib set (≥128) |
| `hailortcli scan` empty on Pi | `sudo rpi-eeprom-update -a`, `dtparam=pciex1_gen=2` in `/boot/firmware/config.txt`, matching `hailo-all` version |
| boxes shifted/scaled | input normalization mismatch — runtime feeds plain uint8 RGB letterboxed; keep `FormatType.UINT8` |
