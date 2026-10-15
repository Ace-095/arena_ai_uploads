# qr_yolov8n.onnx → qr_yolov8n.hef (Hailo Dataflow Compiler, x86)

The Hailo AI HAT is idle until this exists: `detector.kind: auto` climbs
Hailo → ONNX → .pt → classical, and with no `.hef` it silently settles on the
CPU rung. **Measured on the Pi 5 (2026-10-11): tiled ONNX at 2028×1520 runs at
~1 fps (~900 ms/frame, 9 inferences per frame).** At 15 m the 3×3 tiling is
what makes an A3 QR visible at all, so the CPU rung can either see the QR or
keep up — never both. The HEF does the same work in milliseconds, which is
what "QR → MP with no delay" actually requires.

DFC runs on **x86 Linux only** — never on the Pi. One-time setup, then each
retrain is parse → optimize → compile.

## 0. Install the DFC + Model Zoo (x86 Ubuntu)

The Dataflow Compiler is **not on PyPI** (checked: every `hailo*` package
404s) — it comes only from Hailo's developer zone, as a wheel or as their
ready-made docker image:

* docker: `hailo8_ai_sw_suite_<ver>.tar.gz` → `docker load` → run it; the
  suite has `hailo`, the model zoo and `hailortcli` inside, which is the least
  error-prone route;
* or the wheel: `pip install hailo_dataflow_compiler-<ver>-py3-none-linux_x86_64.whl`
  plus `hailo_model_zoo` in a venv with `onnx`, `onnxruntime`, `numpy`.

Verify: `hailo --help` prints the version, e.g. `hailo 3.30.0`.

**Match the versions to the Pi's runtime.** HailoRT on the Pi (`hailortcli
--version`) and the DFC must be from the same major line (HailoRT 4.x ↔ DFC
3.x); a HEF built by a newer DFC than the runtime will fail to load with an
unhelpful error.

## 0.5 Which Hailo is in the hat? — this decides `--hw-arch`

```bash
ssh ace@addc001.local 'hailortcli fw-control identify'   # on the Pi
```

Look at **Device Architecture**:

| report says | compile with |
|---|---|
| `HAILO8` (26 TOPS AI HAT+) | `--hw-arch hailo8` |
| `HAILO8L` (13 TOPS AI HAT+ / AI Kit) | `--hw-arch hailo8l` |

Getting this wrong is the most common "the HEF is rejected" cause, and it is
invisible until runtime: a HEF for `hailo8l` will not load on a `hailo8` and
vice versa. `hailortcli fw-control identify` also gives the FW version, which
is what you match the DFC against.

## 1. Calibration set (≥128 images, DIVERSE, letterboxed like the runtime)

Quantization ranges are tuned on these images, so they must look like what the
Pi feeds the network — not like what `cv2.resize` happens to do. The two rules
that matter, and that `tools/make_calib_set.py` enforces:

* **same letterbox as the runtime** — the tool *imports* `detector._letterbox`
  rather than re-implementing it, so the compiler cannot be pointed at a
  distribution the network never sees in flight (an earlier version used
  `INTER_AREA` when shrinking and differed by up to 131 grey levels on a
  1520×2028 frame — those calib images are not what inference gets);
* **diversity beats volume** — 300–1000 frames with real exposure and scene
  variety are worth more than 1000 near-identical ones.

```bash
cd real_pi/mission_pi

# 1) capture frames ON THE PI (the camera lives there). Any of these work:
#    a) 2 minutes of the bench tool, saving what it sees:
python3 tools/test_yolo_webcam_qr.py --source csi:bottom --no-tile \
    --preset day --save-frames /tmp/bench          # one jpg per frame
#    b) the dataset you already built for training (most representative):
#       /tmp/qr_dataset_med/images/train  (see training/make_qr_dataset.py)
#    c) an existing video of a flight/walkthrough: --video flight.mp4

# 2) build the set (this step is pure numpy/cv2 — it runs on the Pi or on x86)
python3 tools/make_calib_set.py --from /tmp/bench \
    --out training/calib --want 256 --npz
```

It drops black / blown-out / flat frames and near-duplicates, warns when the
set is monotonous (the #1 cause of *"cannot find suitable quantization
params"*), and prints the `hailo optimize` line for you. What it refuses to
do is guess: with fewer than 128 frames, or a brightness spread under 25 grey
levels, or all frames at one exposure, it says so instead of compiling a set
that will quantize badly.

**Include, deliberately:** the QR at several distances (2 m → 15 m) and angles,
day and night lighting, and frames with **no** QR in view — background hunting
is most of what the network does at altitude. `--npz` additionally writes
`training/calib.npz` (`(N,640,640,3)` uint8) for DFC flows that want a numpy
set instead of a folder.

## 2. Model script (.alls) — the Zoo's yolov8n, with three edits

```bash
cp $HAILO_MODEL_ZOO/hailo_model_zoo/cfg/alls/generic/yolov8n.alls ./yolov8n.alls
```

The Zoo file already carries the yolov8 meta-arch, the sigmoid fixup on every
head and `nms_postprocess` — the parts that are easy to get wrong. Edit only
these lines (the Zoo's originals are quoted so you can confirm you are looking
at the same file):

| Zoo line | change to | why |
|---|---|---|
| `nms_postprocess("../../postprocess_config/yolov8n_nms_config.json", meta_arch=yolov8, engine=cpu)` | point the path at **our** `training/nms_config.json` (absolute path is simplest) | our config is 1 class, `score_threshold 0.25`, and it names **our** decoders `qr_yolov8n/bbox_decoder41\|52\|62` — the COCO one has 80 classes and 80 decoders and will not match this network |
| `change_output_activation(conv42\|conv53\|conv63, sigmoid)` | leave as is | those are our head convs (stride 8/16/32); removing it makes every score meaningless |
| `normalization([0,0,0],[255,255,255])` | leave as is | the runtime feeds plain uint8 RGB (`FormatType.UINT8`) — adding a mean/std here is the classic "boxes shifted or scaled" bug |

Keep `nms_postprocess` — our runtime parses the NMS tensor directly
(`hailo_decode.py`, the code `detector.py` calls). The class count itself comes
from the parse step (§3, the network yaml's `classes: 1`), not from the .alls.

The NMS floor lives in `training/nms_config.json` and is deliberately low
(`nms_scores_th 0.25`): it is applied BEFORE our own filter, so a high value
deletes exactly the small, far-away QR this mission hunts. Validate any config
you edit with `tools/hef_info.py --nms-config training/nms_config.json` — it
cross-checks the decoder names, strides and the reg/cls pairs per head, which
is how a rebuilt HEF silently ends up with no usable boxes.

### What the HEF actually emits (measured, 2026-10-12)

Compiling with `nms_postprocess` gives ONE output,
`qr_yolov8n/yolov8_nms_postprocess` = **`[1, 5, 100]`**: 5 values x 100 boxes x
1 class. HailoRT stores it **features-major** —

| axis | meaning |
|---|---|
| `[0]` | batch (1) |
| `[1]` | **5 values per box, in this order**: `y_min, x_min, y_max, x_max, score` — **normalised to 0..1**, not pixels |
| `[2]` | up to 100 detections (the `max_proposals` of the NMS config) |

So it is *not* "100 boxes of 6" and *not* the `(1,5,8400)` raw head. The old
parser read shape `[1,5,100]` as unsupported and returned no boxes for every
frame — which looks exactly like a dead model. `tools/hef_info.py` and
`hailo_decode.py` read it as **style A'**, and the box order is auto-detected
(y_min-first vs x_min-first) from the geometry, so a version that emits xyxy
still works.

For C classes the same tensor is `[1, 5*C, N]`; a 400-row output would mean an
80-class (COCO) network and `hef_info` warns about it.

The runtime **also** parses the raw head (style B: `(1,5,8400)` cxcywh+conf,
where the anchor count 8400 is what distinguishes it from A'), so a HEF
compiled *without* NMS still works — just slower, because NMS then runs on the
Pi's CPU. Prefer the NMS-in-HEF variant.

## 3. Parse → optimize → compile

Our checkpoint's facts (read out of `models/qr_yolov8n.onnx` with `onnx`, so
nothing here is guesswork):

| | value |
|---|---|
| opset | **13** (already the recommended one — no re-export warning) |
| input | `images` — `[1, 3, 640, 640]` float32 |
| output | `output0` — `[1, 5, 8400]` (raw ultralytics head, NMS off) |
| nodes / ops | 263 — Conv 64, Mul 60, Sigmoid 58, Concat 17, Add 9, Split 8, Reshape 8, MaxPool 3, **Resize 2**, Transpose/Slice/Softmax/… |
| trained | Ultralytics 8.4.153, `sha256` prefix `60ae1a1b9b0faced`, 1 class (`qr`) |

```bash
# 1) parse: ONNX -> HAR, using the zoo's yolov8n yaml (change classes: 1)
hailo parse --hw-arch hailo8 \
    --ckpt models/qr_yolov8n.onnx \
    --yaml $HAILO_MODEL_ZOO/hailo_model_zoo/cfg/networks/yolov8n.yaml \
    --har qr_yolov8n.har

# 2) optimize: quantize with OUR calibration set
hailo optimize --hw-arch hailo8 \
    --har qr_yolov8n.har \
    --calib-path training/calib/ \
    --model-script yolov8n.alls \
    --output-har-path qr_yolov8n_quantized.har

# 3) compile
hailo compile --hw-arch hailo8 \
    --har qr_yolov8n_quantized.har \
    --output models/qr_yolov8n.hef
```

(`--hw-arch` = what `hailortcli fw-control identify` said, see §0.5. On
`--yaml`: the zoo ships a per-network yaml with the right end nodes; if your
zoo version calls it differently, `hailo parse --help` lists accepting
`network_name` instead.)

## 4. Verify before shipping — on x86, then on the Pi

`training/nms_config.json` in this folder is the starting config for the
zoo's `nms_postprocess` (1 class, `[640,640]`, `nms_scores_th: 0.25` — a low
floor on purpose: the HEF's score threshold is applied BEFORE our own filter,
so a high value deletes exactly the small, far-away QR the mission hunts).
Reference it from the .alls if your zoo version wants a file, and check any
config you write with:

```bash
python3 tools/hef_info.py --nms-config training/nms_config.json
```

```bash
# x86: does the HEF load and produce boxes?
hailortcli run models/qr_yolov8n.hef --input <one of the calib/*.jpg>

# ship it
scp models/qr_yolov8n.hef ace@addc001.local:~/arena_ai_uploads/real_pi/mission_pi/models/
```

On the Pi, in this order (each one tells you something different):

```bash
cd ~/arena_ai_uploads/real_pi/mission_pi
hailortcli scan                                   # HAT present?
hailortcli fw-control identify                    # Device Architecture -> --hw-arch
python3 tools/hef_info.py                         # does THIS Pi accept the HEF,
                                                  # and can our parser read its
                                                  # outputs? (fastest verdict)
hailortcli run models/qr_yolov8n.hef              # HEF accepted by THIS runtime?
python3 tools/list_cams.py                        # "models/qr_yolov8n.hef OK"
python3 tools/qr_bench.py --detector hailo --hef models/qr_yolov8n.hef \
    --tiles snaps/*.jpg                           # boxes + payload from stills
python3 tools/test_yolo_webcam_qr.py --source csi:bottom --hailo \
    --hef models/qr_yolov8n.hef --preset day --require-decode --tiled
python3 main.py --check                           # the mission's own bring-up
```

`main.py --check` and the UI's bring-up panel report which rung of the ladder
was chosen, so "Hailo" appearing there is the end-to-end proof.

## Troubleshooting (field-tested order)

| symptom | cause / fix |
|---|---|
| `hailortcli run` says the HEF is unsupported / won't load | `--hw-arch` mismatch (hailo8 vs hailo8l, §0.5) or DFC newer than HailoRT |
| `hailortcli scan` empty on the Pi | `sudo rpi-eeprom-update -a`, `dtparam=pciex1_gen=2` in `/boot/firmware/config.txt`, matching `hailo-all` version |
| compiles fine, no boxes at runtime | NMS floor too high for your boxes (`score_threshold`), or the calib set was monotonous; check with the same frames through `--detector onnx` — if ONNX sees the QR and Hailo does not, it is quantization, and the fix is a more diverse calib set |
| boxes shifted/scaled | input normalization mismatch — the runtime feeds plain uint8 RGB letterboxed (`FormatType.UINT8`); do not add your own mean/std into the compile |
| many weak/duplicate boxes | NMS missing from the HEF (style B path): keep `nms_postprocess` in the .alls |
| `ReduceMax`/`SoftplusGrad` warnings | re-export with `opset=13` (ours already is) |
| detectors disagree about confidence | expected: HEF quantization shifts scores slightly; tune `detector.conf_thr` with the bench tool, not by eye |

The `.hef` is ~2-4 MB (smaller than the 24 MB `.pt`), and this repo already
commits `qr_yolov8n.pt`/`.onnx` because the laptop's SITL flights need them —
so commit the `.hef` too, next to them, and the Pi stays reproducible from a
`git pull`. Keep the `calib/` set and the intermediate `.har`s on the x86 box
(they are hundreds of MB); only the `.hef` ships.
