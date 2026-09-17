# Method 2: Real Bench Test with Pi 5 + Hailo AI HAT + Cameras — arena/test1

**Goal:** Validate polygon→fence, configurable height, FOV coverage, and A3 QR detection at 10/15 m on real Pi hardware **without props** (bench). This catches HEF quantization, focus, lighting issues before flight.

**Hardware:**
- Raspberry Pi 5 8 GB + Active Cooler
- Hailo AI HAT (Hailo-8) + heatsink
- Pi Camera Module 3 Standard (front, 66° HFOV, AF) + 15 cm CSI cable
- Waveshare IMX477 IR-CUT B 113° (bottom, 100° HFOV, fixed) + 30 cm CSI cable
- 27 W USB-C PSU, 32 GB SD, Ethernet or WiFi
- A3 print (297×420 mm) matte, high contrast QR, error correction Q

---

## Step 1: Pi OS + HailoRT + Picamera2

```bash
# Flash Raspberry Pi OS Lite 64-bit (Bookworm) via Imager
# Boot Pi 5, SSH

sudo apt update && sudo apt full-upgrade -y
sudo apt install -y python3-pip python3-venv git libcamera-apps

# HailoRT (from Hailo docs — version may change)
# https://hailo.ai/developer-zone/
wget https://hailo-csdata.s3.eu-west-2.amazonaws.com/ModelZoo/.../hailort_4.17.0_arm64.deb
sudo dpkg -i hailort_4.17.0_arm64.deb
sudo apt install -y hailo-all  # includes TAPPAS, HailoRT

# Verify HAT
hailortcli fw-control identify
# Should show Hailo-8, fw version

# Picamera2
pip3 install --upgrade picamera2 opencv-python pyzbar
# Or venv:
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

## Step 2: Clone arena/test1 + HEF model

```bash
cd ~
git clone https://github.com/Ace-095/arena_ai_uploads.git
cd arena_ai_uploads
git checkout arena/test1
cd workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi

# Check config.yaml — max height 15 m, detector tiled
cat config.yaml

# HEF model — should be in models/
ls models/
# qr_yolov8n.hef (1-class YOLOv8n, 640 input, NMS in HEF)
# If not present, download or compile (see 00_DETECTION doc)

# Test HEF loads
python3 - << 'PY'
from detector import get_detector
cfg = {"kind":"hailo","hef_path":"models/qr_yolov8n.hef","input_size":640,"conf_thr":0.25}
det = get_detector(cfg)
print("detector:", det.name)
PY
# Should print: detector: hailo
```

## Step 3: Camera bringup + focus calibration (critical for 15 m)

### Wiring

- Front Cam3 → CSI0 (CAM0 port, near USB-C)
- Bottom IMX477 B → CSI1 (CAM1 port, near HDMI)
- Check `libcamera-hello --list-cameras` — should show 2 cameras

### Focus

**Bottom IMX477 B fixed focus — must be set to infinity for 15 m:**

```bash
# Tool to check focus at distance
python3 tools/cam_probe.py --cam bottom --focus infinity --show

# Place A3 QR on floor, Pi on ladder at 3 m, 5 m, 10 m, 15 m (use tape measure)
# At each height, run:
python3 tools/cam_probe.py --cam bottom --distance 15 --qr-size 0.297 --show-bbox

# If blurry at 15 m, adjust lens ring: unscrew slightly (factory often at 2 m)
# Turn 1/8 turn, re-test until sharp at 15 m. Lock with tiny glue.

# For 10 m mission, focus compromise at 10 m infinity still okay, but verify.
```

**Front Cam3 AF:**

```bash
python3 tools/cam_probe.py --cam front --af continuous --show
# Should autofocus from 0.5 m to infinity, PDAF fast.
# At 15 m, should be sharp.
```

### IR-CUT

```bash
# Day mode: IR-CUT ON (visible light)
# The module has GPIO control — check Waveshare wiki
# For competition day, leave ON. Night needs IR illuminator + OFF.
python3 tools/cam_probe.py --cam bottom --ir-cut on
```

### Resolution

For bench, test both:

- Search: 2028×1520 @40 fps (binned, better SNR, 40 fps → more frames for streak)
- Decode: 4056×3040 @10 fps (full-res, more pixels for decode)

In `config.yaml`:
```yaml
cameras:
  bottom:
    size: [2028, 1520]  # 40 fps search sweet spot
```

During approach, code can switch to full-res if needed.

## Step 4: Detector tuning for A3 at 10/15 m

Edit `config.yaml`:

```yaml
detector:
  kind: auto
  hef_path: models/qr_yolov8n.hef
  input_size: 640
  conf_thr: 0.25      # LOWER for 15 m — was 0.35, 0.25 gives +30% recall
  iou_thr: 0.45
  tile_bottom: true   # MANDATORY for 15 m
  tile_grid: [3, 3]
  full_decode_every_n: 3
  cue_frames: 2

decode:
  required_streak: 3
  miss_tolerance: 2   # allow 2 miss frames at 15 m jitter

mission:
  sweep_alt_m: 15.0
  approach_stair_m: [12.0, 9.0, 7.0]
  grid_overlap: 0.35
  search_speed_ms: 2.0
  resweep_alt_m: 10.0

blob:
  enabled: true
```

### Bench test at 3 m, 5 m, 10 m, 15 m (ladder)

```bash
# Terminal 1: mission_pi main
source .venv/bin/activate
python3 main.py --config config.yaml

# Terminal 2: check detection
curl http://localhost:8000/api/camera/status | jq
curl http://localhost:8000/api/qr/status | jq

# Place A3 QR on floor, Pi on desk 3 m high, bottom cam facing down
# Watch logs: should see "TARGET: A3 QR board detected" + bbox

# Increase height to 5 m (ladder), 10 m, 15 m (balcony)
# At each height, note:
# - BBox conf at 15 m vs 10 m
# - Decode success rate
# - Tiling helps? Disable tiling and compare: conf drops from 0.4 to 0.05 at 15 m without tiling
```

**Expected:**

| Height | Full-res A3 px | Tile A3 px | 640 input px | Conf (HEF good) | Decode |
|--------|----------------|------------|--------------|-----------------|--------|
| 3 m    | 170 px         | 170        | 80 px        | 0.85            | 100%   |
| 5 m    | 100 px         | 100        | 47 px        | 0.70            | 100%   |
| 10 m   | 50 px          | 50         | 23 px        | 0.45            | 90%    |
| 15 m   | 34 px          | 34         | 16 px        | 0.30            | 60% (needs approach) |

At 15 m, detection 0.30 conf is okay if `conf_thr` 0.25 — triggers approach to 12 m where decode improves.

### HEF vs classical

```bash
# Test classical fallback (no HAT)
python3 main.py --config config.laptop.yaml  # kind: classical
# Classical works at 3-5 m but fails past 8 m — proves need for Hailo

# Test HEF
python3 tools/bench_hef.py --hef models/qr_yolov8n.hef --images tests/data/a3_at_15m/ --conf 0.25 --tiles 3x3
# Should be >0.8 recall with tiling, <0.2 without
```

## Step 5: Polygon + FOV coverage bench (no flight)

Even without flight controller, you can test polygon flow:

```bash
# On Pi, start mission_pi
python3 main.py --config config.yaml

# On Windows laptop, bridge in mock mode
python bridge/mp_bridge.py --mock --port 8100

# Open UI http://localhost:8100
# Draw polygon, set max alt 15, cam bottom, overlap 0.35
# Click POLYGON → FENCE INCLUSION, Show FOV coverage
# Verify footprint 35.7×23.0 m at 15 m, spacing 15 m
# Export polygon .poly, coverage plan JSON
```

## Step 6: Full bench with mock FC (loopback)

```bash
# Pop!_OS or Pi: fake SITL
python3 tools/fake_sitl.py --udp 192.168.1.20:14551 --auto --loop

# Pi: mission_pi
python3 main.py --config config.yaml

# Windows: bridge + UI
python bridge/mp_bridge.py --port 8100 --mav-port 14551
# Draw polygon, apply as fence, start mission, watch FSM SEARCH → TARGET_FOUND → DESCEND → DECODED
```

This proves Pi + HAT + cams + detector + mission FSM works end-to-end without props.

## Step 7: Logs and metrics

- Pi: `logs/` — `pending_results.jsonl`, `telemetry.log`, `camera_bottom.h264` if recording
- Check `http://pi-ip:8000/api/fsm/status` — state, payload, armable
- Check `http://pi-ip:8000/api/qr/status` — streak, confirmed, bbox
- Offline decode: `python3 tools/offline_decode.py --video logs/bottom_15m.h264 --conf 0.25 --tiles 3x3` — if offline works but online fails, lower conf_thr or increase fps

## Troubleshooting — Pi bench

| Symptom | Cause | Fix |
|---------|-------|-----|
| Hailo not found | hailort not installed or HAT not seated | `hailortcli fw-control identify`, reseat HAT, check PCIe `lspci` |
| Camera not found | CSI cable reversed or disabled | `libcamera-hello --list-cameras`, check cable, `raspi-config` → cameras |
| Bottom blurry at 15 m | Fixed focus at 2 m factory | Adjust lens ring to infinity, verify with cam_probe at 15 m |
| Detection 0 at 15 m | conf_thr 0.35 too high, no tiling | Set conf_thr 0.25, tile_bottom true, 3×3 |
| HEF low recall | Bad calibration set | Recompile HEF with far A3 images (see 00 doc) |
| High CPU | 4056×3040 @10 fps heavy | Use 2028×1520 @40 fps for search |
| IR-CUT wrong | Night mode in day | Set ir-cut on, check GPIO |

## Success criteria — Method 2

- [ ] Both cams detected, Hailo HAT identified, HEF loads
- [ ] Bottom focus sharp at 15 m (A3 QR readable in cam_probe)
- [ ] A3 detection at 3 m, 5 m, 10 m, 15 m bench with tiling: conf >0.25 at 15 m
- [ ] Decode streak 3/3 at 10 m, approach from 15 m → 12→9→7 m decodes
- [ ] Polygon drawn, converted to fence inclusion, FOV coverage shows correct footprint at 15/10/5 m
- [ ] Mock FC mission SEARCH → TARGET_FOUND → DECODED with A3 print
- [ ] Logs show `fov: {cam: bottom, hfov: 100, footprint_w: 35.7, ...}` and `max_alt_m: 15.0`
