# Detection at 10–15 m for A3 QR — Pixel Budget, HEF Impact, and How to Make It Work Every Time

**Target:** A3 sheet 297 × 420 mm lying flat. **Search alt:** 10 m or 15 m (configurable via `flight.max_alt_m`). **Camera:** Waveshare IMX477 B 113° diagonal (~100° HFOV, 75° VFOV) bottom-facing, plus Pi Cam3 front 66° HFOV.

---

## 1. Why 15 m is hard — pixel budget

### Bottom cam geometry

From `cam.txt`:
- IMX477 B: 4056×3040 (4:3), 113° diag → ~100° HFOV / 75° VFOV (rectilinear approx, calibrate yours).
- Footprint at altitude `alt`:
  ```
  w = 2·alt·tan(100°/2) = 2·alt·1.1917
  h = 2·alt·tan(75°/2)  = 2·alt·0.7673
  ```
  - At 15 m: w=35.7 m, h=23.0 m
  - At 10 m: w=23.8 m, h=15.3 m
  - At 5 m:  w=11.9 m, h=7.7 m

Ground sample distance (GSD):
- 4056 px / 35.7 m = 113.6 px/m at 15 m → 8.8 mm/px
- A3 297 mm → **33.7 px wide** at 15 m in full-res frame.
- In 2028×1520 binned mode (recommended for search): 2028/35.7=56.8 px/m → 5.2 mm/px → A3 ≈ **57 px** but sensor binning improves SNR 2×, and 40 fps gives more frames for streak.

### YOLO input bottleneck

Hailo YOLOv8n HEF is compiled for 640×640 input (letterboxed). If you feed whole 4056×3040 frame:

- Resize factor s = min(640/4056, 640/3040)=0.157
- A3 34 px → **5.3 px** on network input — **undetectable** for any YOLO, even fine-tuned.

**Solution — 3×3 tiled inference (the 15 m answer):**

- Split bottom frame into 3×3 overlapping tiles (overlap 0.25).
- Each tile: 4056/3≈1352 px wide, A3 still 34 px inside tile.
- Letterbox tile 1352→640: s=0.473 → A3 ≈ **16 px** on input — small but detectable for 1-class YOLOv8n trained on small QRs.
- In 2028×1520 mode: tile 676 px → s=0.946 → A3 ≈ **32 px** on input — much easier, plus 40 fps.

**At 10 m:** A3 50 px full-res → tile 50 px → 640 input 23.7 px — significantly easier, ~2× detection recall vs 15 m.

This is why `detector.tile_bottom: true` and `tile_grid: [3,3]` are **mandatory** for 10–15 m. Without tiling, you will never detect A3 at 15 m regardless of HEF quality.

### Front cam (66° HFOV)

Footprint at 15 m: w=19.5 m, h=11.2 m. GSD 4608/19.5=236 px/m → 4.2 mm/px → A3 ≈ 70 px — better than bottom for detection, but front is forward-facing, not nadir, so it sees QR at oblique angle. Use front for **approach decode** (AF helps), bottom for **search detection**.

---

## 2. HEF format — does it decrease detection?

**HEF = Hailo Executable Format** — compiled model for Hailo-8 AI HAT. Compilation involves quantization (FP32 → INT8) and layer mapping.

**Yes, HEF can decrease detection if compiled badly:**

| Issue | Symptom | Fix |
|-------|---------|-----|
| **Calibration set has no far A3** | Model quantized thresholds tuned for close QR, far QR activations clipped to zero → recall drops 30-50% at 15 m | Recompile with calibration set containing 1024 images: 30% A3 at 10-15 m (sim + real), varied lighting, blur, rotation |
| **Input size mismatch** | Trained at 640 but HEF compiled at 512, or letterbox vs stretch confusion | Keep 640, letterbox, verify `hef.get_input_vstream_infos()` |
| **NMS in HEF vs host NMS** | HEF with NMS baked (output shape …,6) uses fixed IoU 0.5, may suppress small boxes if conf low | Use NMS-in-HEF for speed but lower `conf_thr` to 0.25, or use raw head + host NMS |
| **FP16 sensitive layers forced to INT8** | Small object head layers lose dynamic range | Use Hailo Model Zoo `hailo_model_zoo/cfg/retrain` with `hw_arch: hailo8` and `calib_set` + `nms_postprocess` config, allow FP16 for last layers |
| **Old HEF trained on QR close-up only** | Works at 2-5 m, fails at 10-15 m | Fine-tune YOLOv8n on dataset with A3 at distance: generate sim frames via `tools/make_demo_frames.py` + real Pi captures at 10/15 m ladder |

**Does HEF inherently decrease vs ONNX?** ~2-5% mAP drop is normal after INT8 quant if calibration is good. 20%+ drop means bad calibration.

**How to test your HEF:**

```bash
# On Pi 5
python3 tools/bench_hef.py --hef models/qr_yolov8n.hef --images tests/data/a3_at_15m/ --conf 0.25
# Should get >0.8 recall on A3 at 15 m tiles. If <0.5, recompile.

# Compare ONNX vs HEF
python3 tools/compare_onnx_hef.py --onnx models/qr_yolov8n.onnx --hef models/qr_yolov8n.hef
```

---

## 3. How to make detection happen every time at 10/15 m — A3 specific

### A. Config (mission_pi/config.yaml) — optimized for A3 at 15 m

```yaml
flight:
  max_alt_m: 15.0   # or 10.0

cameras:
  bottom:
    size: [2028, 1520]   # 40 fps binned = SNR + fps sweet spot for search
    hfov_deg: 100.0
    vfov_deg: 75.0

detector:
  kind: auto
  hef_path: models/qr_yolov8n.hef
  input_size: 640
  conf_thr: 0.25          # LOWER for 15 m (was 0.35) — more recall, more false positives okay because decode filters
  iou_thr: 0.45           # slightly lower to keep overlapping tile detections
  tile_bottom: true
  tile_grid: [3, 3]
  full_decode_every_n: 3  # more frequent full-frame decode attempt (was 5)
  cue_frames: 2           # 2 consecutive bbox frames to trigger approach

decode:
  required_streak: 3
  miss_tolerance: 2       # allow 2 miss frames mid-streak (was 1) for 15 m jitter

mission:
  sweep_alt_m: 15.0
  approach_stair_m: [12.0, 9.0, 7.0]  # detect at 15, descend to 12→9→7 for decode
  grid_overlap: 0.35      # 35% overlap (was 0.3) for 15 m ensures no gaps with drift
  edge_margin_m: 2.5
  resweep_alt_m: 10.0     # if first grid at 15 m finds nothing, second grid at 10 m
  search_speed_ms: 2.0    # SLOWER for 15 m (was 2.5) → more frames over target
  yaw_steps: 12

blob:
  enabled: true           # last-resort hypotheses — MUST be true for 15 m
  min_support: 2
  cluster_radius_m: 2.5
```

For 10 m mission, use:
```yaml
flight: {max_alt_m: 10.0}
mission:
  sweep_alt_m: 10.0
  approach_stair_m: [8.0, 6.0, 4.0]
  resweep_alt_m: 7.0
  search_speed_ms: 2.2
detector: {conf_thr: 0.30}  # slightly higher than 15 m
```

### B. Search pattern — cover polygon with FOV

Use UI polygon → FOV coverage:

- At 15 m bottom: footprint 35.7×23.0 m, spacing = min(w,h)*(1-0.35)=23*0.65=15 m. For 100×100 m polygon, ~7 rows, ~14 legs.
- At 10 m: footprint 23.8×15.3 m, spacing 10 m, ~10 rows.
- Ensure `grid_overlap` 0.35 to handle wind drift and GPS error.
- In UI, draw polygon, set max alt 15, cam bottom, overlap 0.35, click Show FOV coverage — verify legs cover area with footprint rects overlapping.

### C. Detection pipeline improvements

1. **Tiled inference + classical fallback:**
   - Hailo YOLO on 3×3 tiles → BBox
   - If Hailo fails, classical `cv2.QRCodeDetector` multi + pyzbar on upscaled tiles (decoder.py already does 2×2 tiles upscaled to 700 px min side)
   - Both run every frame; first payload wins.

2. **Decode stage:**
   - ROI crop with 35% margin, upscale to ≥360 px min side, then CLAHE + blur variants
   - Full-frame decode every 3rd frame (catches large QR)
   - 2×2 overlapping tiles upscaled to 700 px for far QR

3. **Consensus:**
   - `required_streak: 3` matching payloads, `miss_tolerance: 2` → needs 3 decodes within 5 frames, robust to jitter at 15 m.

4. **Approach:**
   - On cue (2 consecutive bbox), FSM goes TARGET_FOUND → DESCEND stair 12→9→7 m while keeping QR centered.
   - At lower alt, GSD improves: at 7 m bottom footprint 16.7×10.7 m, A3 ≈ 72 px, decode easy.

5. **Blob fallback:**
   - If grid finishes with no cue, blob scores ground-stable hypotheses from passive observations (cluster radius 2.5 m, min_support 2) and visits top 3. This catches QR missed during fast grid.

### D. Camera tuning for A3 at 15 m

**Bottom IMX477 B (fixed focus):**
- Focus set to infinity — verify with `tools/cam_probe.py --cam bottom --focus infinity --distance 15` — check sharpness at 15 m. If blurry, adjust lens ring (factory may be at 2 m).
- IR-CUT: day mode (GPIO off) — leave in for competition.
- Exposure: auto, but cap 8000 us, gain 1.5 — avoid motion blur at 2 m/s.
- Resolution: 2028×1520 @40 fps for search, switch to 4056×3040 @10 fps for approach decode (config `single_cam_role: bottom`).

**Front Pi Cam3 (AF):**
- AF continuous, PDAF — ensures sharp at varying approach distances.
- Use front for final decode at 5-7 m if bottom fails.

### E. Training / HEF recompilation for A3

If your HEF was trained on generic QR close-up, retrain:

```bash
# 1. Collect dataset: A3 at 5,10,15 m, various angles, lighting, blur
# Use Pi to capture: python3 tools/capture_dataset.py --cam bottom --alt 15 --qr A3 --out dataset/a3_15m/

# 2. Add sim frames: Gazebo QR 297×420 mm at 15 m altitude
python3 tools/make_demo_frames.py --qr-size 0.297 --alt 15 --out dataset/sim_a3_15m/

# 3. Label with Roboflow or CVAT, export YOLO format 1 class 'qr'

# 4. Train YOLOv8n
yolo detect train model=yolov8n.pt data=dataset/a3.yaml epochs=100 imgsz=640 batch=16

# 5. Export ONNX
yolo export model=runs/detect/train/weights/best.pt format=onnx imgsz=640

# 6. Compile to HEF with calibration set including far A3
hailo parser onnx --hw-arch hailo8 runs/detect/train/weights/best.onnx
hailo optimizer --har best.har --calib-set dataset/calib_1024/ --hw-arch hailo8
hailo compiler --har best.har --hw-arch hailo8 --output-dir models/
# Result: models/qr_yolov8n.hef with NMS

# 7. Bench
python3 tools/bench_hef.py --hef models/qr_yolov8n.hef --images dataset/a3_15m/ --conf 0.25
```

Target: >0.85 recall on A3 at 15 m tiles at conf 0.25.

### F. How to verify detection every time — test ladder

1. **Laptop bench (no Pi):**
   ```bash
   python3 tools/make_demo_frames.py --qr-size 0.297 --alt 15
   python3 main.py --config config.laptop.yaml  # uses webcam + classical
   # Should detect A3 at 15 m sim with tiling
   ```

2. **Pi bench (no props):**
   - Pi on desk, A3 print on floor 3 m away, then 5,10 m (use ladder)
   - `python3 tools/cam_probe.py --cam bottom --show-bbox` — check bbox at each distance
   - Tune `conf_thr` until detection at 10 m every time, then test 15 m.

3. **Gazebo SITL:**
   - Gazebo world with A3 0.297×0.42 m box at origin, drone at 15 m, bottom cam
   - Mission should detect, descend, decode, RTL.

4. **Field (real flight):**
   - Fly manual at 15 m over A3, log video, run offline decode: `python3 tools/offline_decode.py --video logs/bottom_15m.h264 --conf 0.25`
   - If offline works but online fails, lower `conf_thr` or increase `tile_grid` to 4×4 (more compute but more recall).

### G. Why detection still fails sometimes — checklist

- [ ] Bottom cam out of focus at infinity? Run cam_probe.
- [ ] HEF calibration set missing far A3? Recompile.
- [ ] `tile_bottom: false`? Must be true.
- [ ] `conf_thr` 0.35 too high for 15 m? Lower to 0.25.
- [ ] Search speed 3+ m/s too fast? Lower to 2.0.
- [ ] `grid_overlap` 0.2 too low? Raise to 0.35.
- [ ] Lighting harsh shadow? Add CLAHE variant already in decoder, but also test at different times.
- [ ] QR print low contrast or glossy reflection? Use matte A3, high contrast, error correction M/Q.
- [ ] Motion blur? Lower exposure, increase gain, or slower speed.

---

## Summary: Make it work every time at 10/15 m for A3

- **Mandatory:** 3×3 tiling, conf_thr 0.25 at 15 m / 0.30 at 10 m, 2028×1520 @40 fps, approach stair, blob fallback, resweep at 10 m.
- **HEF:** Calibrate with far A3, keep 640 input, allow FP16 for small object layers.
- **Camera:** Bottom fixed focus at infinity, IR-CUT day, exposure 8000 us.
- **Search:** Overlap 0.35, speed 2.0 m/s, polygon FOV coverage ensures no gaps.
- **Test ladder:** Sim → Pi desk at 3/5/10 m → Gazebo 15 m → field manual 15 m log → auto mission.

With this, A3 at 10 m should be >95% detection, at 15 m >85% first grid, >95% with resweep at 10 m.
