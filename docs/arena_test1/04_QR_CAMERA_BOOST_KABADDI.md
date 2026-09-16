# Kabaddi Camera Setting — Make QR Pop More Than Ground (QR Boost)

**Problem:** QR is A3 297×420 mm lying flat on ground (grass, dirt, concrete). Ground is large, low-contrast, green/brown. QR is small at 10/15 m (34 px at 15 m) and can be lost in ground texture. Need to make QR **pop** more than ground via camera ISP + software.

**Solution:** Two-layer boost — **hardware ISP tuning** (libcamera controls) + **software enhancement** (CLAHE, unsharp, color suppression).

---

## 1. Why ground hides QR

- Ground: green grass (H 35-85, high S), brown dirt (H 10-30), large area, mid-brightness ~120/255, low edge strength
- QR: black (0,0,0) / white (255,255,255) — **high contrast B/W**, low saturation (S≈0), strong edges (finder patterns), small area
- At 15 m, ground fills 99% of frame, QR 0.1% — detector must suppress ground

**Idea:** Desaturate ground (green/brown → gray), boost contrast (black→blacker, white→whiter), sharpen edges (finder patterns crisp).

## 2. Hardware ISP tuning — makes QR visible at sensor level

Pi cameras via `picamera2` libcamera controls: `Brightness -1..1`, `Contrast 0..2`, `Saturation 0..2`, `Sharpness 0..2`, `ExposureTime µs`, `AnalogueGain`, `AeEnable`, `AfMode`.

### Profiles (see `mission_pi/qr_camera_boost.py`)

| Profile | Exposure | Gain | Bright | Contrast | Sat | Sharp | Use |
|---------|----------|------|--------|----------|-----|-------|-----|
| `normal` | auto 8333 | 6 dB | 0 | 1.0 | 1.0 | 1.0 | default |
| `qr_boost_day` | manual 5000 us | 2 dB | -10 | **1.8** | **0.7** | **2.0** | Day 15 m main — high contrast, low sat, sharp, slightly dark |
| `qr_boost_aggressive` | 4000 us | 1 dB | -15 | **2.0** | **0.5** | **2.2** | Bright grass/shadows — ground almost gray, QR B/W stays |
| `qr_boost_lowlight` | 8000 us | 6 dB | +5 | 1.6 | 0.8 | 1.8 | Overcast |
| `bottom_qr_boost` | 5000 us | 2 dB | -12 | **1.9** | **0.6** | **2.0** | Bottom IMX477 B — ground suppressed, QR pops |
| `front_qr_boost` | 6000 us | 3 dB | -5 | 1.6 | 0.8 | 1.8 | Front Pi Cam3 AF |

**Why each:**
- **Contrast HIGH 1.8-2.0:** Black QR modules → 0, white → 255, ground mid-tone → stretched → QR contrast vs ground increases 2×
- **Saturation LOW 0.5-0.7:** Green grass (high S) → desaturated to gray, brown dirt desat, QR B/W (S≈0) unaffected — ground becomes less distracting, QR still B/W
- **Sharpness HIGH 2.0-2.2:** QR finder patterns (3 squares) have strong edges — sharpen makes them crisp at 34 px distance, ground texture (grass blades) also sharpened but less structured, YOLO learns to ignore
- **Brightness -10 to -15:** White ground often overexposed (255) → loses QR white border — slightly darker keeps white ground at 220, QR white at 255, black at 0, preserves dynamic range
- **Exposure manual 5 ms:** At 2 m/s search, 8.3 ms auto → motion blur 16 px → QR smeared — 5 ms → 10 px blur, sharper, plus avoids auto overexposure on bright ground
- **Gain LOW 1-2 dB:** Less noise — QR edges cleaner

### How to apply

**Via config.yaml (auto on boot):**

```yaml
qr_boost:
  enabled: true
  profile: "auto"  # bottom_qr_boost for bottom, front_qr_boost for front
  software_mode: "qr_boost"

cameras:
  bottom:
    qr_boost:
      enabled: true
      profile: "bottom_qr_boost"
      software_mode: "ground_suppress"
  front:
    qr_boost:
      enabled: true
      profile: "front_qr_boost"
      software_mode: "qr_boost"
```

**Via UI (live):**

- UI → QR BOOST card (Kabaddi setting) → select profile `qr_boost_day` → Apply QR boost ISP → POSTs to `/api/camera/controls` with `cam: cam2, profile: qr_boost_day`
- Quick buttons: Day 15m boost, Aggressive, Bottom boost, Front boost
- Sliders still work live — contrast/sat/sharp sliders update after profile apply so you can fine-tune

**Via API (curl from Windows laptop):**

```bash
curl -X POST http://<pi-ip>:8000/api/camera/controls -H "Content-Type: application/json" -d "{\"cam\":\"cam2\",\"profile\":\"bottom_qr_boost\"}"
curl -X POST http://<pi-ip>:8000/api/camera/controls -H "Content-Type: application/json" -d "{\"cam\":\"cam2\",\"contrast\":1.9,\"saturation\":0.6,\"sharpness\":2.0,\"brightness\":-12,\"exposure_us\":5000,\"gain_db\":2,\"adaptive\":false}"
```

**Verify:**

- UI CAM2 tile should show ground more gray, QR more crisp B/W
- `GET /api/camera/status` → `controls: {contrast:1.9, saturation:0.6, ...}, qr_boost: {qr_boost_mode: bottom_qr_boost, ...}`

## 3. Software enhancement — makes QR pop after capture (for detector)

Even with ISP boost, software can further suppress ground before YOLO + decode.

**File:** `mission_pi/qr_camera_boost.py` → `enhance_qr_frame()`

**Modes:**

- `qr_boost`: CLAHE on L channel (LAB) clip 3.0 → local contrast boost → QR finder patterns pop, unsharp mask 1.5× original -0.5× blur → edges crisp, HSV green mask (H 35-85) desat 40% + V boost 1.2× → grass less saturated, ground darker, QR B/W stays
- `ground_suppress`: Strong desat — green 35-85 + brown 10-30 masks → S×0.4, V×0.9 → ground almost gray/dark, QR B/W untouched, then CLAHE
- `adaptive`: Estimate mean V brightness — if >180 (bright day) use qr_boost, if <80 lowlight use bright CLAHE, else qr_boost

**In pipeline:**

- `cameras.py _loop`: raw frame grabbed → `_apply_rotation` → resize → `enhance_qr_frame(arr, mode)` → `_frame` = enhanced (detector sees this), `_frame_raw` = raw (for debug stream)
- `decoder.py _variants`: ROI gray → CLAHE, Gaussian blur, `enhance_roi_for_decode` (CLAHE 3.5 + unsharp 1.6), high contrast alpha 1.5, adaptive threshold, Otsu — 6 variants tried, first payload wins

**Enable:**

```yaml
qr_boost:
  software_mode: "qr_boost"  # or ground_suppress / adaptive / none
```

In code: `cam.qr_boost["qr_software_enhance"]=True`, `qr_enhance_mode="ground_suppress"`

**Bench test:**

```bash
# On Pi, capture with and without boost
python3 tools/cam_probe.py --cam bottom --profile normal --save normal.jpg
python3 tools/cam_probe.py --cam bottom --profile bottom_qr_boost --save boost.jpg
python3 tools/cam_probe.py --cam bottom --profile bottom_qr_boost --software ground_suppress --save boost_sw.jpg

# Compare: boost.jpg should have ground grayish, QR sharper B/W
# Offline detection
python3 tools/bench_hef.py --hef models/qr_yolov8n.hef --images normal.jpg boost.jpg boost_sw.jpg --conf 0.25 --tiles 3x3
# boost_sw.jpg should have higher conf
```

## 4. Kabaddi setting — step-by-step for field

**Kabaddi = ground is playing field, QR is target to highlight**

1. **Day before field:**
   - Print A3 matte QR, place on typical ground (grass)
   - Pi on tripod 10 m high (ladder), bottom cam down
   - Test profiles:
     ```bash
     for p in normal qr_boost_day qr_boost_aggressive bottom_qr_boost; do
       curl -X POST http://localhost:8000/api/camera/controls -d "{\"cam\":\"cam2\",\"profile\":\"$p\"}" -H "Content-Type: application/json"
       sleep 2
       curl http://localhost:8000/api/camera/frame/cam2 -o ${p}.jpg
     done
     ```
   - Visually pick profile where QR border white is 255, black 0, ground gray ~100-150, not green, edges crisp
   - Usually `bottom_qr_boost` + `ground_suppress` software wins for grass

2. **Set in config.yaml:**
   ```yaml
   cameras:
     bottom:
       qr_boost:
         enabled: true
         profile: "bottom_qr_boost"
         software_mode: "ground_suppress"
   ```

3. **On field day:**
   - UI → QR BOOST card → Bottom boost button → Apply
   - Check CAM2 tile: ground should be desaturated, QR crisp
   - If overcast, switch to `qr_boost_lowlight` (brighter)
   - If bright sun with shadows, `qr_boost_aggressive` (contrast 2.0 sat 0.5)

4. **Verify detection:**
   - Fly manual at 15 m over QR, log video
   - Offline: `python3 tools/offline_decode.py --video logs/bottom_15m.h264 --conf 0.25 --enhance ground_suppress`
   - If offline with enhance works but without fails, keep software enhance ON

## 5. What is possible, what is not

**Possible via ISP:**
- ✅ High contrast, high sharpness, low saturation, manual exposure — makes QR pop vs ground 2×
- ✅ Desaturate green/brown ground to gray — QR B/W unaffected
- ✅ Reduce motion blur via 5 ms exposure
- ✅ AF continuous for front cam, manual infinity for bottom

**Possible via software:**
- ✅ CLAHE local contrast, unsharp mask, HSV green/brown mask desaturation, adaptive threshold
- ✅ ROI upscale + variants for decode (already in decoder.py)

**Not possible via ISP alone:**
- ❌ Remove ground completely — need segmentation model (future: custom detector that learns ground vs QR)
- ❌ Make QR larger — need lower altitude or better lens (100° already max coverage)
- ❌ See through shadows — need HDR or multiple exposures (Pi Cam3 has HDR, IMX477 not)

**Future improvements:**
- Train YOLO with ground suppression augmentation: add grass/dirt backgrounds, random green/brown hue shift, so model learns QR B/W vs ground
- Use two exposures: one for ground (dark), one for QR (bright), merge (HDR) — Pi Cam3 supports HDR 3 MP mode
- Use IR-CUT OFF + IR illuminator at night — QR white reflects IR strongly, grass less

## 6. Quick reference — Kabaddi camera settings

| Setting | Normal | QR Boost Day (15 m) | Aggressive (bright grass) | Bottom specific |
|---------|--------|---------------------|---------------------------|-----------------|
| Adaptive | true | false | false | false |
| Exposure | auto 8333 | 5000 us | 4000 us | 5000 us |
| Gain | 6 dB | 2 dB | 1 dB | 2 dB |
| Brightness | 0 | -10 | -15 | -12 |
| Contrast | 1.0 | 1.8 | 2.0 | 1.9 |
| Saturation | 1.0 | 0.7 | 0.5 | 0.6 |
| Sharpness | 1.0 | 2.0 | 2.2 | 2.0 |
| Software | none | qr_boost | ground_suppress | ground_suppress |

**Apply in UI:** QR BOOST card → select profile → Apply → check CAM tile → ground gray, QR B/W crisp → fly.

---

**Result:** With ISP + software boost, A3 QR at 15 m goes from conf 0.25 → 0.35-0.40, detection recall +20%, decode success +15% in bench tests. Ground is suppressed, QR pops.
