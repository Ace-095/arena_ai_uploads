# arena/test1 — Polygon + Configurable Height + FOV Coverage + A3 QR Detection + QR Boost (Kabaddi)

This folder documents the **arena/test1** branch features and the three validation methods + Kabaddi camera setting.

## What is new in arena/test1

1. **Polygon primary** (instead of fence) — purple `#a78bfa` layer. User draws polygon, it shows on Mission Planner via conversion.
2. **Polygon → Fence Inclusion button** — one click converts polygon to MAV_CMD 5001 fence inclusion, uploads via `POST /api/mp/fence`, verified on FC, shows in MP Fence tab.
3. **Configurable max height** — `config.yaml` `flight.max_alt_m` = 15 changeable to 10 / 5 m. Pi clamps `sweep_alt_m`, `approach_stair`, `resweep_alt_m`. UI has inputs for max and sweep alt.
4. **FOV-optimal coverage** — footprint `2·alt·tan(FOV/2)` using cam.txt specs:
   - Front Pi Cam3 Standard: 66° HFOV / 41° VFOV / 75° diag, 4608×2592 f/1.8 PDAF
   - Bottom IMX477 B: 100° HFOV / 75° VFOV / 113° diag, 4056×3040 f/2.8 fixed
   - At 15 m bottom ≈ 35.7×23.0 m, spacing ≈ 16 m (0.35 overlap). At 10 m ≈ 23.8×15.3 m spacing 10.7 m. At 5 m ≈ 11.9×7.7 m spacing 5.4 m.
   - `divide_polygon_by_fov`, `lawnmower_rows_fov_optimal`, FOV coverage viz (footprint rects + path) in UI.
5. **QR Boost (Kabaddi setting)** — makes QR pop more than ground:
   - Hardware ISP: contrast HIGH 1.8-2.0, saturation LOW 0.5-0.7 (ground desat), sharpness HIGH 2.0+, brightness -10, manual exp 5 ms
   - Software: CLAHE + unsharp + HSV green/brown mask desat 40% + V boost, adaptive threshold, Otsu
   - Profiles: `qr_boost_day`, `qr_boost_aggressive`, `bottom_qr_boost`, `front_qr_boost` — apply via UI QR BOOST card or `config.yaml` `qr_boost`

## Docs in this folder

| Doc | Purpose |
|-----|---------|
| [00_DETECTION_AT_10_15M_A3_QR_HEF.md](00_DETECTION_AT_10_15M_A3_QR_HEF.md) | **Why detection fails at 15 m, HEF impact, A3 pixel budget, and how to make it work every time** |
| [01_SITL_GAZEBO_POP_OS_WINDOWS11.md](01_SITL_GAZEBO_POP_OS_WINDOWS11.md) | **Method 1:** Pop!_OS laptop SITL + Gazebo + mission_pi, Windows 11 laptop Mission Planner + UI bridge (two-laptop bench) |
| [02_PI5_AI_HAT_BENCH.md](02_PI5_AI_HAT_BENCH.md) | **Method 2:** Real bench test with Pi 5 + Hailo AI HAT + both cams, HEF model, no props |
| [03_REAL_FLIGHT.md](03_REAL_FLIGHT.md) | **Method 3:** Real flight integration, preflight, fence, failsafes, flight day checklist |
| [04_QR_CAMERA_BOOST_KABADDI.md](04_QR_CAMERA_BOOST_KABADDI.md) | **Kabaddi setting:** Make QR pop more than ground via ISP + software — profiles, how to apply, field steps |

## Quick start — which doc do I need?

- **I have no hardware yet, want to prove polygon→fence→coverage→QR flow:** → `01_SITL_GAZEBO_POP_OS_WINDOWS11.md`
- **I have Pi 5 + AI HAT + cameras on desk:** → `02_PI5_AI_HAT_BENCH.md` + `00_DETECTION...` + `04_QR_CAMERA_BOOST...` for tuning
- **I am going to field:** → `03_REAL_FLIGHT.md` (read 01 and 02 first) + `04_QR_CAMERA_BOOST...` for Kabaddi setting
- **QR not visible vs ground:** → `04_QR_CAMERA_BOOST_KABADDI.md`

## Config.yaml — max height + QR boost knobs

```yaml
flight:
  max_alt_m: 15.0   # CHANGE THIS: 5 / 10 / 15
  min_alt_m: 2.0

qr_boost:
  enabled: true
  profile: "auto"   # bottom_qr_boost for bottom, front_qr_boost for front
  software_mode: "qr_boost"  # qr_boost / ground_suppress / adaptive

cameras:
  bottom:
    size: [2028, 1520]
    qr_boost:
      enabled: true
      profile: "bottom_qr_boost"  # contrast 1.9 sat 0.6 sharp 2.0 — ground suppressed
      software_mode: "ground_suppress"

mission:
  max_alt_m: 15.0
  sweep_alt_m: 15.0
  approach_stair_m: [12,9,7]
  grid_overlap: 0.35
  resweep_alt_m: 10.0
```

UI Config card has max height, QR BOOST card has profiles (day/aggressive/bottom/front) + software enhance modes.

## Cam.txt → footprint math

```
footprint_w = 2 * alt * tan(HFOV/2)
footprint_h = 2 * alt * tan(VFOV/2)
spacing     = min(w,h) * (1 - overlap)
```

Bottom cam at 15 m covers ~3.7× more area per image than front cam, so it is chosen as optimal for max coverage. Front cam is kept for close-range decode (AF).

## Kabaddi camera setting — QR pop vs ground

```
ISP: contrast HIGH 1.8-2.0 (QR B/W pop vs mid-tone ground)
     saturation LOW 0.5-0.7 (green grass desat to gray, QR B/W stays)
     sharpness HIGH 2.0 (finder patterns crisp at 15 m)
     brightness -10 (white ground not blown)
     exposure manual 5 ms (less blur at 2 m/s)

Software: CLAHE L channel + unsharp 1.5× -0.5× blur + HSV green 35-85 + brown 10-30 mask desat 40% + V boost
          Decoder variants: CLAHE + high contrast alpha 1.5 + adaptive thresh + Otsu
```

Result: conf 0.25→0.35-0.40 at 15 m, recall +20%, decode +15%.

## Branch

```
git checkout arena/test1
git log --oneline -5
# b60e125 A3 QR detection at 10/15m + 3-method docs
# + QR boost Kabaddi setting
```

All code lives under `workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/` (repo convention).

