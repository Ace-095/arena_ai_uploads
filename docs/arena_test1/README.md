# arena/test1 — Polygon + Configurable Height + FOV Coverage + A3 QR Detection

This folder documents the **arena/test1** branch features and the three validation methods.

## What is new in arena/test1

1. **Polygon primary** (instead of fence) — purple `#a78bfa` layer. User draws polygon, it shows on Mission Planner via conversion.
2. **Polygon → Fence Inclusion button** — one click converts polygon to MAV_CMD 5001 fence inclusion, uploads via `POST /api/mp/fence`, verified on FC, shows in MP Fence tab.
3. **Configurable max height** — `config.yaml` `flight.max_alt_m` = 15 changeable to 10 / 5 m. Pi clamps `sweep_alt_m`, `approach_stair`, `resweep_alt_m`. UI has inputs for max and sweep alt.
4. **FOV-optimal coverage** — footprint `2·alt·tan(FOV/2)` using cam.txt specs:
   - Front Pi Cam3 Standard: 66° HFOV / 41° VFOV / 75° diag, 4608×2592 f/1.8 PDAF
   - Bottom IMX477 B: 100° HFOV / 75° VFOV / 113° diag, 4056×3040 f/2.8 fixed
   - At 15 m bottom ≈ 35.7×23.0 m, spacing ≈ 16 m (0.3 overlap). At 10 m ≈ 23.8×15.3 m spacing 10.7 m. At 5 m ≈ 11.9×7.7 m spacing 5.4 m.
   - `divide_polygon_by_fov`, `lawnmower_rows_fov_optimal`, FOV coverage viz (footprint rects + path) in UI.

## Docs in this folder

| Doc | Purpose |
|-----|---------|
| [00_DETECTION_AT_10_15M_A3_QR_HEF.md](00_DETECTION_AT_10_15M_A3_QR_HEF.md) | **Why detection fails at 15 m, HEF impact, A3 pixel budget, and how to make it work every time** |
| [01_SITL_GAZEBO_POP_OS_WINDOWS11.md](01_SITL_GAZEBO_POP_OS_WINDOWS11.md) | **Method 1:** Pop!_OS laptop SITL + Gazebo + mission_pi, Windows 11 laptop Mission Planner + UI bridge (two-laptop bench) |
| [02_PI5_AI_HAT_BENCH.md](02_PI5_AI_HAT_BENCH.md) | **Method 2:** Real bench test with Pi 5 + Hailo AI HAT + both cams, HEF model, no props |
| [03_REAL_FLIGHT.md](03_REAL_FLIGHT.md) | **Method 3:** Real flight integration, preflight, fence, failsafes, flight day checklist |

## Quick start — which doc do I need?

- **I have no hardware yet, want to prove polygon→fence→coverage→QR flow:** → `01_SITL_GAZEBO_POP_OS_WINDOWS11.md`
- **I have Pi 5 + AI HAT + cameras on desk:** → `02_PI5_AI_HAT_BENCH.md` + `00_DETECTION...` for tuning
- **I am going to field:** → `03_REAL_FLIGHT.md` (read 01 and 02 first)

## Config.yaml — max height knob

```yaml
flight:
  max_alt_m: 15.0   # CHANGE THIS: 5 / 10 / 15
  min_alt_m: 2.0

mission:
  max_alt_m: 15.0
  sweep_alt_m: 15.0          # clamped to flight.max_alt_m
  approach_stair_m: [12,9,7] # also clamped
  resweep_alt_m: 10.0        # second grid at 10 m if first fails
```

UI Config card has same knob — changing it calls `map.setMaxAlt()` and reclamps sweep alt, redraws FOV coverage.

## Cam.txt → footprint math

```
footprint_w = 2 * alt * tan(HFOV/2)
footprint_h = 2 * alt * tan(VFOV/2)
spacing     = min(w,h) * (1 - overlap)   # 0.3 overlap = 70% new area per row
```

Bottom cam at 15 m covers ~3.7× more area per image than front cam, so it is chosen as optimal for max coverage. Front cam is kept for close-range decode (AF).

## Branch

```
git checkout arena/test1
git log --oneline -3
# e41cffa polygon + fence inclusion + configurable height + FOV coverage
```

All code lives under `workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/` (repo convention).
