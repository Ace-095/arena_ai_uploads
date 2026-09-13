# OUR MISSION — Working Spec (user-stated, kept for overall view)

> Source: user brief, 2026-09-12. This is OUR build spec — separate from the hehe repo analysis.
> Two workstreams: **(1) UI** — built first. **(2) Pi part.** Nothing is built until the user says go on each piece.

## 1. Mission (one paragraph)

A drone flies over a bounded search area. Somewhere in it sits a **QR code printed on an A3-size board** (297×420mm). The drone must **autonomously detect the board** while flying the area at **up to 15 m altitude**, then **fly to it, lower altitude, decode the QR**, and **send the payload back**. Search area: **max 50×50 m; working assumption ~25×25 m** — "can't take risk", so the plan must hold for the worst case.

## 2. Hardware (as stated)

- **Cam 1 (now, confirmed): Raspberry Pi Camera Module 3** (IMX708).
- **Cam 2 (maybe, not decided): Arducam IMX477 12MP** (HQ-class).
- Single-cam (Pi Cam 3 only) is the design baseline; the plan must work with one camera.
- Flight: quad with ArduPilot/Pixhawk (reuse hehe's MAVLink stack), Pi on board.

## 3. Flight plan (user's)

1. Drone flies the search area at **≤ 15 m** (anywhere in the area).
2. At 15 m it **object-detects** the QR board (detect, NOT decode — user's explicit expectation).
3. On detection → fly to the target → **descend** until the QR decodes.
4. **Transmit the decoded payload back** (to UI / ground station).

## 4. Camera research — FOV & geometry (done 2026-09-12)

### 4.1 Specs (researched, not guessed)

**Raspberry Pi Camera Module 3 — IMX708 (Sony)**
- 11.9 MP, 4608×2592 (16:9), 1/2.43″ (7.4 mm diag), 1.4 µm pixels, PDAF 10 cm–∞
- **Standard lens: 4.74 mm, F1.8 → FOV 75°(D) × 66°(H) × 41°(V)** ← our camera
- Wide-angle variant: 2.75 mm, F2.2 → 120°(D) × 102°(H) × 67°(V)
- Video modes: 1080p50, 720p100, 480p120 (2304×1296@30–56 also listed for the family)
- Since the sensor is 16:9, 1080p/720p are pure downscales → **HFOV stays 66° at all 16:9 video res**

**Arducam IMX477 (Sony, "HQ camera" family)**
- 12.3 MP, 4056×3040 (4:3), 1/2.3″ (7.9 mm diag), 1.55 µm pixels
- Lens variants (this is the choice that matters):
  - **M12 3.9 mm F2.8 → 75°(H)** — the *motorized-focus* version (software focus control, matches the hehe repo's decode-cam role)
  - **CS-mount 6 mm → 65°(H)**
  - **CS-mount 16 mm (C-mount lens + C-CS adapter, manual focus) → ≈22°(H) computed: 2·atan(3.1435/16) = 22.2°H, 16.7°V**
  - (100°/87°H autofocus variant exists for Jetson — not relevant for Pi)

### 4.2 The formulas (so our calcs can be cross-checked)

```
footprint width  W = 2·h·tan(HFOV/2)                      [ground metres covered at altitude h]
GSD (px per m)   = W_px / (2·h·tan(HFOV_crop/2))          [at a given image resolution]
decode limit     px_needed = QR_modules × px_per_module
                 GSD_required = px_needed / QR_width_m
                 h_decode_max = W_px / (GSD_required · 2·tan(HFOV_crop/2))
```
Rules of thumb used: **4 px/module = practical decode minimum, 6 px/module = comfortable** (cv2/pyzbar; WeChatQRCode from opencv-contrib is stronger on small codes).

### 4.3 Numbers — Pi Cam 3 standard lens (66° H)

Ground footprint (nadir): 15 m → **19.5 m × 11.2 m**; 10 m → 13.0 × 7.5; 5 m → 6.5 × 3.7.

| Resolution | GSD @15 m | GSD @10 m | GSD @5 m | A3 board px @15 m |
|---|---|---|---|---|
| 2304×1296 | 118 px/m | 177 | 355 | 35 × 50 px |
| **1920×1080** | **98 px/m** | 148 | 296 | **29 × 41 px** |
| 1280×720 | 66 px/m | 99 | 197 | 19.5 × 28 px |

- **Detection at 15 m: YES in 1080p** (29–50 px target; high-contrast white A3 board with black QR pattern is distinctive). In 720p at 15 m it's ~20 px → borderline. → **use 1080p (or 2304×1296) for the search sweep.**
- Detection ceiling (20-px minimum target, 1080p): h ≈ 22 m. So 15 m is a comfortable margin.

### 4.4 Numbers — decode altitude (assumes QR region ≈ 0.30 m on the board, numeric payload → QR v1 = 21×21 modules)

| Setup (1080p) | 4 px/mod → h_decode | 6 px/mod → h_decode |
|---|---|---|
| **Pi Cam 3 (4.74 mm)** | **5.3 m** | **3.5 m** |
| Pi Cam 3 wide (2.75 mm) | 2.8 m | 1.9 m ← wide lens is strictly worse for this mission |
| IMX477 3.9 mm motorized | 4.5 m | 3.0 m |
| **IMX477 16 mm** | **17.5 m** | **11.6 m** (4K stills: 37 m / 24.6 m) |

Sensitivity: if the QR is printed larger (0.42 m, full long edge) decode alt rises ~40% (Pi Cam 3 → ~7.4 m @4px); if the payload pushes QR to v3 (29 modules) decode alt drops ~27% (~3.8 m @4px, 1080p).

**Cross-check with the hehe repo's claim** ("6mm lens needs ~9m for 4px/module, 16mm ~25m"): their 16mm ≈ 25 m matches full-res 4K math (24.6 m @6px); our 1080p numbers are ~half because 1080p bins 4:3 sensor pixels 2.1×. Repo's "6mm" was an estimate — the real Pi Cam 3 is 4.74 mm.

### 4.5 Conclusions for our plan

1. **Single Pi Cam 3 works for the whole mission**: detect at 10–15 m (1080p) → approach → closed-loop descent, expect decode at **~4–5 m** → floor ~2 m. This is exactly the hehe pattern (descend in 1.5 m steps, check consensus each step, stop on decode) — we keep it.
2. **If we ever add the IMX477, the 16 mm CS lens is the game-changer** (decode from 12–17 m even in 1080p) — but it's manual-focus and narrows search FOV to 22°, so it'd be a second decode-only cam. The 3.9 mm motorized version buys little over Pi Cam 3.
3. **Do NOT buy the 102° wide Pi Cam 3** for this mission — detection ceiling drops to ~11.5 m and decode alt to ~2.8 m.
4. Search geometry: at 15 m each pass sees a 19.5 m-wide swath → 25×25 m area ≈ 2–3 lawnmower rows; expanding-square from centroid also works.
5. Motion blur: sweeping 1.5 m/s at 15 m in 1080p ≈ 5 px blur on a 29 px target → **hold/station-hover over the detection before descending** (hehe's descent already holds XY).

## 5. Open questions (ask user when relevant)

- [x] ~~QR payload format~~ → **CONFIRMED 2026-09-12: 2-digit number** (competition organizers informed us) → QR version 1, 21×21 modules → decode math in §4.4 holds.
- [ ] Exact search-area shape: 50×50 m square vs 50 m radius? Working assumption 25×25 m square.
- [ ] QR print size on the A3 board (assumed ~0.30 m).
- [x] ~~2nd camera in scope?~~ → "very later" — v1 = single Pi Cam 3 only.
- [x] ~~Board orientation/contrast~~ → flat ground (flat plane competition venue), white board with printed QR (probably QR only, nothing else).
- [ ] Exact QR print size on the A3 board (assumed ~0.30 m — still to confirm).

## 6. Working rules (agreed 2026-09-12)

- **Never assume hehe works.** Its code = proven-in-theirs-sim candidates, not gospel. Everything
  is re-verified on OUR bench → SITL → Gazebo after development (user: "assuming everything works
  is a dumb decision"). Reference attachments (e.g. `qrfinal_gpt4.txt`) = ideas only, NOT
  guaranteed-working code.
- UI is built first; UI runs against the documented **API contract** so the Pi (workstream 2)
  implements to the same contract, and a mock backend demos the UI before the Pi exists.

## 7. Build order (agreed)

1. **UI first** (workstream 1) — awaiting user's detailed prompt.
2. Pi part (workstream 2) — later.
3. Reuse-from-hehe candidates: MAVLink bus/fence modules, telemetry hub, QR consensus pipeline, closed-loop descent, FSM skeleton, cockpit UI patterns, mock-FC test rig (see hehe-analysis.md §15).
