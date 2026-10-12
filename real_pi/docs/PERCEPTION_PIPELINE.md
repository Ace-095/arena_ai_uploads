# Perception layer: temporal tracking + a perception state machine

**Scope:** `real_pi/mission_pi/` — the real-flight stack (Pi 5 + Hailo AI HAT +
Pixhawk). Branch `arena/1c4bb365-arena-ai-uploads` (parent commit is identical
to `arena/01a0af20-arena-ai-uploads`, i.e. the requested `af20`).

**What this change is:** an *additive, observation-only* perception layer.
`mission.py` is **not modified** — not one line of the flight path. The layer
observes the frames, boxes and payloads the mission already computes and turns
them into a stabilised, structured result that is readable at
`GET /api/perception`. It never commands the vehicle, and it returns every
value it observes unchanged.

---

## A. Repository inspection report

### What was already there (and was reused, not rewritten)

| Area | File | State found |
|---|---|---|
| Hailo inference | `detector.py::HailoYolo8Detector` | **Complete.** Not a stub. |
| Hailo output decode | `hailo_decode.py` (465 lines) | **Complete.** Handles HailoRT 4.23 `hef.create_configure_params(StreamInterface.PCIe)` and 4 output layouts; NMS tensor `(1,5,100)` read as normalised `y_min,x_min,y_max,x_max,score`. |
| Detector ladder | `detector.get_detector` | `auto` = HEF → ONNX → `.pt` → classical cv2+pyzbar. |
| Models | `models/qr_yolov8n.{hef,onnx,pt}` | 1-class QR YOLOv8n, fine-tuned on synthetic A3. HEF compiled for HailoRT 4.23. |
| QR decoding | `decoder.decode_frame` | ROI margin → boost variants → upscale → full frame → 2×2 tiles. |
| Box gating | `qr_filter.py` | conf / min-side / aspect gate + the window-grill regression fix. |
| Payload consensus | `consensus.py` | N-frame agreement with miss tolerance. |
| Flight FSM | `mission.py` | BOOT, WAIT_LINK, SNAPSHOT, WAIT_TRIGGER, TAKEOVER, SWEEP_YAW, SWEEP_GRID, FALLBACK, TRACK, APPROACH, TRANSMIT, DONE, FAILSAFE. |
| Camera stack | `cameras.py` (1726 lines) | CSI + USB auto-detect, watchdog, ISP boost profiles, autofocus/VCM. |
| Tests | `tests/test_*.py` (40 files) | One per feature, run directly by `tools/run_tests.sh`. |

### What was genuinely missing

1. **No temporal tracking of any kind.** Grep for `track`/`kalman`/`temporal`
   across the tree returns only mode-tracking and `_track_and_approach` (flight
   steering to a cue). Nothing associated a box in frame *N* with the same
   object in frame *N+1*. Consequence: a jittering box moved the decoder ROI
   every frame, and one blurred frame was indistinguishable from "the target
   is gone".
2. **No perception-level state machine.** The brief's
   `SEARCHING → CANDIDATE_DETECTED → … → RESULT_REPORTED` appeared nowhere. The
   existing FSM answers "where is the drone in the mission", not "how much do
   we trust what we are seeing".
3. **No metric harness.** `tools/qr_bench.py` printed boxes, payloads and
   milliseconds. There was no precision, recall, false-positive rate, recovery
   measurement, CPU/RAM/temperature reporting, or stated acceptance criteria.

### Files inspected (read-only)

`real_pi/README.md`, `docs/{01,02,03}_*.md`, `docs/README.md`,
`real_pi/docs/{CAMERA_AND_PIXHAWK_SETUP,qr-resolution-and-command-tests,detection-to-flight-behavior,mark-9-project-report,movement-wire-verification}.md`,
`mission_pi/{config.yaml,main.py,server.py,mission.py,detector.py,hailo_decode.py,decoder.py,qr_filter.py,consensus.py,cameras.py,geo.py,streamm1.py,qr_relay.py,codeinfo.py,requirements.txt}`,
`mission_pi/detectors/{yolo_onnx,yolo_ultralytics}.py`, `mission_pi/models/README.md`,
`mission_pi/training/{README,compile_hef}.md`, `mission_pi/tools/{qr_bench,qr_resolution_probe,test_yolo_webcam_qr,run_tests}.py`,
`mission_pi/tests/*` (sampled: `test_qr_filter.py`, `test_hailo_nesting_fallback.py`, `test_server_first.py`).

---

## B. Hardware / model compatibility report

| Item | Finding | Confidence |
|---|---|---|
| Camera (bottom) | **Arducam UC-517**, IMX477 + motorised VCM focus, M12 3.9 mm f/2.8 | CONFIGURED (`cameras.bottom`) |
| Capture mode | 2028 × 1520 (binned) | CONFIGURED — real sensor mode |
| FOV | 75° H / 60° V / 90° diag | **VENDOR-QUOTED, unverified** — the config itself says to measure it |
| Tuning file | `imx477_af.json`, needs `camera_auto_detect=0`, `[all]`, `dtoverlay=imx477,vcm` | CONFIGURED |
| Camera (front) | IMX708 (Pi Camera Module 3), 4608 × 2592, 66°/41° | CONFIGURED |
| Hailo | AI HAT+ (Hailo-8, 26 TOPS) | ASSUMED from docs; `hailortcli fw-control identify` on the Pi is authoritative |
| HailoRT | HEF compiled with **4.23**; `hailo_decode.py` targets both 4.23 and older APIs | CONFIRMED in code |
| HEF portability | A HEF does **not** move between Hailo-8 / 8L / 10H; `hailo-all` and `hailo-h10-all` cannot co-exist | Documented limit |

**Not verified here:** no Pi, no Hailo device and no camera existed in the
environment where this work was done. Every hardware-dependent number below is
labelled accordingly.

---

## C. Detection feasibility: A3 QR at 10–15 m

Produced by `tools/fov_report.py`, which reads the live `config.yaml` and labels
each input's provenance rather than inventing values.

**Assumption in the table:** the QR symbol fills the A3 short side (297 mm),
25 modules + a 4-module quiet zone on each side (**33 modules total** — the
`px/module` column divides by 33, matching `fov_report.py`); nadir camera; 2 m/s;
2 ms exposure.

| Alt | Ground footprint | mm/px | QR px | px/module | blur px | Verdict |
|---:|---|---:|---:|---:|---:|---|
| 5 m | 7.7 × 5.8 m | 3.8 | 78.5 | 2.38 | 1.06 | DETECT + DECODE (thin margin) |
| 7 m | 10.7 × 8.1 m | 5.3 | 56.1 | 1.70 | 0.76 | DETECT + DECODE (thin margin) |
| 9 m | 13.8 × 10.4 m | 6.8 | 43.6 | 1.32 | 0.59 | DETECT + DECODE (thin margin) |
| **10 m** | 15.3 × 11.6 m | 7.6 | 39.2 | **1.19** | 0.53 | DETECT, **decode marginal** |
| 12 m | 18.4 × 13.9 m | 9.1 | 32.7 | 0.99 | 0.44 | DETECT only |
| **15 m** | 23.0 × 17.3 m | 11.3 | 26.2 | **0.79** | 0.35 | **DETECT only** |

### The two questions, kept apart

* **Detection** needs the QR to be *found* — and "tens of pixels is enough" is
  wrong as stated; see the measured table below.
* **Decoding** needs the QR's *modules* sampled. Below ~1.2 px/module the finder
  patterns do not resolve, and no amount of upscaling invents modules the
  sensor never sampled. **Not met at 10 m, 12 m or 15 m with the configured
  lens and a 297 mm symbol.**

#### Detection: measured, not assumed — SYNTHETIC, not flight evidence

Tested with `tests/test_blob_fallback.py`: a *real* QR panel (v4, 41 modules
incl. quiet zone) rendered on two textured grounds (dark soil, green grass) at
six module scales, plus six decoys of the same footprint (blank white square,
checkerboard, salt-pepper noise, stripes, dash "print", one concentric-square
bullseye). 12 QR cases, 72 decoy cases. Two candidate sources:

| module size | panel | contour scan (global Otsu + 4-gon blobs) | + OpenCV `QRCodeDetector` |
|---:|---:|---|---|
| 4.0 px | 164 px | finds the QR (2 of 2 grounds) | finds it, **decodes** the payload |
| 2.5 px | 102 px | finds it on 1 of 2 grounds | finds it, **decodes** on 1 of 2 |
| 1.6 px | 66 px | **finds it on 0 of 2** | **finds it on 2 of 2** |
| 1.2 px | 49 px | no | no |
| 1.0 px | 41 px | no | no |
| 0.8 px | 33 px | no | no |

Totals: contour-only **3/12** found, **12/72** decoy false positives; with the
detector **5/12** found, **12/72** false positives. Below 1 px/module nothing
measured here finds the QR at all — the finder pattern is under 7 px across,
the resolution floor the established scanners assume.

Why the contour path fails where it does: a high-contrast QR panel destroys a
**global** Otsu threshold (its own black modules drag the threshold down until
the panel is no longer one connected region — measured: 1 contour in the frame
without a QR, 250 with one). It therefore fails hardest exactly where the
last-resort search needs it. `QRCodeDetector` (quirc) is immune, because it
scans for the finder pattern's 1:1:3:1:1 run signature instead of a blob.

A hand-written 1:1:3:1:1 scanner was also prototyped and **rejected on its
measurements**: 11/12 recall but 66/72 decoy false positives, firing on blank
squares, checkers, noise and stripes alike. Delegating to the established
implementation is the deliberate choice, not a shortcut.

Two thresholds are reported because the repo contradicts itself slightly, and
both numbers are useful:

| Threshold | px/module | Source |
|---|---:|---|
| `PX_PER_MODULE_TARGET` | 3.5 | `docs/qr-resolution-and-command-tests.md`: "aim for roughly 3–4 … as a starting margin" |
| `PX_PER_MODULE_EMPIRICAL` | 1.3 | **This repo's own bench table**: a 29-module symbol decoded at 38 px (1.31) and was "at the limit" at 34 px (1.17) |

Quoting only the heuristic would make 10 m look impossible when it demonstrably
is not; quoting only the empirical floor would hide how little margin a
1.2 px/module decode has. Both appear in the tool output.

### What the detector network actually receives (the binding constraint)

The table above counts pixels **in the frame**. The detector does not see the
frame at capture resolution — it letterboxes to `detector.input_size` (640), so a
2028 px frame is shrunk ~3.2× and a 4608 px frame **7.2×** before YOLO runs.
`fov_report.py --cam bottom|front` prints this as a second table:

| Camera | 15 m, full frame | 15 m, 3×3 tiled | Verdict |
|---|---:|---:|---|
| bottom (Arducam, 2028 px) | 8.3 px | 24.8 px | needs tiling |
| front (IMX708, 4608 px) | 9.8 px | 29.3 px | needs tiling |

This is why `detector.tile_bottom: true` is **load-bearing** rather than a
tuning nicety: with full-frame inference at 15 m the panel arrives at the network
as ~8 px and detection collapses regardless of how many sensor pixels you paid
for. Note the counter-intuitive result: the 12 MP front camera is *not* safe at
15 m on raw resolution alone — 4608 px is downscaled harder than 2028 px, so both
land near 10 px until tiling is enabled.

### What would actually change the answer

| Change | px/module at 7 / 10 / 15 m |
|---|---|
| As configured today | 1.70 / 1.19 / 0.79 |
| QR fills the A3 **long** side (420 mm) | 2.40 / 1.68 / 1.12 |
| + shorter payload (v1, 21 modules) | 2.73 / 1.91 / 1.28 |
| + narrower lens (50° HFOV instead of 75°) | 4.50 / 3.15 / 2.10 |
| + 30° mount tilt (foreshortened) | 3.37 / 2.36 / 1.57 |

`*` in the tool marks rows at/above the demonstrated floor. A narrower lens wins
on pixels-per-metre but **shrinks the ground footprint**, so the search grid
re-computes and the mission takes longer — that trade is a mission decision and
is reported, not decided, by the perception tooling.

**Brief's answer, stated plainly:** at 15 m this camera configuration can
*detect* the board but cannot *decode* it. The existing mission design already
accounts for this (`approach_stair_m: [12, 9, 7]`, `resweep_alt_m: 10`) and that
design is load-bearing, not optional. If decoding at 15 m is a hard requirement,
the smallest effective change is a **larger printed symbol** (QR across the
420 mm side) **plus a shorter payload**, or a narrower lens — not a better
detector.

---

## D. Architecture and implementation

```
Arducam UC-517 (CSI) → CameraRig capture thread → frame + ts + seq
        │
        ├─ existing: Mission._worker → detect_tiles(YOLO/Hailo) → qr_filter
        │                                → decode_frame → consensus → cue
        │                                → QR_SEEN / QR relay            [UNCHANGED]
        │
        └─ NEW: perception tap  (perception_tap.py)
                 observes detect_tiles() and decode_frame() return values,
                 returns them UNCHANGED
                          ↓
             PerceptionPipeline (perception.py)
               quality gate (blur on ROI, exposure on scene)
                          ↓
             BoxTracker (tracker.py) — IoU + motion gate, EMA, persistence
                          ↓
             PerceptionFSM
               SEARCHING → CANDIDATE_DETECTED → TARGET_TRACKING → QR_DECODING
                         → PAYLOAD_VALIDATED → RESULT_REPORTED
                          ↓
             PayloadValidator + duplicate suppression (ConsensusBuffer reused)
                          ↓
             structured result → GET /api/perception   (read-only)
```

### How Hailo acceleration is used

Unchanged and reused: the model runs on the AI HAT through
`detector.HailoYolo8Detector`, chosen by the existing `detector.kind: auto`
ladder. The perception layer adds **no** neural network and **no** second model:

* the tracker, FSM, quality gate, ROI extraction and validation are all CPU
  work on already-computed data;
* its own decode path (`decode_own`) exists for the standalone benchmark and is
  **off** in the flight loop, so the mission's decode is observed rather than
  repeated — zero added decode CPU;
* decode attempts are cadence-limited and skipped outright when the frame cannot
  support them (blur below floor, ROI below `min_roi_side_px`).

### The seam (why `mission.py` is untouched)

`Mission._worker()` imports its collaborators **inside the loop body**:

```python
def _worker(self, cam):
    from decoder import decode_frame
    from detector import detect_tiles
    ...
        boxes = detect_tiles(self.detector, frame, ...)
```

so both names are attribute lookups on the module at every call. The tap
replaces those two module attributes with pass-through wrappers — the only place
where `(frame, boxes, payload)` exist together. `uninstall()` restores them by
identity, and `perception.enabled: false` never installs anything.

Guards that make this safe:

* a thread-local depth counter means tile-local `detect()` calls are **not**
  observed as full frames (their coordinates are tile-local; nine tiles would
  otherwise scatter the tracker across nine phantom targets);
* observations are buffered and flushed **once per frame**, because the mission
  performs up to four decodes per frame and feeding each to the tracker would
  make a locked target look lost;
* a watchdog flushes a frame whose successor never arrived (stalled camera);
* every wrapper is try/except'd: if perception raises, the original result is
  still returned and the mission is unaffected;
* `PerceptionTap.name` forwards the inner detector's name **verbatim**, because
  `_worker` selects tiled-vs-full inference from `name != "classical"`.

### Robustness techniques implemented (mapped to the brief)

| Technique | Where | Note |
|---|---|---|
| Short exposure | `perception.exposure_s` → blur estimate | ISP controls the real shutter; this is a configurable budget |
| Blur / underexposure checks | `perception.assess_quality` | blur on ROI, exposure on downscaled scene crop |
| Detection confidence filtering | existing `qr_filter` (reused) | gate applied before the tracker sees boxes |
| Temporal smoothing of boxes | `tracker.BoxTracker` EMA | geometry only — **no frame averaging**, which would add blur |
| Tracking across frames | `tracker.py` | IoU + motion gate + velocity; ids survive fast motion |
| Detection persistence | `tracker.min_hits`, `perception.required_streak` | a single frame cannot confirm anything |
| Multi-frame QR decoding | `decode_every_n` + reused `ConsensusBuffer` | agreement required before publishing |
| Perspective correction | `perception.qr_roi` | `cv2.QRCodeDetector.detect` quad → `warpPerspective` |
| Geometric validation | ROI margin, upsample accounting, `min_roi_side_px` | |
| Duplicate suppression | `perception._reported` + `duplicate_window_s` | reported once per window |
| Graceful recovery | `lost_grace_s`, `error_recover_s`, `max_consecutive_errors` | see the FSM table |
| Expensive stabilisation avoided | — | no optical flow, no ECC, no frame stacking |

### State machine and recovery paths

| From | To | Trigger | Recovery reason recorded |
|---|---|---|---|
| SEARCHING | CANDIDATE_DETECTED | any box survives the filter | |
| CANDIDATE_DETECTED | TARGET_TRACKING | `hits >= min_hits` | |
| TARGET_TRACKING | QR_DECODING | decode cadence due | |
| QR_DECODING | PAYLOAD_VALIDATED | payload valid **and** streak reached | |
| PAYLOAD_VALIDATED | RESULT_REPORTED | next frame (send already happened) | |
| any | SEARCHING | target gone longer than `lost_grace_s` | `target_lost` / `decode_failed` |
| any | ERROR | `max_consecutive_errors` camera/detector/Hailo errors | |
| ERROR | SEARCHING | `error_recover_s` elapsed | `detector_error_recovered` |
| ERROR | TARGET_TRACKING | a target is genuinely back | `track_after_error` |
| — | (detail only) | invalid payload | `invalid_payload` |
| — | (detail only) | payload already reported | `duplicate_payload` |

---

## E. Code changes

### New files (`real_pi/mission_pi/`)

| File | Lines | Purpose |
|---|---:|---|
| `tracker.py` | ~330 | `Track` + `BoxTracker`: association, motion gate, EMA, persistence, expiry |
| `perception.py` | ~700 | quality gate, `qr_roi`, `PayloadValidator`, `PerceptionFSM`, `PerceptionPipeline` |
| `perception_tap.py` | ~300 | read-only install onto the running stack; `ObservationBuffer`; detector decorator |
| `tests/test_tracker.py` | ~230 | 24 tests |
| `tests/test_perception_fsm.py` | ~470 | 45 tests |
| `tests/test_perception_tap.py` | ~420 | 26 tests |
| `tests/test_perception_bench.py` | ~170 | 20 tests |
| `tools/perception_bench.py` | ~640 | Phase-8 metric harness (`make` / `run` / `live`) |
| `tools/fov_report.py` | ~300 | Phase-4 feasibility calculator + lever analysis |
| `docs/PERCEPTION_PIPELINE.md` | this file | |

### Modified files (additive only)

| File | Change |
|---|---|
| `config.yaml` | **new** `perception:` section (30 keys, commented). No existing key changed. |
| `main.py` | install the tap after the `Mission` is built; wrap the resolved detector in `bring_up_thread`. ~8 lines. |
| `server.py` | **new** `GET /api/perception` route. No existing route touched. |
| `real_pi/README.md` | pointer to this document. |

### Explicitly NOT modified

`mission.py`, `qr_filter.py`, `consensus.py`, `decoder.py`, `cameras.py`,
`detector.py`, `hailo_decode.py`, `fc_link.py`, `streamm1.py`, `geo.py`,
`fence.py`, `qr_relay.py` — and every existing test file.

---

## F. Configuration and setup

The layer is controlled by one new config section. Defaults are chosen so that
installing it changes nothing about flight behaviour:

```yaml
perception:
  enabled: true              # false = never installed (no-op)
  blur_min: 18.0             # CALIBRATE on your lens
  luma_min: 25.0
  dark_pct_min: 8.0
  exposure_s: 0.002          # ASSUMED shutter — the blur figure is an estimate
  max_motion_blur_px: 3.0
  require_quality_for_decode: true
  min_hits: 2
  max_misses: 8
  stale_s: 0.5
  decode_every_n: 2
  decode_own: false          # true only for the standalone benchmark
  min_roi_side_px: 28
  perspective_fix: true
  required_streak: 2
  miss_tolerance: 1
  duplicate_window_s: 30.0
  lost_grace_s: 1.5
  error_recover_s: 5.0
  max_consecutive_errors: 5
```

### Run it

```bash
cd real_pi/mission_pi
python3 main.py                       # as before — the tap is automatic
curl http://<pi>:8000/api/perception  # the structured result
```

Set `perception.enabled: false` (or `python3 -c "import perception_tap;
perception_tap.uninstall()"` in a REPL) to switch it off. Nothing else changes.

### Feasibility and benchmarking

```bash
python3 tools/fov_report.py --alts 7,10,15 --exposure-ms 2
python3 tools/perception_bench.py make --out /tmp/qrbench
python3 tools/perception_bench.py run  --corpus /tmp/qrbench \
        --md /tmp/qrbench/results.md --json /tmp/qrbench/results.json
python3 tools/perception_bench.py live --seconds 30 --cam bottom   # needs the Pi
```

---

## G. Automated tests and actual results

New tests: **115**, all passing. Full existing suite re-run: **33 files passed,
0 failed, 2 self-skipped**.

| Suite | Tests | Result |
|---|---:|---|
| `tests/test_tracker.py` | 24 | **OK** |
| `tests/test_perception_fsm.py` | 45 | **OK** |
| `tests/test_perception_tap.py` | 26 | **OK** |
| `tests/test_perception_bench.py` | 20 | **OK** |
| `bash tools/run_tests.sh` (whole repo) | 35 files | **33 passed, 0 failed, 2 skipped** |

Skipped (self-skipping, hardware absent): `test_hef_and_focus.py`,
and `test_fence_e2e_fake_fc.py` skipped in the loaded run.

**Honest note on one flake:** during one full-suite run,
`tests/test_ui_command_api.py` failed; it passed 3/3 in isolation and in the
final full run. It asserts a maximum MJPEG frame gap of 0.65 s on a loaded
sandbox, and the failing run printed a 0.335 s gap on the line *before* the
failure — so the failing assertion was a neighbouring timing check. I could not
reproduce it and I am **not** claiming it is unrelated to my change; the tap
adds one 0.25 s-cadence daemon thread that is live during those tests. Treat it
as an open question, not a closed one.

### Bugs the tests caught in my own code (each was a real defect)

1. **`RESULT_REPORTED` bounced back to `PAYLOAD_VALIDATED`** every frame, which
   would re-report a duplicate forever.
2. **A new track never evaluated the persistence gate**, so `min_hits: 1` did
   not confirm on the first frame — it waited for a second frame it never
   needed.
3. **`lost_grace_s` was defined but unused**; a vanished target dropped the FSM
   out of `TARGET_TRACKING` on the first frame instead of after the grace.
4. **The quality gate flagged every readable QR as "dark"** because it measured
   the 5th-percentile luminance on the QR ROI — and a QR is ~50 % black *by
   construction*. Every decode was skipped. Exposure is now measured on the
   scene, blur on the ROI.
5. **Association failed on the first fast inter-frame step** (no velocity
   history yet), spawning a duplicate track per frame. Fixed with a
   velocity-slack motion gate; the residual limit is pinned by a test.
6. **The exposure downscale could *upscale* a small frame**, turning a black QR
   grey and silently changing what the percentile meant.
7. **`perception_tap` was undefined in `bring_up_thread`** — caught by the
   repo's own `test_server_first.py`, which is exactly why the full suite was
   run before claiming anything.

---

## H. Performance benchmark and measurements

`tools/perception_bench.py run --corpus /tmp/qrbench`. **Environment: x86 CI
sandbox, ONNX on CPU, no Hailo, no camera.** Latency and FPS here are *this
host's* numbers and are explicitly not the Pi 5's.

Stage: **recorded/synthetic** — not flight evidence.

| Metric | Value |
|---|---|
| frames | 78 |
| `detection_recall` (IoU ≥ 0.5) | **0.7907** |
| `detection_recall_cov` (box contains ≥ 80 % of target) | **0.8837** |
| `detection_precision` (IoU ≥ 0.5) | **0.8718** |
| `detection_precision_cov` | **1.0** |
| `fp_per_frame` | **0.0641** |
| `decode_success` (frames labelled with a payload) | **0.5581** |
| `decode_after_detection` | **0.6176** |
| `detect_ms` mean / p95 | 665 / 705 |
| `decode_ms` mean | 868 |
| `pipeline_ms` mean / p95 | 448 / 1485 |
| `pipeline_fps` | 0.89 (sandbox CPU) |
| `time_to_first_payload_s` | **1.33** |

### Acceptance criteria — PROPOSED, not achieved claims

| Criterion | Target | Measured | Verdict |
|---|---|---:|---|
| `detection_recall_at_15m` | ≥ 0.90 | **0.0** | **FAIL** |
| `detection_precision` | ≥ 0.85 | 0.8718 | PASS |
| `fp_per_frame` | ≤ 0.10 | 0.0641 | PASS |
| `decode_success_le9m` | ≥ 0.90 | **1.0** | PASS |
| `pipeline_fps` | ≥ 5.0 | 0.89 | **NOT APPLICABLE** (not the flight host) |
| `time_to_first_payload_s` | ≤ 10 s | 1.33 | PASS |

### Per-condition results

| Condition | tp | fp | fn | recall | decode ok/total |
|---|---:|---:|---:|---:|---:|
| alt7_static | 3 | 0 | 0 | 1.00 | 3/3 |
| alt10_static | 3 | 0 | 0 | 1.00 | 3/3 |
| alt10_offcentre | 3 | 0 | 0 | 1.00 | 3/3 |
| alt10_rot90 | 3 | 0 | 0 | 1.00 | 3/3 |
| alt10_rot45 | 3 | 0 | 0 | 1.00 | **0/3** |
| alt10_tilt30 | 3 | 0 | 0 | 1.00 | **0/3** |
| alt10_bright | 3 | 0 | 0 | 1.00 | 3/3 |
| alt10_dark | 0 | 3 | 3 | 0.00 | 3/3 |
| alt10_noisy | 3 | 0 | 0 | 1.00 | 2/3 |
| alt10_blur (k=9) | 3 | 0 | 0 | 1.00 | **0/3** |
| alt10_blur_strong (k=15) | 0 | 0 | 3 | **0.00** | 0/3 |
| alt10_occluded (22 %) | 3 | 0 | 0 | 1.00 | **0/3** |
| alt10_none (no panel) | 0 | 0 | 0 | n/a | 0/0 |
| **alt15_static** | 0 | 1 | 3 | **0.00** | 0/3 |
| enter_exit | 4 | 1 | 0 | 1.00 | 4/4 |

### What these numbers actually say

* **Detection and decoding fail independently, exactly as predicted.** At
  15 m, detection recall is 0.00 — the panel is present and the model misses
  it. At 7 m, detection is perfect and decode is 3/3. Different problems.
* **The approach stair is load-bearing.** `decode_success_le9m = 1.0` versus
  `decode_success` overall `0.5581`: nearly all decode failures are at ≥10 m,
  which is the altitude the mission deliberately descends past.
* **Rotation and tilt are the biggest decode killers** (0/3 at 45° rotation and
  at 30° tilt, while detection stays 1.00). That is a real, actionable finding
  for the approach: prefer a descent that centres the panel.
* **Blur kills detection first** (k=15 → recall 0.00), which is the case for a
  short shutter.
* **`alt10_dark` looked contradictory** (detection 0/3 but decode 3/3) and was
  investigated rather than reported at face value: the model boxes the panel
  **loosely** (59 px around a 39 px code, IoU 0.43). Strict IoU scores that as a
  miss *and* a false positive while the decoder succeeds. This produced a second
  scoring rule (`*_cov`), which is why `detection_precision_cov` is 1.0 while
  `detection_precision` is 0.87. Both are reported.

### Separating the four kinds of evidence

| Kind | Status |
|---|---|
| 1. Unit tests | **Run and passing** (115 new tests) |
| 2. Recorded/synthetic benchmark | **Run and measured** (tables above) |
| 3. Live-camera bench | **NOT YET MEASURED** — no camera in this environment. `tools/perception_bench.py live` implements it. |
| 4. Actual flight | **NOT MEASURED** — no vehicle, no airfield |

Bench testing does not prove flight performance. Nothing in this document
should be read as a flight-readiness claim.

---

## I. Limitations and unverified assumptions

### Unverified assumptions (in order of how much they'd change the answer)

1. **The 75°/60° FOV is vendor-quoted, not measured.** Every number in
   §C moves with it. Measure it (`tools/cam_probe.py` + a tape measure at 10 m)
   and re-run `tools/fov_report.py`.
2. **Camera tilt is assumed 0° (nadir).** A 30° mount error costs ~13 % of
   pixels-per-module.
3. **`exposure_s: 0.002` is assumed.** The blur figure is an estimate; the ISP
   picks the real shutter and it is not read back.
4. **Robustness claims are synthetic-only.** Synthetic frames contain no real
   sensor noise, no rolling-shutter skew, no prop-wash vibration spectrum and no
   real print/lamination artefacts. The vibration in the corpus is a ±6 px
   translation, which is a toy compared with a real airframe.
5. **`blur_min: 18.0` is a placeholder.** Laplacian variance is scene- and
   lens-dependent; calibrate it on real frames.
6. **Detector-level recall at 15 m with a real lens is unknown.** The synthetic
   15 m panel is 26 px; the model misses it at conf 0.35. Tiling and preset
   tuning may recover it — that measurement needs the real optics.

### Known limitations of the implementation

* The tap uses **module-attribute replacement**, which is a monkey-patch. It is
  safe here only because `_worker` performs in-loop imports; a future refactor
  to module-level imports in `mission.py` would silently disable the layer. The
  behaviour is pinned by `tests/test_perception_tap.py`.
* The tracker is a centroid/IoU tracker with constant-velocity prediction — not
  a Kalman filter, and not appearance-based. Two QRs closer than one box
  diagonal (~53 px at 34 px) are ambiguous to it, by design and by test.
* `decode_own` is off in the flight loop, so the perception layer's own decode
  statistics (`decode_attempts`, `decode_failures`) stay at zero in flight; the
  mission's decode is observed instead. The benchmark turns it on.
* Duplicate suppression is per-process and in-memory; a restart forgets what it
  has already reported.
* No telemetry/gimbal integration. If attitude is available from MAVLink it
  could sharpen the motion gate, but that was not assumed.

### Recommended next steps

1. **Measure the FOV and the 15 m footprint on the real hardware**, then re-run
   `fov_report.py`. This is the single highest-value action.
2. **Run `perception_bench.py live`** on the bench and calibrate `blur_min` and
   `luma_min` against real frames.
3. **Capture a labelled corpus at 7/10/15 m** (`labels.jsonl` format documented
   in the tool) and score it — that converts §H from synthetic to recorded-real.
4. **If 15 m decoding is required**, change the *target*, not the detector:
   print the QR across the A3 long side (420 mm) and shorten the payload. That
   is a +41 % pixels-per-module change for the cost of a printer.
5. **Consider a narrower lens** if search time is acceptable — 50° HFOV nearly
   doubles px/module at 15 m, at the cost of footprint.
6. **Only then** consider retraining. The current numbers do not justify it:
   detection works at 7–10 m and decoding works inside the stair.
7. Any integration of this perception output with autonomous approach or landing
   must be separately validated in simulation and controlled flight. This layer
   deliberately reports `drives_flight: false` and `perception_only: true`, and
   nothing here is a statement that it is safe to descend near a target.
