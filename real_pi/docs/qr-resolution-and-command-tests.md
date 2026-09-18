# QR resolution and UI-command verification

## What was actually tested

Software bench only: no Pi camera, Pixhawk, Mission Planner instance, radio link,
Hailo device or flying vehicle was attached. `FileCamera` tests verify application
state and streaming, **not** physical ISP acceptance, glass-to-glass latency or
flight readiness. Earlier descriptions of “zero delay / zero lost frames / MP
instantly” were too strong. Fresh-connection timing is not camera-to-screen latency.

## Camera Module 3: classical vs YOLO

“Camera 3” here means Raspberry Pi Camera Module 3, not a third stream slot.
The app has two roles, `cam1` / `cam2`. Current configuration requests:

| Role | Sensor match | Capture size | Horizontal FOV |
|---|---|---:|---:|
| front | IMX708 (Cam Module 3) | 4608 × 2592 | 66° |
| bottom | IMX477 | 2028 × 1520 | 100° |

These are **configured**, not measured sensor output/optical calibration. Some
backends resize to the requested dimensions; making an image larger does not
create optical detail. Verify actual camera mode and focus on the Pi.

- **Classical:** OpenCV/optional pyzbar use the capture-sized image. There is no
  application-level 640-pixel resize. Native resolution alone does not guarantee
  finder-pattern detection; the algorithms can still fail on a small QR.
- **YOLO full-frame:** the 4608-wide frame is letterboxed into 640 × 640. Its image
  content becomes 640 × 360 plus padding: **7.2× smaller linearly** (13.9% retained
  width), not a 7.2× camera-resolution change or a detection-success percentage.
  A 72-pixel QR becomes only 10 pixels across at the network input.
- **YOLO 3×3, overlap 0.25:** the actual tile widths are 1920–2304 pixels. Resizing
  these to 640 makes the same QR 20–24 pixels wide: **2–2.4× the full-frame size**.
  This is nine inferences, not free extra resolution.
- **Decode:** detected boxes map back onto the **original capture-sized frame**;
  the ROI is cropped there and optionally upscaled. The decoder does not read the
  640 network image or the 960-wide JPEG preview. Upscaling cannot restore missing
  modules. The periodic full-frame decode path remains as a fallback.
- **Preview:** `stream.width: 960` and JPEG quality affect the laptop view, not the
  detector's input dimensions. Lower preview bandwidth before lowering capture
  resolution when preserving QR detail matters.

### Reproducible synthetic result

Bundled ONNX model, confidence/filter floor 0.35, OpenCV decoder (no pyzbar),
4608 × 2592 green background, centered `QR-TEST-001`, sizes include quiet zone.
Each cell means “detected a box overlapping the QR AND decoded the correct
payload from that box on the native frame.” This table excludes periodic
full-frame decode fallback; it is not the entire mission's success rate.

| QR outer width in capture pixels | Classical | YOLO full-frame | YOLO 3×3 |
|---:|:---:|:---:|:---:|
| 48 | no | no | no |
| 64 | no | no | yes |
| 80 | no | no | yes |
| 96 | no | yes | yes |
| 144 | yes | yes | yes |

The known-location native ROI also failed at 48 pixels, despite bypassing the
detector. This illustrates why a bigger box/confidence tweak cannot solve missing
sampling detail. On this sandbox with two CPU cores assigned, YOLO full-frame
was about 0.08–0.11 s per case, versus 0.64–0.84 s for nine tiles. These are **not
Pi 5/Hailo FPS figures** and do not include the full concurrent flight workload.

Re-run with your venv from `real_pi/mission_pi`:

```bash
python tools/qr_resolution_probe.py --width 4608 --height 2592 --sizes 48 64 80 96 144
```

### How to improve real scanning

1. Keep native capture detail; focus at the intended distance. Good lighting,
   sharp optics, short enough exposure for motion, and an undamaged quiet zone
   matter more than an aggressively sharpened preview.
2. Use tiled YOLO for small candidates, then native ROI decode. Bottom tiling is
   enabled by default. **Cam3 is front by default and was not tiled**: set
   `detector.tile_front: true` for that role if needed, then restart. It is false
   by default to avoid silently multiplying dual-camera CPU load. The worker now
   honors `tile_overlap` from config instead of silently using the default.
3. Use short QR payloads (fewer modules) and larger prints. Aim for roughly
   3–4 captured pixels per module as a starting margin, then validate on actual
   target material. This is a design heuristic, not a guaranteed decode threshold.
4. If undersampled, increase QR size or reduce distance within approved flight
   limits. Do not try to fix it by merely lowering YOLO confidence; that admits
   more false boxes. Ground/night strict presets still require real decoding for
   flight cues.
5. Use the Hailo path when available; profile CPU fallback under both cameras.
   Reduce preview fps/width before sacrificing capture detail. Do not assume that
   raising `input_size` works with a fixed-shape ONNX/HEF model.

For a nadir camera over flat ground:
`ground_width = 2 × height × tan(horizontal_FOV/2)` and
`QR_pixels ≈ image_width × QR_width / ground_width`.
At 15 m, a 20 cm square including its quiet zone is approximately **47 pixels**
with the configured Cam3 4608/66° geometry, but only **11 pixels** with the
configured bottom 2028/100° geometry. Front-camera ground views are oblique, so
that Cam3 nadir example is NOT an actual front-camera flight prediction. Camera
role/lens choice matters at least as much as detector choice; 15 m detection
cannot be equated to guaranteed payload decoding.

## Fence corrections

The old implementation had real failures, not just an absent visual overlay:

- `MISSION_ITEM_INT` put FENCE's value into `frame` and omitted the trailing
  `mission_type` extension (defaulting to ordinary mission).
- It repeated the closing vertex, and set polygon size on only the first item.
- It ignored retransmitted requests and subscribed too late for the final ACK.
- Mission download and parameter-read echoes had the same subscribe-after-send
  race. A simulated instantaneous FC reproduces both failures on the old code.
- UI submitted a second competing upload through the MP bridge, displayed an
  incompatible response shape, and swallowed clear failures.
- Blocking FC calls ran on the ASGI event loop and could pause MJPEG.

Now the Pi uses correctly encoded MAVLink2 polygon items (count on every vertex,
no duplicate closure), handles re-requests/out-of-order requests, subscribes before
sending, checks source/type, and serializes its mission transfers. ACK is followed
by polygon readback; only matching readback updates the mission/UI as confirmed.
Coordinates are validated, including zero latitude/longitude, duplicates,
nonfinite/out-of-range coordinates and intersecting/zero-area polygons. The UI
supports **one inclusion polygon**; other FC fence layouts return an error rather
than being flattened into a misleading shape. Failed edits may have affected the
FC before readback failed: refresh before proceeding, never trust the old drawing.

Uploads/clears are rejected while armed. No FC link, busy transaction, rejected
ACK, failed clear and mismatched readback are visible errors. There is one writer:
selected Pi, or bridge fallback only when no Pi URL is selected. A stale bridge
fence event cannot overwrite a remote Pi's result. FC work runs in a thread,
leaving the HTTP event loop available for streams.

**Uploading a polygon is NOT enabling fence enforcement.** We deliberately do
not change `FENCE_ENABLE`, `FENCE_TYPE` or `FENCE_ACTION`. Verify those separately
in Mission Planner and download/refresh its Fence view. Uploading through another
GCS does not guarantee MP automatically refreshes its cached polygon. The UI's
old “armable” label is now a fence check, not a flight-safety certification.

## Other command fixes

- QR ISP/software buttons now target the selected Pi, not the laptop origin.
- Camera tabs select the camera the boost buttons actually modify.
- Sliders re-seed from Pi state after preset/profile changes, preventing the next
  slider adjustment from restoring stale exposure/adaptive settings.
- `bench` restores the normal ISP profile instead of leaving the previous boost.
  Software enhancement is a separate control and remains independently selected.
- Altitude accepts finite valid numbers, clamps the sweep when lowering the
  ceiling, and reports whether `FENCE_ALT_MAX` was actually echoed by the FC.
  Pi runtime updates are not silently represented as successful FC updates.
- UI altitude/preset edits are **runtime**, not writes to `config.yaml`. Edit that
  file and restart for persistence. The preset list reflects the loaded config,
  not arbitrary changes on disk without a restart.
- RTL/LAND and MP parameter-read/log tools intentionally still use the laptop
  bridge. These tests do not claim those flight actions ran on physical hardware.

## Regression coverage and commands

From `real_pi/mission_pi`, with the venv dependencies installed:

```bash
# Extra test-only dependencies:
pip install httpx qrcode pillow opencv-python-headless
# Optional real DOM/app.js click harness (Node >=18):
(cd ../mission-ui && npm ci)
python tests/test_fence_protocol.py
python tests/test_ui_command_api.py
python tests/test_resolution_pipeline.py
python tests/test_preset_stream.py
```

- **8 wire/validation tests:** real pymavlink serialization parsed by a simulated
  FC, instantaneous ACK/readback/parameter replies, repeated/out-of-order requests,
  legacy requests, rejection, timeout, wrong sender and malformed polygons.
- **7 live HTTP integration tests:** successful fence/clear/readback; offline,
  armed, busy, rejection/mismatch/invalid input; camera and preset changes; altitude
  clamp/echo; a continuous stream during a delayed fence upload; UI click harness.
- **DOM harness:** actual `index.html` and `app.js` handlers in jsdom drive real
  HTTP requests to the Pi fixture. Map rendering, websocket displays and MP are
  stubbed. This is not a full browser/network/CORS or actual Leaflet drawing test.
- **2 resolution-contract tests:** network and tile dimensions, original-frame
  decode input, front/bottom/classical tile choice and configured overlap.
- **Continuous stream:** a simulated 1.2 s FC upload did not interrupt the same
  MJPEG connection: repeated runs delivered 30–31 frames in roughly 2.5 s, worst
  observed inter-frame gap 0.084–0.168 s. This is localhost/file-camera continuity,
  not camera-to-screen Wi-Fi latency.

Hardware acceptance still required, **disarmed and props removed**: connect UI to
Pi, upload a small local polygon, verify the UI readback and independently download
it in MP, test clear/re-upload, inspect enforcement parameters, verify physical
camera focus/settings and stream latency under the intended detector workload.
Do not treat the software bench as permission to skip preflight checks.
