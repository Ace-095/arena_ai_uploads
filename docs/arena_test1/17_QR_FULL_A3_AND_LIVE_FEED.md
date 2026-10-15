# 17 — QR full-A3 + live camera feed (true MJPEG) — Pop!_OS + SITL + Pi

**Date:** 2026-09-17
**Branch:** `arena/01a0a7de-arena-ai-uploads`
**Fixes:** 2 user-reported issues after doc16
  1) QR model in Gazebo very small, white part big — QR should cover entire A3 white part (QR is entire A3, not QR in A3)
  2) Camera live feed still not live, shows as photos (1s snapshots) — wants true live feed, check main branch `streamm1.py`

---

## 1) QR full-A3 fix

### Root cause
`tools/make_qr_panel.py` old mode:
```py
side = int(min(w,h) * 0.72)  # 72% fill
panel.paste(code, ((w-side)//2, int(h*0.08)))
d.text(... caption)
```
- QR 72% of A3, pasted top with big white border + caption text
- In Gazebo `qr_panel_a3` box 0.297x0.42 uses `qr.png` as albedo_map — entire box shows texture
- So visual = small QR centered in big white A3 → detector sees ~50% white, QR small, fails at 15m unless 2x-A3 big panel

### Fix — `--full` flag, QR IS entire A3
Updated `tools/make_qr_panel.py`:
- `--full` makes QR 95% of A3 panel (min side), centered, thin black border only
- No caption, no big white — QR covers entire white part
- User: "QR should be entire A3 not QR in A3"

```bash
python3 tools/make_qr_panel.py --text MISSION-QR-001 \
  --out sim/models/qr_panel_a3/materials/textures/qr.png --full --dpi 300

python3 tools/make_qr_panel.py --text MISSION-QR-001 \
  --out sim/models/qr_panel_big/materials/textures/qr.png --full --dpi 300
```

Result: `qr.png` 3507x4960px (A3 @300dpi), 62K, QR square 95% width, centered.

### World default switched to honest A3
`sim/worlds/mission_world.sdf` before: default `qr_panel_big` 594x840mm 2x-A3 (classical needs 80+ px)
Now: default `qr_panel_a3` 297x420mm honest A3, because YOLO tiled mandatory detects 34px@15m (0.81).

```xml
<!-- HONEST A3: QR IS entire A3 95% fill -->
<include><uri>model://qr_panel_a3</uri><name>qr_target</name><pose>8 0 0 0 0 0</pose></include>
<!-- 2x-A3 big alternative commented for classical fallback -->
```

Verify in Gazebo:
```
gz sim -v4 -r sim/worlds/mission_world.sdf
# QR panel at 8,0 should show QR covering almost entire white box, not small QR in big white
```

For print: same PNG prints at 100% on A3 — QR covers almost entire sheet, matches sim.

---

## 2) Camera live feed — true MJPEG, not photos

### Root cause (why user saw photos)
- `streamm1.py` exists and is correct: CamStream thread jpeg(max_width,width quality) fps 12 width 960 q70, StreamManager sync after rig.detect + start_all
- `server.py` /api/camera/stream/{cam} serves multipart/x-mixed-replace, 404 if streams=None → UI falls back to polling
- `mission-ui/js/camera.js` MJPEG first: `img.src = base + /api/camera/stream/{cam}`, onerror → polling
- BUT old polling = 1000ms (1fps photos), and MJPEG never retried — if stream failed once (bridge restart, camera drop), tile stayed in polling photos forever → user sees "not live and photos"

Previous doc16 fixed:
- `gz_cam_bridge.py` bind 0.0.0.0 not 127.0.0.1 (2-laptop Pop!_OS 192.168.29.221 + Windows)
- `cameras.py` USBCamera _grab auto-reconnect after 10 fails

Still needed: UI retry logic + faster polling for live feel.

### Fix — camera.js true live
`mission-ui/js/camera.js` v2.1 2026-09-17:
- `startMjpeg(id)`: `img.src = base + /api/camera/stream/{cam}?fps=12&t=Date.now()`, onload logs "MJPEG live", onerror → polling with 3s retry
- `startPolling(id)`: interval 200ms (5fps live) not 1000ms photos, plus retryTimer every 3s tries MJPEG again → auto-recovery true live
- Modal also MJPEG first, polling 200ms fallback, 3s upgrade retry
- `stopAll` clears both pollTimer and retryTimer

```js
e.img.src = base + '/api/camera/stream/' + id + '?fps=12&t=' + Date.now();
// polling
c.pollTimer = setInterval(tick, 200); // 5fps live not 1s photos
c.retryTimer = setTimeout(retryLoop, 3000); // auto-retry MJPEG
```

### Verification — 5 terminals Pop!_OS

**T-A Gazebo + bridge:**
```
cd mission_pi
export GZ_SIM_RESOURCE_PATH=$(pwd)/sim/models:$GZ_SIM_RESOURCE_PATH
gz sim -v4 -r sim/worlds/mission_world.sdf
# separate SYSTEM python (not venv) — bind 0.0.0.0 for 2-laptop
python3 tools/gz_cam_bridge.py --bind 0.0.0.0 --port 8099 --qr-boost --contrast 1.8 --saturation 0.6 --sharpness 2.0 --enhance-mode ground_suppress
curl http://127.0.0.1:8099/health # frames>0 age<1
```

**T-B SITL + MP (order matters — doc15):**
```
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --no-mavproxy --map --aircraft test -l -35.363262,149.165237,584,0
# LISTEN 5760 — then MP
mavproxy.py --master=tcp:127.0.0.1:5760 --out=udp:127.0.0.1:14550 --out=tcp:192.168.29.221:5760 --out=tcp:127.0.0.1:5762 --console
# MP Flight Data alive? check TCP LISTEN 5762
ss -tlnp | grep 5762
```

**T-C mission_pi:**
```
cd mission_pi
source venv/bin/activate (or system)
python3 main.py --config config.gazebo.yaml
# logs: streams ok 12fps 960w q70 x2
curl http://127.0.0.1:8000/health | jq .cams,.streams
# cams running true frames>0, streams seq rising
curl -v http://127.0.0.1:8000/api/camera/stream/cam1?fps=12 --output /tmp/mjpeg.mjpg &
# should show multipart/x-mixed-replace continuous
curl http://127.0.0.1:8000/api/camera/frame/cam1 -o /tmp/frame.jpg && xdg-open /tmp/frame.jpg
```

**T-D mission-ui bridge (Pop!_OS):**
```
cd mission-ui
python3 -m http.server 8100
# http://127.0.0.1:8100/?pi=http://127.0.0.1:8000
# CAM1+CAM2 tiles should show "live 12fps MJPEG" not "polling snapshots"
# kill bridge: pkill -f gz_cam_bridge — tiles should go polling 200ms then auto-retry MJPEG when bridge back
```

**T-E Windows laptop (2-laptop):**
```
# Pi link: http://192.168.29.221:8000
# Bridge: http://192.168.29.221:8100?pi=http://192.168.29.221:8000
# CAM tiles live MJPEG 12fps, enlarge modal also live
```

### Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| Tiles show "polling snapshots" 1s photos | /api/camera/stream 404 — streams=None or not running | Check main.py streams.sync(rig) + start_all after rig.detect, config.gazebo.yaml stream fps 12 width 960 |
| MJPEG fails then stays photos | Old camera.js no retry | Update to v2.1 — polling 200ms + 3s MJPEG retry, or reload page |
| Frames dark / no frames | Bridge 127.0.0.1 blocks Windows, or camera _grab stuck | Use --bind 0.0.0.0, check cameras.py auto-reconnect after 10 fails, curl :8099/health |
| QR still small white big | Old qr.png 72% texture cached | Regenerate with --full --dpi 300, clear Gazebo cache ~/.gz/rendering/ogre2, re-launch world |
| QR not detected @15m | Using qr_panel_big old texture or classical-only | Use qr_panel_a3 honest A3 with YOLO tiled mandatory (34px@15m 0.81), config.gazebo.yaml detector yolo |

---

## Files changed

- `mission_pi/tools/make_qr_panel.py` — added --full flag, 95% fill QR IS entire A3, thin border, no caption
- `mission_pi/sim/models/qr_panel_a3/materials/textures/qr.png` — regenerated 3507x4960 62K full-A3
- `mission_pi/sim/models/qr_panel_big/materials/textures/qr.png` — regenerated same full-A3
- `mission_pi/sim/worlds/mission_world.sdf` — default qr_panel_a3 honest A3 (QR full), big panel commented alternative
- `mission-ui/js/camera.js` — true live MJPEG: 200ms polling live not 1s photos, 3s MJPEG auto-retry, modal live, clear timers

No changes to `streamm1.py` — already implements true live (same as main branch), encode once per tick share many, period 1/fps, cam.jpeg(max_width,quality).

---

## Manual check checklist (user requested)

- [ ] `python3 tools/make_qr_panel.py --full --dpi 300` generates full-A3 QR covering white part
- [ ] Gazebo `mission_world.sdf` shows QR covering entire A3 box at 8,0, not small QR in big white
- [ ] Bridge `gz_cam_bridge.py --bind 0.0.0.0 :8099` health frames>0 age<1
- [ ] `main.py --config config.gazebo.yaml` streams ok 12fps 960w q70 x2, /health cams running true
- [ ] `curl /api/camera/stream/cam1?fps=12` returns multipart continuous MJPEG
- [ ] UI `http://127.0.0.1:8100/?pi=http://127.0.0.1:8000` tiles show live 12fps MJPEG, not photos
- [ ] Kill bridge, tiles go polling 200ms live, bridge back → auto-retry MJPEG in 3s
- [ ] Windows `http://192.168.29.221:8100?pi=http://192.168.29.221:8000` live tiles + modal live
