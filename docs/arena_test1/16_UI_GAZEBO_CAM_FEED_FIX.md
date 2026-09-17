# Fix: UI No Feed of Gazebo Cams Working Continuously

**Your issue:** In UI `http://192.168.29.221:8000` or `http://localhost:8100`, Gazebo cam tiles dark / not continuous, even though `gz_cam_bridge` and `mission_pi` logs say cameras started 2560x1440 @25fps.

**Root causes:**
1. Bridge binds `127.0.0.1` by default — Windows laptop `192.168.29.221:8099` can't reach it, only Pop!_OS localhost can. Fixed: default bind now `0.0.0.0` for 2-laptop.
2. OpenCV `VideoCapture` MJPEG URL drops (bridge restart, network hiccup) and never reconnects — cam stops forever. Fixed: auto-reconnect after 10 failed reads.
3. High-res 2560x1440 MJPEG via FFMPEG backend can lag/buffer — need to check health endpoints.

---

## Fix Applied (Code)

### 1. `tools/gz_cam_bridge.py` — bind 0.0.0.0 by default
```python
# Before: default 127.0.0.1 — Windows can't reach
ap.add_argument("--bind", default="127.0.0.1")

# After: 0.0.0.0 for 2-laptop Pop!_OS+Windows, 127.0.0.1 for single laptop
ap.add_argument("--bind", default="0.0.0.0", help="Bind 0.0.0.0 for 2-laptop")
```
- Now `http://192.168.29.221:8099/` reachable from Windows, not just `127.0.0.1:8099`
- Check: `curl http://192.168.29.221:8099/health` from Windows PowerShell

### 2. `cameras.py` USBCamera — auto-reconnect on MJPEG drop
```python
def _grab(self):
    ok, frame = self._cap.read()
    if ok and frame is not None:
        self._fail = 0
        return frame
    self._fail = getattr(self, '_fail', 0) + 1
    if self._fail >= 10:
        try:
            self._close()
            time.sleep(0.5)
            self._open()  # reopen VideoCapture URL
            self._fail = 0
            ok, frame = self._cap.read()
            return frame if ok else None
        except Exception:
            time.sleep(1.0)
    return None
```
- Before: one read fail → return None forever → UI dark
- After: 10 fails → close + open → reconnect → continuous feed

### 3. StreamManager keeps last good JPEG
- `streamm1.py` CamStream: if `cam.jpeg()` returns None, keep old `_jpg` — UI shows last frame with increasing `age_s`, not blank
- Check: `curl http://127.0.0.1:8000/api/cameras | python3 -m json.tool` — frames count should climb, age_s <1s

---

## Manual Checks — All Commands (Pop!_OS)

### Terminal 1 — Gazebo
```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
gz sim -v 4 -r sim/worlds/mission_world.sdf
# Check topics:
gz topic -l | grep iris
# /iris/front/image + /iris/bottom/image must exist
```

### Terminal 2 — Bridge (SYSTEM /usr/bin/python3, bind 0.0.0.0)
```bash
conda deactivate; deactivate
/usr/bin/python3 -c "import gz.transport13; print('ok')"
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
/usr/bin/python3 tools/gz_cam_bridge.py --port 8099 --bind 0.0.0.0 --qr-boost --contrast 1.8 --saturation 0.6 --sharpness 2.0 --enhance-mode ground_suppress --verbose
# Check Pop!_OS:
curl http://127.0.0.1:8099/health | python3 -m json.tool
# frames should climb, age_s <1s, size [2560,1440]
# Browser: http://127.0.0.1:8099/ — both front+bottom boosted mjpeg
# Check from Windows (2-laptop):
# PowerShell: curl http://192.168.29.221:8099/health
# Must work now with 0.0.0.0 bind, not just 127.0.0.1
# Browser Windows: http://192.168.29.221:8099/bottom.mjpg — should show ground+QR
```

### Terminal 3 — SITL (venv-ardupilot, --no-mavproxy method from working branch)
```bash
source ~/venv-ardupilot/bin/activate
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --no-mavproxy
# Wait SERIAL0 on TCP port 5760
# Windows MP: TCP 192.168.29.221:5760 Connect → Flight Data alive → Terminal prints SERIAL1 on TCP port 5762 → now 5762 LISTEN
# Check:
ss -tlnp | grep -E "5760|5762|8099"
# 0.0.0.0:5760 LISTEN, 0.0.0.0:5762 LISTEN (after MP connects), 0.0.0.0:8099 LISTEN
```

### Terminal 4 — mission_pi YOLO (after MP connected, 5762 LISTEN)
```bash
source ~/venv-ardupilot/bin/activate
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
python3 main.py --config config.gazebo.yolo.5760.yaml
# Or config.gazebo.yolo.yaml if 5762 works
# Logs: cam1 actual 2560x1440 @25fps, cam2 actual 2560x1440 @25fps, streams ok 12fps 960w q70 x2, detector yolo ready, qr_boost ON
# Check:
curl http://127.0.0.1:8000/health | python3 -m json.tool | grep -A5 cams
# cam1 running true frames >0 age_s <1, cam2 same
curl http://127.0.0.1:8000/api/cameras | python3 -m json.tool
# frames climbing
```

### Terminal 5 — UI Checks (Continuous Feed)

**Pop!_OS UI:**
```bash
xdg-open http://127.0.0.1:8000/
# Should show CAM1 front + CAM2 bottom tiles, 12fps smooth, green boxes when QR detected

# Direct stream URLs (open in browser):
# http://127.0.0.1:8000/api/camera/stream/cam1?fps=12
# http://127.0.0.1:8000/api/camera/stream/cam2?fps=12
# Should show continuous MJPEG, not 404/503

# Snapshot fallback:
# http://127.0.0.1:8000/api/camera/frame/cam1
# http://127.0.0.1:8000/api/camera/frame/cam2
# Should return JPEG, not 503 no frame yet
```

**Windows UI (2-laptop):**
```powershell
# Browser Windows:
# http://192.168.29.221:8000/ — Pi UI with cams
# http://192.168.29.221:8000/api/camera/stream/cam2?fps=12 — bottom cam continuous
# http://192.168.29.221:8099/bottom.mjpg — bridge direct, should be continuous
# http://localhost:8100 — mission-ui bridge UI, Pi link http://192.168.29.221:8000, should show CAM1+CAM2 tiles

# If tiles dark in http://localhost:8100:
# 1. Check Pi link box: http://192.168.29.221:8000 — connect — CAM1+CAM2 tiles should light up (UI polls Pi /api/camera/stream)
# 2. Check browser console F12 → Network → /api/camera/stream/cam2 should be 200 multipart, not 404/503
# 3. Check Pi /health from Windows: curl http://192.168.29.221:8000/health — cams running true frames >0 age_s <1
# 4. If age_s >5s, cam stopped — check Pop!_OS Terminal 4 logs for "no frames for 10 reads — trying reopen" — auto-reconnect should fix
# 5. If bridge /health age_s >5s, Gazebo topics dead — restart Gazebo Terminal 1
```

---

## Troubleshooting UI No Feed

| Symptom | Check | Fix |
|---------|-------|-----|
| UI tiles dark, /api/camera/stream 404 | `curl http://127.0.0.1:8000/api/cameras` — no cams | cameras.py detect failed — check config.gazebo.yolo.yaml cameras device http://127.0.0.1:8099/front.mjpg, bridge running |
| UI tiles dark, /api/camera/stream 503 not running | `curl http://127.0.0.1:8000/health` cams running false | USBCamera _open failed — bridge not running or URL wrong, check `curl http://127.0.0.1:8099/health` |
| UI tiles dark, /api/camera/frame 503 no frame yet | `curl http://127.0.0.1:8000/api/cameras` frames 0 age null | VideoCapture can't read MJPEG — OpenCV FFMPEG backend issue, try backend mjpeg + size [2560,1440], check bridge JPEG quality 80 |
| Bridge /health age_s >5s, frames not climbing | `gz topic -l \| grep iris` | Gazebo not publishing — restart Gazebo Terminal 1, check GZ_SIM paths |
| Bridge http://192.168.29.221:8099/ not reachable from Windows | `ss -tlnp \| grep 8099` shows 127.0.0.1:8099 | Old bridge binds 127.0.0.1 — kill and restart with --bind 0.0.0.0 (now default) + `sudo ufw allow 8099/tcp` |
| Pi /health cams ok but UI http://localhost:8100 tiles dark | Browser F12 Network /api/camera/stream/cam2 404 | mission-ui bridge Pi link not set — in http://localhost:8100, Pi link box paste http://192.168.29.221:8000 → Connect → CAM1+CAM2 tiles should appear (UI polls Pi stream) |
| Stream lag, old frame, age_s increasing | `curl http://127.0.0.1:8000/api/cameras` age_s >2s | High-res 2560x1440 MJPEG via FFMPEG lags — check Pop!_OS CPU, try lower stream fps 12 width 960 quality 70 (already), or check bridge contrast/sat/sharp enhance is PIL (CPU) — try --enhance-mode none for less CPU |
| YOLO boxes but no cam feed | Detector ok but cam jpeg fails | `cam.jpeg()` needs cv2 — check `python3 -c "import cv2; print(cv2.__version__)"` in venv-ardupilot |

---

## Verification Continuous Feed (No Crash)

- [ ] Gazebo topics /iris/front/image + /iris/bottom/image exist
- [ ] Bridge :8099/health frames climbing age_s <1s size [2560,1440] qr_boost ON, bind 0.0.0.0, reachable from Windows http://192.168.29.221:8099/health
- [ ] Bridge MJPEG http://127.0.0.1:8099/bottom.mjpg continuous in browser, not stopping
- [ ] SITL ss -tlnp 5760 LISTEN after start, 5762 LISTEN after MP connects to 5760
- [ ] mission_pi /health cams running true frames >0 age_s <1, streams 12fps 960w q70 x2, detector yolo ready
- [ ] Pi stream direct http://127.0.0.1:8000/api/camera/stream/cam2?fps=12 continuous MJPEG in browser
- [ ] Pi snapshot http://127.0.0.1:8000/api/camera/frame/cam2 returns JPEG
- [ ] Pop!_OS UI http://127.0.0.1:8000/ CAM1+CAM2 tiles continuous 12fps
- [ ] Windows UI http://192.168.29.221:8000/ same, and http://localhost:8100 with Pi link http://192.168.29.221:8000 shows CAM1+CAM2 tiles continuous
- [ ] Kill bridge Terminal 2 → Pi logs "no frames for 10 reads — trying reopen" → auto-reconnect when bridge restarted → feed resumes, no crash, no restart of Pi needed
- [ ] Logs: no traceback, /health ready true when FC+cam ok, false with reason when not

This fix ensures continuous Gazebo cam feed in UI — bridge 0.0.0.0 + auto-reconnect + stream keeps last JPEG + health endpoints.
