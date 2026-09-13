"""QR-hunt mission: DO_SPRAYER trigger -> GUIDED takeover -> search -> decode.

Flow (drone is at ~15 m over home in AUTO when we take over):
  WAIT_TRIGGER  plan scan (sprayer cmds 216/222/42600) + MISSION_CURRENT;
                peek-ahead item fetch + STATUSTEXT backup [or POST /takeover]
  TAKEOVER      set GUIDED, verify; snapshot already holds home + fence
  SWEEP_YAW     12 stepped 30° yaws with settle pauses (sharp frames beat
                motion blur); any fresh bbox cue jumps to TRACK
  SWEEP_GRID    serpentine over the geofence polygon at sweep_alt; cue jumps
  TRACK         bottom-cam bbox -> ground offset -> guided goto above the
                candidate (target clamped INSIDE the fence, always)
  APPROACH      descend the altitude stair while the cue is fresh; decode
                every frame; payload consensus -> TRANSMIT
  TRANSMIT      STATUSTEXT QR:<payload> xN to MP + ws event to UI
  DONE          post_action (RTL default | AUTO resume | LOITER)
  FAILSAFE      link/battery/timeout: best-effort RTL + loud logging

Two-level sensing: bbox cues (fast, steer flight) and payload consensus
(N matching decodes, triggers transmit). Front-cam cues are bearing-only:
yaw toward them and step forward until the bottom cam takes over.
"""
import logging
import threading
import time

import geo
from consensus import ConsensusBuffer

log = logging.getLogger("mission")


class Mission:
    def __init__(self, fc, rig, detector, hub, cfg):
        self.fc = fc
        self.rig = rig
        self.detector = detector
        self.hub = hub
        self.cfg = cfg
        m = cfg.get("mission", {})
        self.sweep_alt = float(m.get("sweep_alt_m", 15.0))
        self.approach_stair = [float(a) for a in m.get("approach_stair_m", [12.0, 9.0, 7.0])]
        self.grid_overlap = float(m.get("grid_overlap", 0.5))
        self.yaw_steps = int(m.get("yaw_steps", 12))
        self.yaw_settle_s = float(m.get("yaw_settle_s", 1.0))
        self.trigger_timeout_s = float(m.get("trigger_timeout_s", 600))
        self.search_timeout_s = float(m.get("search_timeout_s", 600))
        self.approach_timeout_s = float(m.get("approach_timeout_s", 240))
        self.min_batt_pct = float(m.get("min_batt_pct", 25))
        self.post_action = str(m.get("post_action", "RTL")).upper()
        self.arrive_m = float(m.get("arrive_m", 2.0))
        self.trigger_cmds = set(m.get("trigger_cmds", [216, 222, 42600]))
        self.trigger_peek_ahead = bool(m.get("trigger_peek_ahead", False))
        self.trigger_require_auto = bool(m.get("trigger_require_auto", True))
        d = cfg.get("detector", {})
        self.tile_bottom = bool(d.get("tile_bottom", True))
        tg = d.get("tile_grid", [3, 3])
        self.tile_rows, self.tile_cols = int(tg[0]), int(tg[1])
        self.full_decode_every = int(d.get("full_decode_every_n", 5))
        self.cue_frames = int(d.get("cue_frames", 2))

        self.phase = "BOOT"
        self.detail = "init"
        self.home = None
        self.fence = []
        self.plan = []
        self.sprayer_seqs = []
        self.payload = None
        self._cue = None          # (cam, x, y, w, h, ts)
        self._cue_hits = 0
        self._consensus = ConsensusBuffer(
            required_consecutive=int(cfg.get("decode", {}).get("required_streak", 3)),
            miss_tolerance=int(cfg.get("decode", {}).get("miss_tolerance", 1)))
        self._stop = threading.Event()
        self._takeover = threading.Event()
        self._abort = threading.Event()
        self._workers = []
        self._lock = threading.Lock()
        self._t_phase = time.time()
        self.stats = {"frames": 0, "cues": 0, "decodes": 0}

    # -- helpers --------------------------------------------------------
    def _set_phase(self, phase, detail=""):
        with self._lock:
            self.phase = phase
            self.detail = detail
            self._t_phase = time.time()
        log.info("phase -> %s (%s)", phase, detail)
        self.hub.push("fsm", {"phase": phase, "detail": detail})
        self.hub.push("log", {"level": "INFO", "msg": "mission: %s — %s" % (phase, detail)})

    def _log(self, level, msg):
        log.info("%s", msg)
        self.hub.push("log", {"level": level, "msg": msg})

    def status(self):
        with self._lock:
            cs = self._consensus.status()
            return {"phase": self.phase, "detail": self.detail,
                    "payload": self.payload, "tracking": cs["tracking"],
                    "streak": "%d/%d" % (cs["streak"], cs["required"]),
                    "home": self.home, "fence_n": len(self.fence),
                    "sprayer_seqs": list(self.sprayer_seqs),
                    "cams": self.rig.status(), "link": self.fc.link_ok(),
                    "stats": dict(self.stats), "ts": time.time()}

    def request_takeover(self):
        self._takeover.set()

    def request_abort(self):
        self._abort.set()

    def _clamp_to_fence(self, lat, lon, cur_lat, cur_lon):
        """Pull a goto target inside the geofence (binary shrink toward
        current position). Returns (lat, lon, inside: bool)."""
        if not self.fence or geo.point_in_polygon(lat, lon, self.fence):
            return lat, lon, True
        lo, la, ln = 0.0, cur_lat, cur_lon
        hi = 1.0
        for _ in range(10):
            mid = (lo + hi) / 2.0
            tlat = cur_lat + (lat - cur_lat) * mid
            tlon = cur_lon + (lon - cur_lon) * mid
            if geo.point_in_polygon(tlat, tlon, self.fence):
                lo, la, ln = mid, tlat, tlon
            else:
                hi = mid
        return la, ln, lo > 0.01

    # -- detection workers (one per camera) ------------------------------
    def _worker(self, cam):
        from decoder import decode_frame
        from detector import detect_tiles
        tiled = (self.tile_bottom and cam.facing == "bottom"
                 and self.detector.name != "classical")
        n = 0
        last_count = -1
        while not self._stop.is_set():
            frame, ts, count = cam.latest()
            if frame is None or count == last_count:
                time.sleep(0.05)
                continue
            last_count = count
            n += 1
            try:
                if tiled:
                    boxes = detect_tiles(self.detector, frame,
                                         rows=self.tile_rows, cols=self.tile_cols)
                else:
                    boxes = self.detector.detect(frame)
            except Exception as e:
                log.debug("%s detect failed: %r", cam.name, e)
                time.sleep(0.1)
                continue
            self.stats["frames"] += 1
            cam.set_overlay([(b.x, b.y, b.w, b.h, "qr %.2f" % b.conf) for b in boxes])
            payload = None
            if boxes:
                for b in boxes[:3]:
                    payload = decode_frame(frame, (b.x, b.y, b.w, b.h))
                    if payload:
                        break
            if payload is None and n % self.full_decode_every == 0:
                payload = decode_frame(frame)
            if payload:
                self.stats["decodes"] += 1
            newly, _st = self._consensus.update(payload)
            if newly:
                self._on_payload(payload, cam.name)
            # bbox cue (steers flight before any decode exists)
            if boxes and payload is None:
                b = max(boxes, key=lambda b: b.conf)
                self._on_cue(cam, b)
            elif not boxes:
                with self._lock:
                    if self._cue and self._cue[0] == cam.name and \
                            time.time() - self._cue[5] > 2.0:
                        self._cue = None
                        self._cue_hits = 0

    def _on_cue(self, cam, box):
        with self._lock:
            self._cue = (cam.name, box.x, box.y, box.w, box.h, time.time())
            self._cue_hits += 1
            hits = self._cue_hits
        if hits == self.cue_frames:
            self.stats["cues"] += 1
            self.hub.push("event", {"type": "qr_cue", "cam": cam.name,
                                    "box": [box.x, box.y, box.w, box.h]})
            self._log("WARN", "QR cue on %s (%s)" % (cam.name, cam.facing))

    def _on_payload(self, payload, cam_name):
        with self._lock:
            if self.payload is None:
                self.payload = payload
        self._log("WARN", "QR CONFIRMED via %s: %r" % (cam_name, payload))

    def _fresh_cue(self, max_age=2.0, facing=None):
        with self._lock:
            if not self._cue or time.time() - self._cue[5] > max_age:
                return None
            if facing:
                cam = self.rig.get(self._cue[0])
                if not cam or cam.facing != facing:
                    return None
            return self._cue

    # -- guided helpers ---------------------------------------------------
    def _goto_hold(self, lat, lon, alt, hold_s, arrive_m=None):
        """Send goto at 1 Hz for hold_s (or until arrival). Returns True
        when arrived / cue-prompted exit requested by caller polling."""
        arrive_m = arrive_m or self.arrive_m
        end = time.time() + hold_s
        while time.time() < end:
            if self._abort.is_set() or self._stop.is_set():
                return False
            try:
                self.fc.goto_global(lat, lon, alt)
                pos = self.fc.get_position(timeout=1.5)
            except Exception as e:
                log.debug("goto hold: %r", e)
                time.sleep(1.0)
                continue
            d = geo.haversine_m(pos["lat"], pos["lon"], lat, lon)
            if d <= arrive_m:
                return True
            time.sleep(1.0)
        return False

    def _battery_ok(self):
        try:
            b = self.fc.get_battery()
        except Exception:
            return True
        pct = b.get("pct")
        if pct is not None and pct < self.min_batt_pct:
            self._log("ERROR", "battery %s%% < %s%% — aborting search" % (pct, self.min_batt_pct))
            return False
        return True

    # -- main -------------------------------------------------------------
    def run(self):
        try:
            self._run()
        except Exception as e:
            log.exception("mission crashed: %r", e)
            self._set_phase("FAILSAFE", "crash: %s" % e)
        finally:
            self._stop.set()

    def _run(self):
        # WAIT_LINK
        self._set_phase("WAIT_LINK", "waiting for FC heartbeat")
        t0 = time.time()
        while not self.fc.link_ok():
            if time.time() - t0 > 60 or self._abort.is_set():
                self._set_phase("FAILSAFE", "no FC link")
                return
            time.sleep(0.5)
        # SNAPSHOT (while still in AUTO — home + fence + plan + sprayer)
        self._set_phase("SNAPSHOT", "reading home / fence / plan")
        try:
            self.home = self.fc.get_home(timeout=5.0)
            self._log("INFO", "home: %.7f, %.7f" % (self.home[0], self.home[1]))
        except Exception as e:
            self._set_phase("FAILSAFE", "no home: %s" % e)
            return
        try:
            self.fence = self.fc.read_fence(timeout=8.0)
        except Exception as e:
            self._log("WARN", "fence read failed: %s" % e)
            self.fence = []
        if len(self.fence) < 3:
            # no fence is fine on the bench (and off-nominal in the field):
            # the grid sweep falls back to a home-centered box.
            self._log("WARN", "no geofence on FC — grid sweep will use home box")
            self.fence = []
        else:
            w, h = geo.polygon_size_m(self.fence)
            self._log("INFO", "search area: %d fence verts, ~%.0f x %.0f m" % (
                len(self.fence), w, h))
        try:
            self.fc.set_message_interval(33, 5.0)    # GLOBAL_POSITION_INT
            self.fc.set_message_interval(147, 1.0)   # BATTERY_STATUS
            self.fc.set_message_interval(42, 2.0)    # MISSION_CURRENT
        except Exception:
            pass
        try:
            self.plan = self.fc.read_plan(timeout=8.0)
            self.sprayer_seqs = self.fc.find_sprayer_seqs(self.plan, self.trigger_cmds)
        except Exception as e:
            self._log("WARN", "plan read failed: %s" % e)
        if not self.sprayer_seqs:
            fb = self.cfg.get("mission", {}).get("trigger_seq")
            self.sprayer_seqs = [fb] if fb is not None else []
            self._log("WARN", "no sprayer cmd %s in plan — trigger_seq=%s" % (
                sorted(self.trigger_cmds), fb))
        else:
            self._log("INFO", "sprayer seqs: %s" % (self.sprayer_seqs,))
        # start detection workers + stream overlays
        for cam in self.rig.cams.values():
            t = threading.Thread(target=self._worker, args=(cam,),
                                 name="det-" + cam.name, daemon=True)
            t.start()
            self._workers.append(t)
        # WAIT_TRIGGER
        self._set_phase("WAIT_TRIGGER", "waiting for sprayer seqs %s" % (self.sprayer_seqs,))
        if not self._wait_trigger():
            return
        # TAKEOVER
        self._set_phase("TAKEOVER", "commanding GUIDED")
        try:
            self.fc.set_mode("GUIDED", timeout=8.0)
        except Exception as e:
            self._set_phase("FAILSAFE", "GUIDED refused: %s" % e)
            return
        t1 = time.time()
        while time.time() - t1 < 6.0 and self.fc.mode != "GUIDED":
            time.sleep(0.5)
        if self.fc.mode != "GUIDED":
            self._set_phase("FAILSAFE", "GUIDED not confirmed (mode=%s)" % self.fc.mode)
            return
        # hold + normalize to sweep altitude (takeover may precede 15 m)
        try:
            pos = self.fc.get_position()
            self._goto_hold(pos["lat"], pos["lon"], self.sweep_alt, 25.0, arrive_m=3.0)
        except Exception as e:
            log.warning("hold failed: %r", e)
        if self._abort.is_set():
            self._aborted()
            return
        # SEARCH
        if not self._sweep_yaw():
            return
        if not self._sweep_grid():
            return
        # found nothing in the pattern — one last transmit-less exit
        self._finish(found=False)

    def _wait_trigger(self):
        """Multi-cue trigger (aligned with the ftest references):
        1. MISSION_CURRENT reaches/passes a plan-scanned sprayer seq.
        2. Current item's command is a sprayer cmd (DO items don't emit
           ITEM_REACHED, so CURRENT + fetch is the reliable path).
        3. Optional peek-ahead: NEXT item is a sprayer cmd (claude3 style).
        4. STATUSTEXT sprayer backup. All gated on AUTO unless configured
           otherwise; manual POST /takeover always works."""
        first = min(self.sprayer_seqs) if self.sprayer_seqs else None
        if first is None:
            self._log("WARN", "no trigger seq — waiting for manual takeover only")
        t0 = time.time()
        q = self.fc.subscribe(["MISSION_CURRENT", "MISSION_ITEM_REACHED", "STATUSTEXT"])
        last_seq = None
        try:
            while time.time() - t0 < self.trigger_timeout_s:
                if self._abort.is_set():
                    self._set_phase("FAILSAFE", "aborted while waiting for trigger")
                    return False
                if self._takeover.is_set():
                    self._log("WARN", "manual takeover requested")
                    return True
                if not self.fc.link_ok():
                    time.sleep(0.5)
                    continue
                try:
                    m = q.get(timeout=1.0)
                except Exception:
                    continue
                if m.get_type() == "STATUSTEXT":
                    raw = m.text
                    txt = raw.decode("utf-8", "ignore") if isinstance(raw, bytes) else str(raw)
                    auto = (not self.trigger_require_auto) or self.fc.mode == "AUTO"
                    if "sprayer" in txt.lower() and auto:
                        self._log("WARN", "trigger: sprayer STATUSTEXT %r" % txt.strip()[:60])
                        return True
                    continue
                seq = getattr(m, "seq", None)
                if seq is None:
                    continue
                if m.get_type() == "MISSION_CURRENT" and seq != last_seq:
                    self._log("INFO", "mission current: %d" % seq)
                auto = (not self.trigger_require_auto) or self.fc.mode == "AUTO"
                if first is not None and seq >= first and auto:
                    self._log("WARN", "trigger: mission seq %d >= sprayer %d" % (seq, first))
                    return True
                if seq != last_seq:
                    last_seq = seq
                    if not auto:
                        continue
                    probe = [seq] + ([seq + 1] if self.trigger_peek_ahead else [])
                    for pr in probe:
                        try:
                            it = self.fc.get_mission_item(pr, timeout=1.5)
                        except Exception:
                            continue
                        if it is not None and it.command in self.trigger_cmds:
                            self._log("WARN", "trigger: item %d is sprayer cmd %d%s" % (
                                pr, it.command, " (peek-ahead)" if pr != seq else ""))
                            return True
            self._set_phase("FAILSAFE", "trigger timeout")
            return False
        finally:
            self.fc.unsubscribe(q)

    def _sweep_yaw(self):
        self._set_phase("SWEEP_YAW", "%d x %.0f deg stepped sweep" % (
            self.yaw_steps, 360.0 / self.yaw_steps))
        step = 360.0 / self.yaw_steps
        t0 = time.time()
        for i in range(self.yaw_steps):
            if self._abort.is_set():
                return self._aborted()
            if not self._battery_ok():
                return self._aborted()
            if self.payload:
                return self._transmit_and_finish()
            if time.time() - t0 > self.search_timeout_s:
                break
            if self._fresh_cue():
                return self._track_and_approach()
            try:
                self.fc.condition_yaw(step, speed_deg_s=20.0, relative=True)
            except Exception as e:
                log.warning("yaw step failed: %r", e)
            time.sleep(step / 20.0 + self.yaw_settle_s)
        return True

    def _sweep_grid(self):
        if self.payload:
            return self._transmit_and_finish()
        cam = self.rig.get("cam2") or self.rig.get("cam1")
        if cam is None:
            self._set_phase("FAILSAFE", "no camera for grid sweep")
            return False
        from geo import footprint_m
        fw, fh = footprint_m(self.sweep_alt, cam.hfov_deg, cam.size[0], cam.size[1])
        spacing = max(2.0, min(fw, fh) * (1.0 - self.grid_overlap))
        fence = self.fence
        if len(fence) < 3:
            # bench fallback (SITL has no fence): cover a home-centered box
            import math as _m
            half = float(self.cfg.get("mission", {}).get("nofence_half_m", 20.0))
            lat0, lon0 = self.home[0], self.home[1]
            dlat = half / 111320.0
            dlon = half / (111320.0 * max(0.2, _m.cos(_m.radians(lat0))))
            fence = [(lat0 - dlat, lon0 - dlon), (lat0 - dlat, lon0 + dlon),
                     (lat0 + dlat, lon0 + dlon), (lat0 + dlat, lon0 - dlon)]
            self._log("WARN", "no fence — covering %.0f m home box" % (half * 2))
        rows = geo.lawnmower_rows(fence, spacing,
                                  origin=(self.home[0], self.home[1]))
        self._set_phase("SWEEP_GRID", "%d legs, %.1fm spacing @ %.0fm" % (
            len(rows), spacing, self.sweep_alt))
        t0 = time.time()
        for i, (lat, lon) in enumerate(rows):
            if self._abort.is_set():
                return self._aborted()
            if not self._battery_ok():
                return self._aborted()
            if self.payload:
                return self._transmit_and_finish()
            if time.time() - t0 > self.search_timeout_s:
                self._log("WARN", "search timeout — finishing without payload")
                return True
            cue = self._fresh_cue()
            if cue:
                return self._track_and_approach()
            self._log("INFO", "grid leg %d/%d" % (i + 1, len(rows)))
            end = time.time() + 40.0
            while time.time() < end:
                if self._abort.is_set():
                    return self._aborted()
                if self.payload:
                    return self._transmit_and_finish()
                if self._fresh_cue():
                    return self._track_and_approach()
                try:
                    self.fc.goto_global(lat, lon, self.sweep_alt)
                    pos = self.fc.get_position(timeout=1.5)
                except Exception:
                    time.sleep(1.0)
                    continue
                if geo.haversine_m(pos["lat"], pos["lon"], lat, lon) <= self.arrive_m:
                    break
                time.sleep(1.0)
        return True

    def _track_and_approach(self):
        self._set_phase("TRACK", "steering to QR cue")
        t0 = time.time()
        stair_idx = 0
        while time.time() - t0 < self.approach_timeout_s:
            if self._abort.is_set():
                return self._aborted()
            if not self._battery_ok():
                return self._aborted()
            if self.payload:
                return self._transmit_and_finish()
            cue = self._fresh_cue(max_age=2.5)
            if cue is None:
                self._log("INFO", "cue lost — resuming grid")
                return self._sweep_grid()
            cam_name, x, y, w, h = cue[0], cue[1], cue[2], cue[3], cue[4]
            cam = self.rig.get(cam_name)
            try:
                pos = self.fc.get_position(timeout=1.5)
            except Exception:
                time.sleep(0.5)
                continue
            alt = pos["alt_rel"]
            if cam is not None and cam.facing == "bottom":
                cx, cy = x + w / 2.0, y + h / 2.0
                ex, ny = geo.nadir_pixel_to_ground_m(
                    cx, cy, cam.size[0], cam.size[1], cam.hfov_deg,
                    max(alt, 1.0), pos.get("hdg") or 0.0, cam.rotation_deg)
                olat, olon = self.home[0], self.home[1]
                ex0, ny0 = geo.latlon_to_enu(pos["lat"], pos["lon"], olat, olon)
                tlat, tlon = geo.enu_to_latlon(ex0 + ex, ny0 + ny, olat, olon)
                tlat, tlon, inside = self._clamp_to_fence(tlat, tlon, pos["lat"], pos["lon"])
                if not inside:
                    self._log("WARN", "cue projects outside fence — holding")
                    time.sleep(1.0)
                    continue
                # descend the stair as we center up
                off = (ex ** 2 + ny ** 2) ** 0.5
                if off < 1.5 and stair_idx < len(self.approach_stair):
                    self._set_phase("APPROACH", "descending to %.0fm" % self.approach_stair[stair_idx])
                    stair_idx += 1
                target_alt = self.approach_stair[stair_idx - 1] if stair_idx else max(alt, 5.0)
                target_alt = min(target_alt, alt)  # never climb on approach
                self.fc.goto_global(tlat, tlon, target_alt)
            else:
                # front cam: bearing-only — yaw toward it, step forward
                yaw = pos.get("hdg") or 0.0
                brg = geo.front_pixel_bearing_deg(x + w / 2.0, cam.size[0],
                                                  cam.hfov_deg, yaw)
                try:
                    self.fc.condition_yaw(brg, speed_deg_s=25.0, relative=False)
                except Exception:
                    pass
                step_m = 3.0
                t = __import__("math").radians(brg)
                olat, olon = self.home[0], self.home[1]
                ex0, ny0 = geo.latlon_to_enu(pos["lat"], pos["lon"], olat, olon)
                tlat, tlon = geo.enu_to_latlon(
                    ex0 + step_m * __import__("math").sin(t),
                    ny0 + step_m * __import__("math").cos(t), olat, olon)
                tlat, tlon, inside = self._clamp_to_fence(tlat, tlon, pos["lat"], pos["lon"])
                if inside:
                    self.fc.goto_global(tlat, tlon, alt)
            time.sleep(0.5)
        self._log("WARN", "approach timeout — resuming grid")
        return self._sweep_grid()

    def _transmit_and_finish(self):
        self._set_phase("TRANSMIT", "relaying %r" % (self.payload,))
        from qr_relay import relay_qr
        rcfg = self.cfg.get("relay", {})
        relay_qr(self.payload, self.fc, self.hub,
                 interval_s=float(rcfg.get("interval_s", 2.0)),
                 window_s=float(rcfg.get("window_s", 15.0)))
        try:
            pos = self.fc.get_position(timeout=2.0)
            hold_until = time.time() + float(rcfg.get("window_s", 15.0))
            while time.time() < hold_until and not self._abort.is_set():
                try:
                    self.fc.goto_global(pos["lat"], pos["lon"], pos["alt_rel"])
                except Exception:
                    pass
                time.sleep(1.0)
        except Exception:
            pass
        self._finish(found=True)
        return False  # stop search loops

    def _aborted(self):
        self._log("ERROR", "abort — commanding RTL")
        try:
            self.fc.set_mode("RTL", timeout=6.0)
        except Exception as e:
            self._log("ERROR", "RTL failed: %s" % e)
        self._set_phase("FAILSAFE", "aborted by request/battery")
        return False

    def _finish(self, found):
        if self.payload and not found:
            found = True
        self._log("WARN", "mission %s — post_action %s" % (
            "PAYLOAD %r" % self.payload if found else "no QR found", self.post_action))
        try:
            if self.post_action in ("RTL", "AUTO", "LOITER", "LAND"):
                self.fc.set_mode(self.post_action, timeout=8.0)
        except Exception as e:
            self._log("ERROR", "post_action %s failed: %s" % (self.post_action, e))
        self._set_phase("DONE", "payload=%s" % (self.payload,))
