"""QR-hunt mission: DO_SPRAYER trigger -> GUIDED takeover -> search -> decode.

Flow (drone is at ~15 m over home in AUTO when we take over):
  WAIT_TRIGGER  plan scan (sprayer cmds 216/222/42600) + MISSION_CURRENT;
                peek-ahead item fetch + STATUSTEXT backup [or POST /takeover]
  TAKEOVER      set GUIDED, verify; snapshot already holds home + fence
  SWEEP_YAW     12 stepped 30° yaws with settle pauses (sharp frames beat
                motion blur); any fresh bbox cue jumps to TRACK
  SWEEP_GRID    serpentine over the geofence polygon at sweep_alt, then a
                best-effort re-sweep at resweep_alt_m; cue jumps to TRACK
  FALLBACK      blob hypotheses (top-3, verified on descent, own 60 s
                cap): steer-only, never transmits; a real cue hands back
                to TRACK, a decode to TRANSMIT
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
from qr_filter import QrFilter, apply_preset

log = logging.getLogger("mission")


class Mission:
    def __init__(self, fc, rig, detector, hub, cfg):
        self.fc = fc
        self.rig = rig
        self.detector = detector
        self.hub = hub
        self.cfg = cfg
        m = cfg.get("mission", {})
        # --- arena/test1: configurable max height via config.yaml ---
        # flight.max_alt_m or mission.max_alt_m controls the ceiling (5/10/15 m etc)
        flight_cfg = cfg.get("flight", {})
        raw_max = m.get("max_alt_m", flight_cfg.get("max_alt_m", 15.0))
        try:
            import geo as _geo_check
            self.max_alt_m = _geo_check.validate_max_alt(raw_max, default=15.0)
        except Exception:
            self.max_alt_m = float(raw_max) if raw_max else 15.0
        # sweep_alt is clamped to max_alt
        raw_sweep = float(m.get("sweep_alt_m", self.max_alt_m))
        self.sweep_alt = max(1.0, min(self.max_alt_m, raw_sweep))
        # approach stair also clamped to max_alt
        raw_stair = m.get("approach_stair_m", [12.0, 9.0, 7.0])
        self.approach_stair = [max(1.0, min(self.max_alt_m, float(a))) for a in raw_stair]
        # keep stair sorted descending for approach
        self.approach_stair = sorted(self.approach_stair, reverse=True)
        self.grid_overlap = float(m.get("grid_overlap", 0.3))
        self.edge_margin_m = float(m.get("edge_margin_m", 2.0))
        self.resweep_alt_m = float(m.get("resweep_alt_m", 10.0))
        # clamp resweep too
        if self.resweep_alt_m > 0:
            self.resweep_alt_m = max(1.0, min(self.max_alt_m, self.resweep_alt_m))
        # coverage mode: fov_optimal uses camera FOV to divide area
        self.coverage_mode = str(m.get("coverage_mode", "fov_optimal"))
        self.min_overlap = float(m.get("min_overlap", 0.2))
        self.search_speed_ms = float(m.get("search_speed_ms", 2.5))
        self.yaw_steps = int(m.get("yaw_steps", 12))
        self.yaw_settle_s = float(m.get("yaw_settle_s", 1.0))
        self.trigger_timeout_s = float(m.get("trigger_timeout_s", 600))
        self.search_timeout_s = float(m.get("search_timeout_s", 600))
        self.approach_timeout_s = float(m.get("approach_timeout_s", 240))
        # WAIT_LINK budget. main.py brings the FC link up on its own thread
        # (retrying) while the UI is already served, so this only has to
        # outlast a slow SITL start / a late MP connection, not a hard fail.
        self.link_timeout_s = float(m.get("link_timeout_s", 600.0))
        self.min_batt_pct = float(m.get("min_batt_pct", 25))
        self.post_action = str(m.get("post_action", "RTL")).upper()
        self.arrive_m = float(m.get("arrive_m", 2.0))
        # 216 = DO_SPRAYER (what MP writes), 222 = ftest fallback,
        # 223 = ADDC reference stack's sprayer marker, 42600 = ftest custom.
        self.trigger_cmds = set(m.get("trigger_cmds", [216, 222, 223, 42600]))
        self.trigger_peek_ahead = bool(m.get("trigger_peek_ahead", False))
        self.trigger_require_auto = bool(m.get("trigger_require_auto", True))
        self._advancing = False  # latched by _wait_trigger when CURRENT moves
        d = cfg.get("detector", {})
        self.tile_bottom = bool(d.get("tile_bottom", True))
        tg = d.get("tile_grid", [3, 3])
        self.tile_rows, self.tile_cols = int(tg[0]), int(tg[1])
        self.full_decode_every = int(d.get("full_decode_every_n", 5))
        self.cue_frames = int(d.get("cue_frames", 2))
        # ---- real_pi fake-QR filter + environment presets ---------------
        # The window-grill fix: every detector box must pass conf/size/
        # aspect gates before it can drive a cue or an overlay (see
        # qr_filter.py). detector.preset selects the active environment
        # preset; POST /api/qr/preset switches it live (and the webcam
        # bench tool shares the SAME preset table from config.yaml).
        self.qf = QrFilter.from_config(cfg)
        self.preset_name = str(d.get("preset", "day"))
        try:
            if apply_preset(self, self.preset_name, cfg=cfg) is None:
                # unknown preset in config — fall back to the built-in
                # `day` preset rather than flying with a broken config.
                self.preset_name = "day"
                apply_preset(self, "day", cfg=cfg)
        except Exception as e:
            log.warning("preset %r failed to apply: %r", self.preset_name, e)

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
        self._search_t0 = None     # search-budget clock (yaw+grid+resweep share it)
        self._speed_par = None     # (name, scale, orig_raw): cruise-speed snapshot to restore
        # initialised here (not only in reset()) so a fresh Mission and a
        # reset Mission have identical shape — code can read them unguarded.
        self._target_last = None   # (ts, lat, lon) of the last guided target
        self._offmode_n = 0        # consecutive off-mode heartbeat reads
        self._blob_last_push = 0.0
        b = cfg.get("blob", {})
        self.blob_enabled = bool(b.get("enabled", True))
        self.blob_top_k = int(b.get("top_k", 3))
        self.blob_per_s = float(b.get("per_candidate_s", 20.0))
        self.blob_cap_s = float(b.get("global_cap_s", 60.0))
        self.blob_observe_s = float(b.get("observe_every_s", 2.0))
        self.blob_cfg = dict(b)
        self._blob_obs = []        # passive ground-projected sightings
        self._blob_last_obs = 0.0
        cvcfg = cfg.get("coverage", {})
        self.cov_cell_m = float(cvcfg.get("cell_m", 5.0))
        self.cov_every_s = float(cvcfg.get("push_every_s", 5.0))
        self._cov = set()            # (ix, iy) searched cells
        self._cov_hot = set()        # genuinely-new cells since last push
        self._cov_last_push = 0.0
        rcfg = cfg.get("relay", {})
        from store_forward import StoreForward
        self.store_fwd = StoreForward(
            self.fc, lambda: self.hub,
            path=str(rcfg.get("store_path", "logs/pending_results.jsonl")),
            poll_s=float(rcfg.get("flush_poll_s", 5.0)),
            enabled=bool(rcfg.get("store_forward", True)))
        # ---- real_pi two-stage QR delivery (NO DELAY to Mission Planner) --
        # Stage 1: QR_SEEN:<payload> STATUSTEXT the moment a payload decodes
        # in ONE frame — MP Messages tab gets it in ~one telemetry tick,
        # before the confirm-streak.  Stage 2: QR:<payload> on confirm
        # (streak from decode.required_streak) + the resend window.
        self.relay_announce = bool(rcfg.get("announce", True))
        self.relay_announce_window_s = float(
            rcfg.get("announce_window_s", 6.0))
        self.relay_announce_interval_s = float(
            rcfg.get("announce_interval_s", 1.5))
        self.relay_on_confirm = bool(rcfg.get("on_confirm", True))
        self._announced = set()      # payloads already QR_SEEN-reports
        self._confirmed_relayed = False

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
            # arena/test1: expose max_alt and FOV coverage info
            cam = self.rig.get("cam2") or self.rig.get("cam1")
            fov_info = {}
            if cam:
                try:
                    fw, fh = geo.footprint_m(self.sweep_alt, cam.hfov_deg, cam.size[0], cam.size[1])
                    fov_info = {"cam": cam.name, "hfov_deg": cam.hfov_deg,
                                "footprint_w_m": round(fw, 1), "footprint_h_m": round(fh, 1),
                                "alt_m": self.sweep_alt, "max_alt_m": self.max_alt_m}
                except Exception:
                    fov_info = {"max_alt_m": getattr(self, "max_alt_m", 15.0)}
            return {"phase": self.phase, "detail": self.detail,
                    "payload": self.payload, "tracking": cs["tracking"],
                    "streak": "%d/%d" % (cs["streak"], cs["required"]),
                    "home": self.home, "fence_n": len(self.fence),
                    "sprayer_seqs": list(self.sprayer_seqs),
                    "cams": self.rig.status(), "link": self.fc.link_ok(),
                    "stats": dict(self.stats), "ts": time.time(),
                    "max_alt_m": getattr(self, "max_alt_m", 15.0),
                    "sweep_alt_m": getattr(self, "sweep_alt", 15.0),
                    "fov": fov_info,
                    "coverage_mode": getattr(self, "coverage_mode", "fov_optimal"),
                    "qr_preset": getattr(self, "preset_name", "day"),
                    "qr_filter": getattr(self, "qf", None) and self.qf.as_dict()}

    def set_preset(self, name):
        """Runtime environment switch (POST /api/qr/preset). Returns the
        applied preset dict, or None for an unknown name."""
        p = apply_preset(self, name, cfg=self.cfg)
        if p is not None:
            try:
                self.hub.push("event", {"type": "qr_preset", "preset": name,
                                        "preset_cfg": p})
            except Exception:
                pass
            self._log("WARN", "QR preset -> %s (%s)" % (name, p.get("desc", "")))
        return p

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
            # real_pi fake-QR filter: grill/speck/odd-aspect boxes die HERE,
            # before they can touch the overlay or raise a flight cue.
            boxes = self.qf.filter_boxes(boxes)
            payload = None
            payload_box = None
            if boxes:
                for b in boxes[:3]:
                    payload = decode_frame(frame, (b.x, b.y, b.w, b.h))
                    if payload:
                        payload_box = b
                        break
            if payload is None and n % self.full_decode_every == 0:
                payload = decode_frame(frame)
            if payload:
                self.stats["decodes"] += 1
            # overlay: green-ish labels for survivors, and the decoded box
            # gets the payload text (matches the webcam bench tool).
            cam.set_overlay([(b.x, b.y, b.w, b.h,
                              "QR %.2f %s" % (b.conf, payload) if b is payload_box
                              else "qr %.2f" % b.conf) for b in boxes])
            # real_pi STAGE 1 (no-delay): the moment a payload decodes in a
            # SINGLE frame — before the confirm-streak has any say — put
            # QR_SEEN:<payload> on STATUSTEXT so Mission Planner's Messages
            # tab has it within one telemetry tick. Once per payload.
            if (self.relay_announce and payload
                    and payload not in self._announced):
                self._announce_qr(payload)
            newly, _st = self._consensus.update(payload)
            if newly:
                self._on_payload(payload, cam.name)
            # bbox cue (steers flight before any decode exists).
            # real_pi: strict presets (ground/night/kabaddi) set
            # require_decode_for_cue — then ONLY a box that actually decoded
            # to a payload raises a cue, so a window grill can never steer
            # the drone. Default presets keep the fast bbox cue (survivors
            # are geometry-filtered, and the flight path still needs cues
            # at 15 m where the QR cannot decode yet).
            if self.qf.require_decode_for_cue:
                if boxes and payload_box is not None:
                    self._on_cue(cam, payload_box)
                elif not boxes:
                    self._clear_stale_cue(cam)
            else:
                if boxes and payload is None:
                    b = max(boxes, key=lambda b: b.conf)
                    self._on_cue(cam, b)
                elif not boxes:
                    self._clear_stale_cue(cam)

    def _clear_stale_cue(self, cam):
        """Drop this camera's cue once it has been box-less for > 2 s."""
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

    def _announce_qr(self, payload):
        """Stage 1 of the two-stage delivery: INSTANT QR_SEEN report.

        Fire-and-forget on purpose — the announce must never slow the
        detection loop (a 1.5 s GPS grab is the only wait, bounded).
        """
        with self._lock:
            if payload in self._announced:
                return
            self._announced.add(payload)
        try:
            gps = None
            try:
                p = self.fc.get_position(timeout=1.5)
                gps = (p["lat"], p["lon"], p.get("alt_rel"))
            except Exception:
                pass
            from qr_relay import announce
            announce(payload, fc=self.fc, hub=self.hub, gps=gps,
                     window_s=self.relay_announce_window_s,
                     interval_s=self.relay_announce_interval_s)
            self._log("WARN", "QR_SEEN sent to MP (instant, pre-confirm): %r"
                      % payload)
        except Exception as e:
            log.warning("qr announce failed: %r", e)

    def _on_payload(self, payload, cam_name):
        with self._lock:
            if self.payload is None:
                self.payload = payload
            already_relayed = self._confirmed_relayed
        self._log("WARN", "QR CONFIRMED via %s: %r" % (cam_name, payload))
        # real_pi STAGE 2: the official QR:<payload> hits MP the moment the
        # confirm-streak completes — NOT after the approach descent. The
        # relay window resends it while the mission keeps flying.
        if (self.relay_on_confirm and not already_relayed
                and not self._confirmed_relayed):
            self._confirmed_relayed = True
            try:
                from qr_relay import relay_qr
                rcfg = self.cfg.get("relay", {})
                _gps = None
                try:
                    _p = self.fc.get_position(timeout=1.5)
                    _gps = (_p["lat"], _p["lon"], _p.get("alt_rel"))
                except Exception:
                    pass
                relay_qr(self.payload, self.fc, self.hub,
                         interval_s=float(rcfg.get("interval_s", 2.0)),
                         window_s=float(rcfg.get("window_s", 15.0)),
                         store=self.store_fwd.store if self.store_fwd.enabled
                         else None,
                         gps=_gps)
                self._log("WARN", "QR relay fired on confirm (MP gets it "
                                  "now): %r" % self.payload)
            except Exception as e:
                log.warning("confirm relay failed: %r", e)

    def _push_target(self, lat, lon, conf=None, cam=None, box=None,
                     inside=True):
        """`target_detected` on the UI contract — the map draws the marker.

        Throttled to one push per 2 s unless the candidate actually moved
        (>1 m): TRACK ticks at 2 Hz and the UI only needs to see the target
        travel, not every control cycle.
        """
        if lat is None or lon is None:
            return
        now = time.time()
        with self._lock:
            last = getattr(self, "_target_last", None)
            if last and (now - last[0]) < 2.0:
                try:
                    if geo.haversine_m(last[1], last[2], lat, lon) < 1.0:
                        return
                except Exception:
                    return
            self._target_last = (now, lat, lon)
        try:
            self.hub.push("event", {"type": "target_detected", "lat": lat,
                                    "lon": lon, "confidence": conf, "cam": cam,
                                    "bbox": box, "inside_fence": bool(inside),
                                    "phase": self.phase})
        except Exception:
            pass

    def stop(self):
        """Ask every loop to wind down (called on shutdown)."""
        self._abort.set()
        self._stop.set()

    def reset(self):
        """Clear per-run state so `run()` can be called again in the same
        process (bench re-runs, and the auto-restart after a no-link
        FAILSAFE). Threads/objects owned elsewhere (fc, rig, detector, hub,
        store) are deliberately kept."""
        with self._lock:
            self.phase, self.detail = "BOOT", "reset"
            self.home = None
            self.fence = []
            self.plan = []
            self.sprayer_seqs = []
            self.payload = None
            self._cue = None
            self._cue_hits = 0
            self._advancing = False
            self._target_last = None
            self._workers = []
            self._search_t0 = None
            self._speed_par = None
            self._offmode_n = 0
            self.stats = {"frames": 0, "cues": 0, "decodes": 0}
            self._cov = set()
            self._cov_hot = set()
            self._cov_last_push = 0.0
            self._blob_obs = []
            self._blob_last_push = 0.0
            self._blob_last_obs = 0.0
            self._confirmed_relayed = False   # new hunt may re-relay QR
            self._t_phase = time.time()
        # NOTE: self._announced is deliberately NOT reset — a re-run hunt
        # must not re-spam MP with QR_SEEN lines for the same payload.
        try:
            self._consensus.reset()
        except Exception:
            pass
        self._stop.clear()
        self._takeover.clear()
        self._abort.clear()
        log.info("mission state reset — ready for another run")

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

    def _guided_ok(self):
        """True while the vehicle is still flying OUR search (GUIDED).

        Anything else 3 ticks straight (external RTL, failsafe, user mode
        bump) trips the search — better than burning 40 s timeouts per leg
        while the drone RTL's away (seen 2026-09-14: 13 min wasted).
        """
        try:
            guided = (self.fc.mode == "GUIDED")
        except Exception:
            guided = False
        if guided:
            self._offmode_n = 0
            return True
        self._offmode_n = getattr(self, "_offmode_n", 0) + 1
        return self._offmode_n < 3  # tolerate 1-2 stale reads

    # -- main -------------------------------------------------------------
    def run(self):
        try:
            self.store_fwd.start()
            self._run()
        except Exception as e:
            log.exception("mission crashed: %r", e)
            self._set_phase("FAILSAFE", "crash: %s" % e)
        finally:
            try:
                self.store_fwd.stop()
            except Exception:
                pass
            self._stop.set()

    def _run(self):
        self._cov = set()
        self._cov_hot = set()
        self._cov_last_push = 0.0
        # WAIT_LINK — link_timeout_s: 0 (or negative) means "wait forever"
        # (real-flight default: the FC link is brought up on its own thread
        # and the mission simply waits; the UI stays up the whole time).
        link_budget = self.link_timeout_s if self.link_timeout_s > 0 \
            else float("inf")
        self._set_phase("WAIT_LINK", "waiting for FC heartbeat")
        t0, next_note = time.time(), 15.0
        while not self.fc.link_ok():
            waited = time.time() - t0
            if waited > link_budget or self._abort.is_set():
                self._set_phase("FAILSAFE", "no FC link")
                return
            if waited > next_note:
                next_note += 15.0
                self._log("WARN", "still no FC heartbeat after %.0fs — the UI "
                                  "link stays up; SITL/Gazebo + MP must be "
                                  "connected first (see SIM_GUIDE.md §6)" % waited)
            time.sleep(0.5)
        # SNAPSHOT (while still in AUTO — home + fence + plan + sprayer)
        self._set_phase("SNAPSHOT", "reading home / fence / plan")
        # streams FIRST: Telem-class ports are heartbeat-only until rates
        # are set — home/plan/trigger all depend on these.
        try:
            self.fc.start_streams()
        except Exception:
            pass
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
        # contract event: the UI logs the plan the Pi actually sees
        try:
            self.hub.push("event", {
                "type": "plan_synced", "items": len(self.plan or []),
                "trigger_seq": (self.sprayer_seqs[0] if self.sprayer_seqs
                                else None),
                "fence_verts": len(self.fence or []),
                "home": list(self.home) if self.home else None})
        except Exception:
            pass
        # snapshot cruise speed so the search can fly search_speed_ms and
        # hand the FC back exactly as found (best-effort; a dead param
        # channel must not brick the mission).
        self._snapshot_speed()
        # start detection workers + stream overlays
        for cam in self.rig.cams.values():
            if getattr(cam, "display_only", False):
                continue  # mirror tiles are UI-only; the source cam feeds detection
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
        while time.time() - t1 < 8.0 and self.fc.mode != "GUIDED":
            time.sleep(0.5)
        if self.fc.mode != "GUIDED":
            # DO_SET_MODE was ACKed above — the FC accepted GUIDED. A missing
            # heartbeat echo must not brick the mission the way the stale
            # AUTO gate bricked the trigger, so proceed: the hold-arrival
            # check below is the real confirmation that GUIDED is live.
            log.warning("GUIDED not echoed by heartbeat (mode=%s age=%.1fs) — "
                        "proceeding on COMMAND_ACK; hold arrival is the check",
                        self.fc.mode, self.fc.mode_age())
        # hold + normalize to sweep altitude (takeover may precede 15 m)
        try:
            pos = self.fc.get_position()
            arrived = self._goto_hold(pos["lat"], pos["lon"], self.sweep_alt, 25.0, arrive_m=3.0)
            if not arrived:
                log.warning("takeover hold did not converge (mode=%s) — "
                            "GUIDED may be inactive; continuing to SEARCH anyway",
                            self.fc.mode)
        except Exception as e:
            log.warning("hold failed: %r", e)
        if self._abort.is_set():
            self._aborted()
            return
        # SEARCH (yaw + grid + resweep share one search_timeout_s budget)
        self._search_t0 = time.time()
        if self.search_speed_ms > 0:
            if self._speed_par is None:
                self._log("WARN", "no speed param snapshot — flying FC default")
            else:
                _nm, _sc, _raw = self._speed_par
                try:
                    got = self.fc.set_param(_nm, self.search_speed_ms * _sc)
                    self._log("WARN", "search speed: %s -> %s (echo %s)" % (
                        _nm, self.search_speed_ms * _sc, got))
                except Exception as e:
                    self._log("WARN", "search speed set failed (%s) — flying FC default" % e)
        if not self._sweep_yaw():
            return
        if not self._sweep_grid():
            return
        if self.resweep_alt_m > 0 and not self.payload and not self._search_expired():
            self._log("WARN", "grid empty @ %.0fm — re-sweeping @ %.0fm" % (
                self.sweep_alt, self.resweep_alt_m))
            if not self._sweep_grid(alt=self.resweep_alt_m):
                return
        if not self.payload and self.blob_enabled:
            if not self._blob_fallback():
                return
        # found nothing in the pattern — one last transmit-less exit
        self._finish(found=False)

    def _gate_state(self):
        """Trigger gate state: (gate_open, gate_name).

        - AUTO: vehicle heartbeat says AUTO and is fresh (< 5 s) — the
          ADDC reference shape.
        - DEADMAN: vehicle heartbeats are stale/missing BUT the mission
          visibly advanced since WAIT began. An advancing mission IS an
          executing mission, so a sprayer cue is trustworthy even when
          the mode channel is broken (this exact failure cost us a full
          bench flight: MP heartbeats held fc.mode off AUTO throughout).
        - override: trigger_require_auto=false (bench escape hatch).
        """
        fresh_auto = (self.fc.mode == "AUTO" and self.fc.mode_age() < 5.0)
        if not self.trigger_require_auto:
            return True, "override"
        if fresh_auto:
            return True, "AUTO"
        if self.fc.mode_age() >= 5.0 and self._advancing:
            return True, "DEADMAN"
        return False, "closed(mode=%s age=%.1fs advancing=%s)" % (
            self.fc.mode, self.fc.mode_age(), self._advancing)

    def _fire_trigger(self, why):
        self._log("WARN", "trigger: %s" % why)
        try:
            self.fc.send_statustext("TRIGGER: %s" % why[:40])
        except Exception:
            pass
        self._log("WARN", "hb sources: %s" % (self.fc.hb_sources(),))
        return True

    def _wait_trigger(self):
        """Multi-cue trigger (shaped after the ADDC reference MONITOR_AUTO):
        1. PRIMARY: autopilot STATUSTEXT "Mission: N Sprayer" (vehicle
           sysid only) — ArduPilot broadcasts this when DO_SPRAYER fires.
        2. MISSION_CURRENT reaches/passes a plan-scanned sprayer seq.
        3. Current item's command is a sprayer cmd (DO items don't emit
           ITEM_REACHED, so CURRENT + fetch is the reliable path).
        4. Optional peek-ahead: NEXT item is a sprayer cmd (claude3 style).
        Cues fire through the AUTO gate (fresh vehicle heartbeat) or the
        DEADMAN gate (stale mode channel + visibly advancing mission);
        manual POST /takeover always works."""
        first = min(self.sprayer_seqs) if self.sprayer_seqs else None
        if first is None:
            self._log("WARN", "no trigger seq — waiting for manual takeover only")
        # Mid-flight-reboot safety (reference MONITOR_AUTO entry): booting
        # into an already-GUIDED FC means a prior run died mid-takeover.
        if self.fc.mode == "GUIDED" and self.fc.mode_age() < 5.0:
            self._log("ERROR", "FC already GUIDED at boot (prior run died?) — commanding RTL")
            try:
                self.fc.set_mode("RTL", timeout=5.0)
            except Exception as e:
                self._log("ERROR", "reboot-RTL failed: %s" % e)
            self._set_phase("FAILSAFE", "reboot in GUIDED — RTL commanded")
            return False
        self._log("INFO", "hb sources at WAIT: %s" % (self.fc.hb_sources(),))
        self._advancing = False  # latched once CURRENT visibly moves
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
                    if "sprayer" in txt.lower():
                        try:
                            src = m.get_srcSystem()
                        except Exception:
                            src = -1
                        if src != self.fc.target_system:
                            self._log("INFO", "sprayer text from non-vehicle sysid %d: %r" % (
                                src, txt.strip()[:60]))
                            continue
                        gate_open, gate = self._gate_state()
                        self._log("WARN", "sprayer STATUSTEXT %r (gate=%s)" % (
                            txt.strip()[:60], gate))
                        if gate_open:
                            return self._fire_trigger("sprayer STATUSTEXT @seq %s" % last_seq)
                    continue
                seq = getattr(m, "seq", None)
                if seq is None:
                    continue
                if m.get_type() == "MISSION_CURRENT" and seq != last_seq:
                    self._log("INFO", "mission current: %d" % seq)
                changed = (seq != last_seq)
                if changed:
                    if last_seq is not None and not self._advancing:
                        self._advancing = True
                        self._log("INFO", "mission advancing (%s -> %d)" % (last_seq, seq))
                    last_seq = seq
                gate_open, gate = self._gate_state()
                if first is not None and seq >= first and gate_open:
                    return self._fire_trigger("mission seq %d >= sprayer %d (gate=%s)" % (
                        seq, first, gate))
                if changed:
                    if not gate_open:
                        continue
                    probe = [seq] + ([seq + 1] if self.trigger_peek_ahead else [])
                    for pr in probe:
                        try:
                            it = self.fc.get_mission_item(pr, timeout=1.5)
                        except Exception:
                            continue
                        if it is not None and it.command in self.trigger_cmds:
                            return self._fire_trigger("item %d is sprayer cmd %d%s" % (
                                pr, it.command, " (peek-ahead)" if pr != seq else ""))
            self._set_phase("FAILSAFE", "trigger timeout")
            return False
        finally:
            self.fc.unsubscribe(q)

    def _search_expired(self):
        """One search budget shared by yaw + grid + resweep (approach runs
        on its own approach_timeout_s so a find never starves the stair)."""
        return (self._search_t0 is not None
                and time.time() - self._search_t0 > self.search_timeout_s)

    def _sweep_yaw(self):
        self._set_phase("SWEEP_YAW", "%d x %.0f deg stepped sweep" % (
            self.yaw_steps, 360.0 / self.yaw_steps))
        step = 360.0 / self.yaw_steps
        for i in range(self.yaw_steps):
            if self._abort.is_set():
                return self._aborted()
            if not self._battery_ok():
                return self._aborted()
            if not self._guided_ok():
                return self._mode_tripped()
            if self.payload:
                return self._transmit_and_finish()
            if self._search_expired():
                break
            if self._fresh_cue():
                return self._track_and_approach()
            try:
                self.fc.condition_yaw(step, speed_deg_s=20.0, relative=True)
            except Exception as e:
                log.warning("yaw step failed: %r", e)
            time.sleep(step / 20.0 + self.yaw_settle_s)
        return True

    def _sweep_grid(self, alt=None):
        if self.payload:
            return self._transmit_and_finish()
        cam = self.rig.get("cam2") or self.rig.get("cam1")
        if cam is None:
            self._set_phase("FAILSAFE", "no camera for grid sweep")
            return False
        # arena/test1: alt clamped to max_alt_m from config.yaml
        raw_alt = float(alt) if alt else self.sweep_alt
        alt = geo.clamp_altitude(raw_alt, self.max_alt_m,
                                 min_alt_m=float(self.cfg.get("flight", {}).get("min_alt_m", 1.0)))
        # FOV-optimal coverage: use camera footprint at this alt
        # cam.txt: front 66° HFOV, bottom 100° HFOV (113° diagonal)
        # Divide area according to cam FOV to cover max possible
        if self.coverage_mode == "fov_optimal":
            # use geo.lawnmower_rows_fov_optimal for max coverage
            try:
                plan = geo.lawnmower_rows_fov_optimal(
                    self.fence if len(self.fence) >= 3 else [],
                    alt, cam.hfov_deg, cam.size[0], cam.size[1],
                    origin=(self.home[0], self.home[1]) if self.home else None,
                    edge_margin_m=self.edge_margin_m,
                    overlap=self.grid_overlap,
                    max_alt_m=self.max_alt_m)
                # plan may be empty if no fence — fallback handled below
                fw = plan.get("footprint_w_m", 0)
                fh = plan.get("footprint_h_m", 0)
                spacing = plan.get("spacing_m", max(2.0, min(fw, fh) * (1.0 - self.grid_overlap)))
                self._log("INFO", "FOV-optimal: alt=%.1fm footprint %.1fx%.1fm spacing %.1fm cam %s %.0f°HFOV (max_alt %.1fm)" % (
                    alt, fw, fh, spacing, cam.name, cam.hfov_deg, self.max_alt_m))
            except Exception as e:
                log.debug("fov_optimal failed: %r, falling back", e)
                fw, fh = geo.footprint_m(alt, cam.hfov_deg, cam.size[0], cam.size[1])
                spacing = max(2.0, min(fw, fh) * (1.0 - self.grid_overlap))
        else:
            fw, fh = geo.footprint_m(alt, cam.hfov_deg, cam.size[0], cam.size[1])
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
            self._log("WARN", "no fence — covering %.0f m home box (max_alt %.1fm, footprint %.1fx%.1fm)" % (
                half * 2, self.max_alt_m, fw, fh))

        # Use FOV-optimal if available and fence exists
        if self.coverage_mode == "fov_optimal" and len(fence) >= 3:
            try:
                opt = geo.lawnmower_rows_fov_optimal(
                    fence, alt, cam.hfov_deg, cam.size[0], cam.size[1],
                    origin=(self.home[0], self.home[1]),
                    edge_margin_m=self.edge_margin_m,
                    overlap=self.grid_overlap,
                    max_alt_m=self.max_alt_m)
                rows = opt["waypoints"]
            except Exception:
                rows = geo.lawnmower_rows(fence, spacing,
                                          origin=(self.home[0], self.home[1]),
                                          edge_margin_m=self.edge_margin_m)
        else:
            rows = geo.lawnmower_rows(fence, spacing,
                                      origin=(self.home[0], self.home[1]),
                                      edge_margin_m=self.edge_margin_m)
        self._set_phase("SWEEP_GRID", "%d legs, %.1fm spacing @ %.0fm, %.0fm edge" % (
            len(rows), spacing, alt, self.edge_margin_m))
        for i, (lat, lon) in enumerate(rows):
            if self._abort.is_set():
                return self._aborted()
            if not self._battery_ok():
                return self._aborted()
            if self.payload:
                return self._transmit_and_finish()
            if self._search_expired():
                self._log("WARN", "search budget spent — finishing without payload")
                return True
            cue = self._fresh_cue()
            if cue:
                return self._track_and_approach()
            self._log("INFO", "grid leg %d/%d" % (i + 1, len(rows)))
            end = time.time() + 40.0
            while time.time() < end:
                if self._abort.is_set():
                    return self._aborted()
                if not self._guided_ok():
                    return self._mode_tripped()
                if self.payload:
                    return self._transmit_and_finish()
                if self._fresh_cue():
                    return self._track_and_approach()
                try:
                    self.fc.goto_global(lat, lon, alt)
                    pos = self.fc.get_position(timeout=1.5)
                except Exception:
                    time.sleep(1.0)
                    continue
                self._blob_observe(cam, pos, alt)
                self._cover_tick(cam, pos, alt)
                if geo.haversine_m(pos["lat"], pos["lon"], lat, lon) <= self.arrive_m:
                    break
                time.sleep(1.0)
        return True

    def _blob_observe(self, cam, pos, alt_cmd):
        """Passive hypothesis logging (runs during grid legs, throttled).

        Scores the bottom frame for QR-ish rectangles and ground-projects
        each into _blob_obs. Front-cam frames can't be ranged, so they are
        skipped — the bottom cam is the localizing eye.
        """
        if not self.blob_enabled or cam is None or cam.facing != "bottom":
            return
        now = time.time()
        if now - self._blob_last_obs < self.blob_observe_s:
            return
        self._blob_last_obs = now
        try:
            from blob_fallback import find_candidates
            frame, _ts, _cnt = cam.latest()
            if frame is None:
                return
            alt = max(float(pos.get("alt_rel") or alt_cmd or self.sweep_alt), 1.0)
            ctx = {"alt_m": alt, "hfov_deg": cam.hfov_deg,
                   "img_w": cam.size[0], "img_h": cam.size[1]}
            for (x, y, w, h, score, _m) in find_candidates(frame, self.blob_cfg, ctx):
                try:
                    ex, ny = geo.nadir_pixel_to_ground_m(
                        x + w / 2.0, y + h / 2.0, cam.size[0], cam.size[1],
                        cam.hfov_deg, alt, pos.get("hdg") or 0.0, cam.rotation_deg)
                    ox, oy = self.home[0], self.home[1]
                    ex0, ny0 = geo.latlon_to_enu(pos["lat"], pos["lon"], ox, oy)
                    self._blob_obs.append({"e": ex0 + ex, "n": ny0 + ny,
                                           "score": float(score), "ts": now})
                except Exception:
                    continue
            if len(self._blob_obs) > 3000:
                del self._blob_obs[:1500]
        except Exception as e:
            log.debug("blob observe failed: %r", e)

    def _blob_stage_score(self, cam, cl):
        """Score the hypothesis at the current viewpoint.

        Returns (score, offset_m): best nearby candidate's score and its
        ground distance from the drone (the re-centering check). (0.0, None)
        when nothing re-found near expectation.
        """
        from blob_fallback import find_candidates
        try:
            frame, _ts, _cnt = cam.latest()
            pos = self.fc.get_position(timeout=1.5)
        except Exception:
            return 0.0, None
        if frame is None:
            return 0.0, None
        try:
            alt = max(float(pos.get("alt_rel") or 1.0), 1.0)
            ctx = {"alt_m": alt, "hfov_deg": cam.hfov_deg,
                   "img_w": cam.size[0], "img_h": cam.size[1]}
            cands = find_candidates(frame, self.blob_cfg, ctx)
        except Exception:
            return 0.0, None
        if not cands:
            return 0.0, None
        olat, olon = self.home[0], self.home[1]
        ex0, ny0 = geo.latlon_to_enu(pos["lat"], pos["lon"], olat, olon)
        rad = float(self.blob_cfg.get("cluster_radius_m", 2.0)) * 1.5 + 1.0
        best, best_d = None, 1e9
        for (x, y, w, h, score, _m) in cands:
            try:
                ex, ny = geo.nadir_pixel_to_ground_m(
                    x + w / 2.0, y + h / 2.0, cam.size[0], cam.size[1],
                    cam.hfov_deg, alt, pos.get("hdg") or 0.0, cam.rotation_deg)
            except Exception:
                continue
            d = ((ex0 + ex - cl["e"]) ** 2 + (ny0 + ny - cl["n"]) ** 2) ** 0.5
            if d < best_d:
                best, best_d = (score, (ex ** 2 + ny ** 2) ** 0.5), d
        if best is None or best_d > rad:
            return 0.0, None
        return best

    def _cover_tick(self, cam, pos, alt_cmd):
        """Mark searched cells + push throttled full-set snapshots."""
        if not pos or self.home is None:
            return
        try:
            from geo import cover_cells, footprint_m
            fw, fh = footprint_m(float(alt_cmd), cam.hfov_deg,
                                 cam.size[0], cam.size[1])
            new = cover_cells(pos["lat"], pos["lon"],
                              (self.home[0], self.home[1]),
                              self.cov_cell_m, min(fw, fh) / 2.0)
            fresh = new - self._cov
            self._cov |= new
            self._cov_hot |= fresh
            now = time.time()
            if fresh and now - self._cov_last_push >= self.cov_every_s:
                self._cov_last_push = now
                hot = sorted(self._cov_hot)
                self._cov_hot = set()
                self.hub.push("coverage", {
                    "origin": {"lat": self.home[0], "lon": self.home[1]},
                    "cell_m": self.cov_cell_m,
                    "cells": sorted(self._cov),
                    "hot": hot, "total": len(self._cov)})
        except Exception as e:
            log.debug("coverage tick: %r", e)

    def _blob_fallback(self):
        """Last resort: visit top-3 blob hypotheses with descent verification.

        STEER/DESCEND/CENTER only — this path can NEVER transmit. A real
        detector cue hands to TRACK; a decoder payload (+consensus) hands
        to TRANSMIT. Budgets: per-candidate blob_per_s, global blob_cap_s
        on its own clock (independent of the spent search budget).
        """
        if self.payload:
            return self._transmit_and_finish()
        cam = self.rig.get("cam2")
        if cam is None or cam.facing != "bottom":
            self._log("WARN", "blob fallback: no bottom cam — skipping")
            return True
        from blob_fallback import cluster_observations
        clusters = cluster_observations(
            self._blob_obs, self.blob_cfg)[:max(1, self.blob_top_k)]
        if not clusters:
            self._log("WARN", "blob fallback: no ground-stable hypotheses (%d raw sightings)"
                      % len(self._blob_obs))
            return True
        self._set_phase("FALLBACK", "%d hypotheses, %.0fs cap" % (len(clusters), self.blob_cap_s))
        t_cap = time.time() + self.blob_cap_s
        for ci, cl in enumerate(clusters):
            if self._abort.is_set() or not self._battery_ok():
                return self._aborted()
            if self.payload:
                return self._transmit_and_finish()
            if time.time() >= t_cap:
                break
            if not self._blob_visit(cl, ci, len(clusters), t_cap):
                return False
        return True

    def _blob_visit(self, cl, idx, total, t_cap):
        """One hypothesis: center above it, descend stages, verify the
        QR-structure trajectory. Returns False only when the mission must
        stop (transmit/abort happened inside); True = try the next one."""
        from blob_fallback import trend_ok
        cam = self.rig.get("cam2")
        olat, olon = self.home[0], self.home[1]
        tlat, tlon = geo.enu_to_latlon(cl["e"], cl["n"], olat, olon)
        try:
            pos0 = self.fc.get_position(timeout=2.0)
        except Exception:
            return True
        tlat, tlon, inside = self._clamp_to_fence(tlat, tlon, pos0["lat"], pos0["lon"])
        if not inside:
            self._log("WARN", "blob #%d: outside fence — skipping" % (idx + 1))
            return True
        t_end = min(t_cap, time.time() + self.blob_per_s)
        entry_alt = max(float(pos0.get("alt_rel") or self.sweep_alt), 3.0)
        stages = [entry_alt] + [a for a in self.approach_stair if a < entry_alt - 1.0]
        commit = float(self.blob_cfg.get("commit_score", 0.7))
        tol = float(self.blob_cfg.get("center_tol_m", 3.0))
        settle = float(self.blob_cfg.get("settle_s", 2.0))
        max_miss = int(self.blob_cfg.get("max_misses", 2))
        self._log("WARN", "blob #%d/%d: visiting (score %.2f, x%d) stages %s" % (
            idx + 1, total, cl["score"], cl["support"],
            ",".join("%.0f" % a for a in stages)))
        traj, misses = [], 0
        for salt in stages:
            arrived = False
            while time.time() < t_end:
                if self._abort.is_set() or not self._battery_ok():
                    return self._aborted()
                if not self._guided_ok():
                    return self._mode_tripped()
                if self.payload:
                    return self._transmit_and_finish()
                if self._fresh_cue():
                    self._log("WARN", "blob #%d: real cue — handing to TRACK" % (idx + 1))
                    return self._track_and_approach()
                try:
                    self.fc.goto_global(tlat, tlon, salt)
                    pos = self.fc.get_position(timeout=1.5)
                except Exception:
                    time.sleep(1.0)
                    continue
                d = geo.haversine_m(pos["lat"], pos["lon"], tlat, tlon)
                a = abs(float(pos.get("alt_rel") or salt) - salt)
                if d <= max(self.arrive_m, 1.0) and a <= 1.5:
                    arrived = True
                    break
                time.sleep(1.0)
            if not arrived or time.time() >= t_end:
                self._log("WARN", "blob #%d: time — next" % (idx + 1))
                return True
            time.sleep(settle)
            if self.payload:
                return self._transmit_and_finish()
            if self._fresh_cue():
                self._log("WARN", "blob #%d: real cue — handing to TRACK" % (idx + 1))
                return self._track_and_approach()
            sc, off = self._blob_stage_score(cam, cl)
            if off is None:
                misses += 1
                self._log("INFO", "blob #%d @ %.0fm: not re-found (miss %d)" % (idx + 1, salt, misses))
                if misses >= max_miss:
                    self._log("WARN", "blob #%d: keeps disappearing — abort" % (idx + 1))
                    return True
                continue
            misses = 0
            traj.append(sc)
            self._log("INFO", "blob #%d @ %.0fm: score %.2f (off %.1fm)" % (idx + 1, salt, sc, off))
            if off > tol * 2.0:
                self._log("WARN", "blob #%d: won't stay centered (%.1fm) — abort" % (idx + 1, off))
                return True
            if len(traj) >= 2 and not trend_ok(traj, self.blob_cfg):
                self._log("WARN", "blob #%d: flat trajectory %s — abort" % (
                    idx + 1, ",".join("%.2f" % s for s in traj)))
                return True
            if sc >= commit:
                self._log("WARN", "blob #%d: score %.2f — committed, descending" % (idx + 1, sc))
        self._log("WARN", "blob #%d: stages exhausted, no decode — next" % (idx + 1))
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
                self._push_target(tlat, tlon, cam=cam_name,
                                  box=[x, y, w, h], inside=inside)
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
        # real_pi: by the time the approach finishes, the QR:<payload>
        # relay has usually ALREADY been running since confirm (stage 2) —
        # don't double-fire it. If relay.on_confirm is disabled in config,
        # this is the (old) single relay point.
        if self._confirmed_relayed:
            self._set_phase("TRANSMIT",
                            "relay already running since confirm: %r"
                            % (self.payload,))
        else:
            self._set_phase("TRANSMIT", "relaying %r" % (self.payload,))
            from qr_relay import relay_qr
            rcfg = self.cfg.get("relay", {})
            try:
                _p = self.fc.get_position(timeout=2.0)
                _gps = (_p["lat"], _p["lon"], _p.get("alt_rel"))
            except Exception:
                _gps = None
            relay_qr(self.payload, self.fc, self.hub,
                     interval_s=float(rcfg.get("interval_s", 2.0)),
                     window_s=float(rcfg.get("window_s", 15.0)),
                     store=self.store_fwd.store if self.store_fwd.enabled
                     else None,
                     gps=_gps)
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

    def _snapshot_speed(self):
        """Snapshot cruise speed across the 4.7 rename (WP_SPD in m/s
        first, WPNAV_SPEED in cm/s fallback). Sets self._speed_par to
        (name, scale, orig_raw) or None."""
        self._speed_par = None
        for _nm, _sc in (("WP_SPD", 1.0), ("WPNAV_SPEED", 100.0)):
            try:
                _raw = self.fc.get_param(_nm)
                self._speed_par = (_nm, _sc, float(_raw))
                self._log("INFO", "speed snapshot: %s = %s" % (_nm, _raw))
                break
            except Exception as e:
                log.debug("speed snapshot %s failed: %r", _nm, e)
        if self._speed_par is None:
            self._log("WARN", "speed read failed (WP_SPD + WPNAV_SPEED) — search flies FC default")

    def _restore_speed(self):
        if self._speed_par is None:
            return
        _nm, _sc, _raw = self._speed_par
        try:
            self.fc.set_param(_nm, float(_raw))
            self._log("INFO", "%s restored to %s" % (_nm, _raw))
        except Exception as e:
            log.warning("speed restore failed: %r", e)
        self._speed_par = None

    def _mode_tripped(self):
        mode = getattr(self.fc, "mode", "?")
        self._log("ERROR", "vehicle left GUIDED (now %s) — external RTL/failsafe? "
                           "aborting search instead of burning timeouts" % mode)
        if self.payload:
            return self._transmit_and_finish()
        if mode in ("RTL", "LAND"):
            self._restore_speed()
            self._set_phase("DONE", "external %s — search aborted" % mode)
            return False
        return self._finish(False)

    def _aborted(self):
        self._restore_speed()
        self._log("ERROR", "abort — commanding RTL")
        try:
            self.fc.set_mode("RTL", timeout=6.0)
        except Exception as e:
            self._log("ERROR", "RTL failed: %s" % e)
        self._set_phase("FAILSAFE", "aborted by request/battery")
        return False

    def _finish(self, found):
        self._restore_speed()
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
