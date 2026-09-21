#!/usr/bin/env python3
"""mission_pi entry point.

Bring-up order (fixed 2026-09-14 — the Pi link is the FIRST thing up):

  1. build the objects (cheap, cannot block)
  2. START THE HTTP/WS SERVER and print the real paste-ready URLs
  3. in a background thread: detector -> cameras -> streams -> FC link
     (each fail-soft: a failure is recorded in bringup.Bringup, reported at
     GET /health + the WS `log` channel, and never kills the server)
  4. run the mission (it waits for the FC link the background thread is
     still negotiating), then keep serving forever so the UI can inspect
     cameras/state after DONE or FAILSAFE.

Why: the old order connected the FC first and started the server last, so
"SITL not up yet" (or a missing cv2, or a camera that would not open)
exited with a traceback and the laptop UI saw `connection refused` on
http://<ip>:8000 with nothing to explain why. Now the UI always connects
and tells you exactly which subsystem is missing.

Usage:
  python main.py --check                 probe FC + cameras + Hailo, exit
  python main.py --config config.yaml    flight / bench run
  python main.py --no-fc --no-cams       UI-link smoke test (server only)
"""
import argparse
import logging
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

log = logging.getLogger("main")


def load_config(path):
    cfg = {}
    if path and os.path.isfile(path):
        try:
            import yaml
            with open(path) as f:
                cfg = yaml.safe_load(f) or {}
        except Exception as e:
            print("config %s unreadable (%r) — defaults" % (path, e))
    elif path:
        print("config %s not found — running on defaults "
              "(cp config.laptop.yaml config.yaml, or pass --config)" % path)
    return cfg


# ---------------------------------------------------------------- hints ----
def fc_hint(device, err):
    """Turn a raw connect error into the sentence that actually fixes it."""
    text = str(err)
    dev = str(device or "")
    if "5762" in dev and ("refused" in text.lower() or "errno 111" in text.lower()):
        return ("SITL's serial1 (tcp 5762) only starts listening AFTER a GCS "
                "connects to serial0 (tcp 5760) — connect Mission Planner "
                "first, then mission_pi retries and joins by itself.")
    if "refused" in text.lower():
        return ("nothing is listening on %s — is SITL/Gazebo up? "
                "(sim_vehicle.py ... --no-mavproxy)" % dev)
    if "no serial candidates" in text.lower():
        return ("no /dev/ttyACM* or /dev/ttyUSB* — USB cable, or "
                "`sudo usermod -aG dialout $USER` then re-login.")
    if "timed out" in text.lower() or "no FC heartbeat" in text:
        return ("port opened but no vehicle heartbeat — check SERIALn_PROTOCOL "
                "= MAVLink2 on that port, and that MP is not the only client.")
    if "pymavlink not installed" in text:
        return "pip install -r requirements.txt (inside the venv)."
    return ""


def ws_impl(server):
    """Which websocket implementation uvicorn will use — or None.

    Without one, uvicorn answers /ws/telemetry with a bare 404: HTTP works,
    the camera tiles work, and the UI's PI lamp simply stays red with
    "pi ws closed (retry N)". That single missing extra is the difference
    between "the Pi link is broken" and "the Pi link works", so it gets
    checked and shouted about at startup instead of being guessed at.
    """
    impl = getattr(getattr(server, "config", None), "ws_protocol_class", None)
    if impl is not None:
        return getattr(impl, "__name__", str(impl))
    for mod in ("websockets", "wsproto"):
        try:
            __import__(mod)
            return mod
        except Exception:
            continue
    return None


def print_urls(port, host):
    """Print the exact strings to paste into the laptop UI."""
    try:
        import bringup as bu
        urls = bu.lan_urls(port)
    except Exception:
        urls = []
    print("-" * 68)
    print("Pi link (paste ONE of these into the UI's Pi-link box on Windows):")
    for u in urls:
        print("    %s" % u)
    print("    http://127.0.0.1:%d        (this machine only)" % port)
    if not urls:
        print("  !! no LAN IPv4 found — the laptop cannot reach this box yet")
        print("     (check WiFi/cable: `hostname -I`)")
    if str(host) not in ("0.0.0.0", "::", ""):
        print("  !! server.host=%s is NOT 0.0.0.0 — other machines are locked "
              "out" % host)
    print("  from Windows:  curl http://<that-ip>:%d/health" % port)
    print("  if that hangs:  sudo ufw allow %d/tcp   (and 5760/5762 for SITL)"
          % port)
    print("-" * 68)
    sys.stdout.flush()
    return urls


# ---------------------------------------------------------------- check ----
def run_check(args):
    print("== mission_pi bring-up check ==")
    cfg = load_config(args.config)
    _fcc = cfg.get("fc", {}) or {}
    # `conn:` is the documented key; accept `device:` too (easy to write).
    # "auto"/empty = USB auto-detect.
    dev_arg = args.device or _fcc.get("conn") or _fcc.get("device")
    if str(dev_arg or "").lower() in ("", "auto", "none"):
        dev_arg = None
    print("[fc] probing %s ..." % (dev_arg or "serial auto-detect"))
    try:
        from fc_link import find_fc
        conn, dev = find_fc(device=dev_arg)
        print("[fc] OK: %s (sysid=%d)" % (dev, conn.target_system))
        conn.close()
    except Exception as e:
        print("[fc] FAIL: %s" % e)
        h = fc_hint(dev_arg, e)
        if h:
            print("[fc] hint: %s" % h)
    print("[cam] probing per %s ..." % args.config)
    try:
        from cameras import CameraRig, list_video_devices
        rig = CameraRig(cfg)
        rig.detect()
        for name, cam in rig.cams.items():
            print("[cam] %s: kind=%s model=%s facing=%s size=%s" % (
                name, cam.kind, cam.model, cam.facing, cam.size))
        if not rig.cams:
            print("[cam] none assigned (v4l2 nodes: %s)" % list_video_devices())
    except Exception as e:
        print("[cam] FAIL: %s" % e)
    print("[vision] cv2/pyzbar import check ...")
    for mod in ("cv2", "pyzbar", "numpy", "yaml", "fastapi", "uvicorn",
                "pymavlink"):
        try:
            __import__(mod)
            print("[vision] %-10s OK" % mod)
        except Exception as e:
            print("[vision] %-10s MISSING (%s)" % (mod, e.__class__.__name__))
    print("[hailo] probing runtime ...")
    try:
        import hailo_platform  # noqa
        print("[hailo] hailo_platform import OK")
    except Exception as e:
        print("[hailo] unavailable (%r) — classical fallback will be used" % (e,))
    hef = args.hef or ""
    print("[hef] %s %s" % (hef or "(none given)",
                           "EXISTS" if hef and os.path.isfile(hef) else ""))
    print("[net] websocket implementation ...")
    try:
        from server import create_server
        probe = create_server(None, "127.0.0.1", 1)  # config only, never run
        impl = ws_impl(probe)
    except Exception:
        impl = ws_impl(None)
    if impl:
        print("[net] ws impl: %s" % impl)
    else:
        print("[net] ws impl: NONE — the UI Pi link would 404!")
        print("[net]     fix: pip install \"uvicorn[standard]\"   (or websockets)")
    port = int(cfg.get("server", {}).get("port", args.port or 8000))
    print("[net] UI would serve on port %d:" % port)
    try:
        import bringup as bu
        if not bu.port_free(port):
            print("[net] !! port %d is ALREADY in use — a previous mission_pi "
                  "is still running (the UI would reach THAT one)" % port)
        for u in bu.lan_urls(port):
            print("[net]     %s" % u)
    except Exception as e:
        print("[net] probe failed: %r" % (e,))
    print("== check done ==")


# ------------------------------------------------------------ bring-up -----
def bring_up_thread(cfg, args, fc, rig, det_box, streams, mission, hub, bu):
    """Everything that can fail, in fail-soft order, off the main thread."""

    def say(sub, state, detail):
        bu.mark(sub, state, detail)
        hub.push("log", {"level": "INFO" if state == "ok" else "WARN",
                         "msg": "bring-up %s: %s (%s)" % (sub, state, detail)})
        log.info("bring-up %s: %s (%s)", sub, state, detail)

    # 1) detector -------------------------------------------------------
    try:
        from detector import get_detector
        det = get_detector(cfg.get("detector", {}))
        det_box[0] = det
        mission.detector = det
        say("detector", "ok", det.name)
    except Exception as e:
        bu.fail("detector", e)
        hub.push("log", {"level": "ERROR",
                         "msg": "detector unavailable (%s) — decode-only "
                                "pipeline, cues will be rare" % e})

    # 2) cameras --------------------------------------------------------
    if args.no_cams:
        say("cameras", "skipped", "--no-cams")
    else:
        try:
            with rig.rescan_lock():
                detected = rig.detect()
                for name, cam in detected.items():
                    old = rig.cams.get(name)
                    # the auto-rescan timer may have adopted this camera
                    # already (it starts faster) — never double-open a
                    # device; keep the running object instead.
                    if (old is not None and old.running
                            and (old.kind, old.index, old.model)
                            == (cam.kind, cam.index, cam.model)):
                        continue
                    try:
                        cam.start()
                        rig.cams[name] = cam
                    except Exception as e:
                        log.error("%s start failed: %r", name, e)
            if rig.cams:
                say("cameras", "ok", ", ".join(
                    "%s=%s/%s" % (n, c.model, c.facing)
                    for n, c in sorted(rig.cams.items())))

                # --- QR boost: make QR pop more than ground (Kabaddi setting) ---
                try:
                    cam_cfg = cfg.get("cameras", {}) or {}
                    qr_boost_cfg = cfg.get("qr_boost", {}) or {}
                    global_enabled = qr_boost_cfg.get("enabled", True)
                    for cam_name, cam in rig.cams.items():
                        role = cam.facing
                        role_cfg = cam_cfg.get(role, {}) or {}
                        per_cam_qr = role_cfg.get("qr_boost", {}) or {}
                        enabled = per_cam_qr.get("enabled", global_enabled)
                        if not enabled:
                            continue
                        profile = per_cam_qr.get("profile") or qr_boost_cfg.get("profile")
                        if not profile:
                            if role == "bottom":
                                profile = "bottom_qr_boost"
                            elif role == "front":
                                profile = "front_qr_boost"
                            else:
                                profile = qr_boost_cfg.get("mode", "qr_boost_day")
                        try:
                            cam.apply_qr_profile(profile)
                            say("cameras", "ok", "%s QR boost: %s" % (cam_name, profile))
                            hub.push("log", {"level": "INFO",
                                             "msg": "%s QR boost %s — contrast high, sat low, sharp high, QR pops vs ground" % (cam_name, profile)})
                        except Exception as e:
                            log.warning("%s QR boost %r failed: %r", cam_name, profile, e)
                        sw_mode = per_cam_qr.get("software_mode") or qr_boost_cfg.get("software_mode", "qr_boost")
                        if sw_mode:
                            cam.qr_boost["qr_software_enhance"] = True
                            cam.qr_boost["qr_enhance_mode"] = sw_mode
                            log.info("%s software QR enhance %s enabled (CLAHE+unsharp+green suppress)", cam_name, sw_mode)
                except Exception as e:
                    log.warning("QR boost setup failed: %r", e)
                    hub.push("log", {"level": "WARN", "msg": "QR boost setup failed: %s" % e})

            else:
                bu.mark("cameras", "failed", "no camera assigned")
                hub.push("log", {"level": "ERROR",
                                 "msg": "no cameras — the mission will "
                                        "failsafe at search; check config "
                                        "`cameras:` + /dev/video*"})
        except Exception as e:
            bu.fail("cameras", e)

    # 3) UI streams (needs the cameras to exist first) -------------------
    try:
        streams.sync(rig)
        streams.start_all()
        say("streams", "ok" if streams.streams else "skipped",
            streams.describe())
    except Exception as e:
        bu.fail("streams", e)

    # 4) FC link (retrying — the UI stays up the whole time) -------------
    if args.no_fc:
        say("fc", "skipped", "--no-fc (UI/camera-only mode)")
        bu.finish()
        return
    fc_cfg = cfg.get("fc", {}) or {}
    device = (args.device or fc_cfg.get("conn") or fc_cfg.get("device"))
    # real_pi: conn: "auto" (or empty) = USB auto-detect — never pass the
    # literal word "auto" to the serial opener.
    if str(device or "").lower() in ("", "auto", "none"):
        device = None
    # real_pi: honour fc.bauds from config (Pixhawk default 115200 first)
    try:
        cfg_bauds = [int(b) for b in (fc_cfg.get("bauds") or [])]
    except Exception:
        cfg_bauds = []
    # Self-checked fix: try both 5760 and 5762 — user reports Pi needs MP first on 5762
    # Old config only tried 5762, failed when MP not yet connected or SITL only on 5760
    # New: build list of candidates: configured device + fallbacks 5760,5762,14550
    candidates = []
    # Physical-flight default: try USB serial auto-detection FIRST. Empty
    # fc.conn must not jump straight to the SITL fallback addresses.
    if device is None:
        candidates.append(None)
    else:
        candidates.append(device)
    # Add fallbacks if not already
    for fb in ("tcp:127.0.0.1:5762", "tcp:127.0.0.1:5760", "tcp:127.0.0.1:5760", "udp:127.0.0.1:14550"):
        if fb not in candidates:
            candidates.append(fb)
    # Also try 0.0.0.0 variants for 2-laptop
    for fb in ("tcp:0.0.0.0:5762", "tcp:0.0.0.0:5760"):
        if fb not in candidates:
            candidates.append(fb)
    bauds = (args.baud,) if args.baud else (
        tuple(cfg_bauds) if cfg_bauds else (115200, 57600, 921600))
    retry_s = float(fc_cfg.get("retry_s", args.fc_retry or 5.0))
    timeout_s = float(fc_cfg.get("connect_timeout_s", args.fc_timeout or 0))

    def fc_event(kind, detail):
        """Watchdog events go to the UI log — a silent link loss is how a
        mission ends up 'just not working' with nobody able to say why."""
        lvl = {"link_lost": "ERROR", "link_retry": "WARN",
               "link_restored": "WARN"}.get(kind, "INFO")
        hub.push("log", {"level": lvl, "msg": "FC %s: %s" % (kind, detail)})
        hub.push("event", {"type": "fc_link", "state": kind, "detail": detail})
        bu.note("fc", "%s — %s" % (kind, detail))
    attempt, t0 = 0, time.time()
    # Self-checked fix: try all candidates round-robin, not just configured device
    # Old loop only tried `device`, so if MP not yet on 5760, 5762 had no heartbeat and failed forever
    # New: cycle through candidates list (5762,5760,14550,0.0.0.0 variants)
    cand_idx = 0
    while not bu.stop.is_set():
        attempt += 1
        # Pick candidate in round-robin
        cur_device = candidates[cand_idx % len(candidates)] if candidates else device
        cand_idx += 1
        try:
            dev = fc.connect(device=cur_device, bauds=bauds, supervise=True,
                             on_event=fc_event,
                             dead_s=float(fc_cfg.get("dead_s", 10.0)),
                             retry_s=max(2.0, retry_s))
            say("fc", "ok", "%s (attempt %d, tried %s)" % (dev, attempt, cur_device))
            break
        except Exception as e:
            msg = "%s" % e
            hint = fc_hint(cur_device, e)
            # /health must keep telling the truth about INTENT: the
            # configured device (or auto-detect), not whichever fallback
            # candidate was tried last.
            try:
                fc.device = device if device else "auto-detect"
            except Exception:
                pass
            bu.note("fc", "attempt %d failed (%s): %s" % (attempt, cur_device, msg))
            if attempt == 1 or attempt % 6 == 0:
                hub.push("log", {"level": "ERROR",
                                 "msg": "FC link attempt %d failed (%s): %s%s"
                                        % (attempt, cur_device, msg,
                                           (" — " + hint) if hint else "")})
            if attempt == 1:
                bu.fail("fc", RuntimeError(msg + ((" — " + hint) if hint else "")))
                bu.mark("fc", "pending", "retrying every %.0fs over %s" % (retry_s, ", ".join(candidates[:3])))
            log.warning("FC link attempt %d failed (%s): %s%s", attempt, cur_device, msg,
                        (" — " + hint) if hint else "")
            if timeout_s and (time.time() - t0) > timeout_s:
                bu.mark("fc", "failed", "gave up after %.0fs" % timeout_s)
                hub.push("log", {"level": "ERROR",
                                 "msg": "FC link: gave up after %.0fs — the UI "
                                        "stays up, restart or fix the link"
                                        % timeout_s})
                break
            bu.stop.wait(min(30.0, max(1.0, retry_s)))
    bu.finish()


# ------------------------------------------------------------------ main ---
def main():
    ap = argparse.ArgumentParser(description="mission_pi — QR hunt companion")
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--port", type=int, default=None,
                    help="HTTP/WS port (default: server.port in config, 8000)")
    ap.add_argument("--host", default=None,
                    help="bind address (default: server.host in config, "
                         "0.0.0.0 — do NOT use 127.0.0.1 or the laptop UI "
                         "cannot reach the Pi)")
    ap.add_argument("--device", default=None,
                    help="FC link (default: fc.conn in config, else serial "
                         "auto-detect; e.g. tcp:127.0.0.1:5762 for SITL)")
    ap.add_argument("--baud", type=int, default=None)
    ap.add_argument("--check", action="store_true", help="probe hardware and exit")
    ap.add_argument("--hef", default=None, help="HEF path override for --check")
    ap.add_argument("--no-fc", action="store_true",
                    help="skip the FC link (UI + cameras only)")
    ap.add_argument("--no-cams", action="store_true",
                    help="skip camera bring-up (link/mission dry run)")
    ap.add_argument("--fc-retry", type=float, default=None,
                    help="seconds between FC connect retries (default 5)")
    ap.add_argument("--fc-timeout", type=float, default=None,
                    help="give up on the FC link after N s (default 0 = "
                         "keep retrying; the UI stays up either way)")
    ap.add_argument("--doctor", action="store_true",
                    help="run tools/link_doctor.py and exit")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(name)s %(levelname)s: %(message)s")
    # Line-buffer stdout even when redirected (systemd/nohup/tee): the URL
    # banner is the thing the operator needs to see IMMEDIATELY, not at exit.
    try:
        sys.stdout.reconfigure(line_buffering=True)
        sys.stderr.reconfigure(line_buffering=True)
    except Exception:
        pass
    if args.doctor:
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        "tools"))
        import link_doctor
        return link_doctor.main([
            "--config", args.config] + (["--port", str(args.port)]
                                        if args.port else []))
    if args.check:
        run_check(args)
        return 0

    cfg = load_config(args.config)
    # accept either `server:` or `ui:` for host/port — the shipped configs use
    # `server:`, older/other configs use `ui:`. CLI flags win over both.
    srv_cfg = cfg.get("server") or cfg.get("ui") or {}
    cfg["server"] = srv_cfg
    port = int(args.port or srv_cfg.get("port")
               or os.environ.get("MISSION_PI_PORT") or 8000)
    host = str(args.host or srv_cfg.get("host")
               or os.environ.get("MISSION_PI_HOST") or "0.0.0.0")
    srv_cfg["port"], srv_cfg["host"] = port, host

    import bringup as bu_mod
    from server import create_app, create_server

    bu = bu_mod.Bringup()
    bu.stop = threading.Event()          # bring-up thread cancellation flag

    # ---- objects first (nothing here blocks or needs hardware) ----------
    from fc_link import FCLink
    from cameras import CameraRig
    from mission import Mission
    from streamm1 import StreamManager

    fc = FCLink()
    rig = CameraRig(cfg)
    det_box = [None]
    streams = StreamManager(rig, cfg.get("stream", {}))   # adopts cams later
    mission = Mission(fc, rig, det_box[0], hub=None, cfg=cfg)

    # ---- 2) SERVER FIRST: the UI must be able to reach us no matter what --
    if not bu_mod.port_free(port, host if host != "0.0.0.0" else "0.0.0.0"):
        print("!! port %d already in use — a previous mission_pi is probably "
              "still running (kill it, or use --port)" % port)
    app = create_app(rig, mission, fc, streams, bu, port=port)
    mission.hub = app.state.hub
    hub = app.state.hub
    server = create_server(app, host, port)
    srv = threading.Thread(target=server.run, name="http", daemon=True)
    srv.start()
    deadline = time.time() + 8.0
    while time.time() < deadline and not server.started:
        time.sleep(0.05)
    if server.started:
        bu.mark("server", "ok", "%s:%d" % (host, port))
    else:
        bu.mark("server", "failed", "uvicorn did not start on %s:%d" % (host, port))
        print("!! HTTP server failed to start on %s:%d — the UI cannot "
              "connect (port busy? bad host?)" % (host, port))
    impl = ws_impl(server)
    if impl is None:
        msg = ("uvicorn has NO websocket implementation — the UI's Pi link "
               "(ws://.../ws/telemetry) answers 404 and the PI lamp stays red. "
               "Fix: pip install \"uvicorn[standard]\"  (or: pip install "
               "websockets). HTTP + camera tiles keep working meanwhile, and "
               "the UI falls back to polling /api/fsm/status.")
        print("!! " + msg)
        bu.note("server", "%s:%d (NO websocket library!)" % (host, port))
        app.state.hub.push("log", {"level": "ERROR", "msg": msg})
    else:
        log.info("websocket impl: %s", impl)
    urls = print_urls(port, host)
    bu.set_urls(urls)
    print("UI streams: /api/camera/stream/cam1|2 (%s)" % streams.describe())
    hub.push("log", {"level": "INFO",
                     "msg": "mission_pi up on %s:%d — %s" % (host, port,
                                                             ", ".join(urls) or
                                                             "loopback only")})
    if not server.started:
        return 2

    # ---- 3) fail-soft bring-up in the background -----------------------
    up = threading.Thread(target=bring_up_thread, name="bring-up", daemon=True,
                          args=(cfg, args, fc, rig, det_box, streams, mission,
                                hub, bu))
    up.start()

    # ---- 3b) camera auto-rescan timer (real_pi hot-plug recovery) ------
    # Every auto_rescan_s (config cameras.auto_rescan_s, 0 = off) re-run
    # camera auto-detect: a USB cam re-plugged mid-mission, a CSI cam that
    # came back after a libcamera hiccup — adopted and streamed live
    # without restarting mission_pi. The per-camera watchdogs (capture
    # reopen + stream watchdog) cover single-cam failures in between.
    try:
        rescan_s = float((cfg.get("cameras", {}) or {}).get(
            "auto_rescan_s", 30))
    except Exception:
        rescan_s = 30.0

    def _rescan_loop():
        while not bu.stop.wait(max(5.0, rescan_s)):
            try:
                changed = rig.rescan()
                if changed:
                    streams.sync(rig)
                    streams.start_all()
                    hub.push("log", {"level": "WARN",
                                     "msg": "camera auto-rescan: %s"
                                            % ", ".join(changed)})
                    hub.push("event", {"type": "cameras",
                                       "rescan": True, "changed": changed})
                if streams.streams:
                    streams.sync(rig)  # keep streams aligned (idempotent)
            except Exception as e:
                log.warning("camera auto-rescan failed: %r", e)

    if rescan_s > 0 and not args.no_cams:
        threading.Thread(target=_rescan_loop, name="cam-rescan",
                         daemon=True).start()
        log.info("camera auto-rescan: every %.0fs (POST "
                 "/api/cameras/rescan to force)", rescan_s)

    # ---- 4) mission, then serve forever --------------------------------
    try:
        while True:
            mission.detector = det_box[0] or mission.detector
            mission.run()
            phase = getattr(mission, "phase", "?")
            detail = getattr(mission, "detail", "")
            print("mission ended (%s: %s) — server keeps running for the UI "
                  "(Ctrl-C to stop)" % (phase, detail))
            hub.push("log", {"level": "WARN",
                             "msg": "mission ended (%s) — Pi link stays up"
                                    % phase})
            # Bench convenience: if we only died for want of an FC link and
            # the link has since come up, run the mission again instead of
            # making the operator restart the process (the UI stays connected).
            if phase == "FAILSAFE" and "no FC link" in str(detail) and \
                    not args.no_fc:
                waited = 0
                while waited < 600 and not fc.link_ok() and not bu.stop.is_set():
                    time.sleep(1.0)
                    waited += 1
                if fc.link_ok():
                    log.warning("FC link recovered — restarting the mission")
                    hub.push("log", {"level": "WARN",
                                     "msg": "FC link up — mission restarting"})
                    mission.reset()
                    continue
            break
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("stopped by user")
    finally:
        bu.stop.set()
        try:
            server.should_exit = True
            srv.join(timeout=6.0)
        except Exception:
            pass
        try:
            mission.stop()
        except Exception:
            pass
        try:
            streams.stop_all()
        except Exception:
            pass
        try:
            rig.stop_all()
        except Exception:
            pass
        try:
            if det_box[0] is not None:
                det_box[0].close()
        except Exception:
            pass
        try:
            fc.close()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
