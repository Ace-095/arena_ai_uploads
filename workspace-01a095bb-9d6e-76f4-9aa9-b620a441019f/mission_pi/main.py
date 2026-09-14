#!/usr/bin/env python3
"""mission_pi entry point.

Bring-up order on the Pi:
  1. python main.py --check        probe FC + cameras + Hailo, print report
  2. python main.py --config config.yaml
     (fc auto-detect -> cameras -> detector -> http/ws server -> mission)

The laptop UI then opens http://<pi-ip>:8000  (paste as the Pi link).
"""
import argparse
import logging
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def load_config(path):
    cfg = {}
    if path and os.path.isfile(path):
        try:
            import yaml
            with open(path) as f:
                cfg = yaml.safe_load(f) or {}
        except Exception as e:
            print("config %s unreadable (%r) — defaults" % (path, e))
    return cfg


def run_check(args):
    print("== mission_pi bring-up check ==")
    cfg = load_config(args.config)
    dev_arg = args.device or cfg.get("fc", {}).get("conn")
    print("[fc] probing %s ..." % (dev_arg or "serial auto-detect"))
    try:
        from fc_link import find_fc
        conn, dev = find_fc(device=dev_arg)
        print("[fc] OK: %s (sysid=%d)" % (dev, conn.target_system))
        conn.close()
    except Exception as e:
        print("[fc] FAIL: %s" % e)
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
    print("[hailo] probing runtime ...")
    try:
        import hailo_platform  # noqa
        print("[hailo] hailo_platform import OK")
    except Exception as e:
        print("[hailo] unavailable (%r) — classical fallback will be used" % (e,))
    hef = args.hef or ""
    print("[hef] %s %s" % (hef or "(none given)",
                           "EXISTS" if hef and os.path.isfile(hef) else ""))
    print("== check done ==")


def main():
    ap = argparse.ArgumentParser(description="mission_pi — QR hunt companion")
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--device", default=None,
                        help="FC link (default: fc.conn in config, else serial "
                             "auto-detect; e.g. tcp:127.0.0.1:5763 for SITL)")
    ap.add_argument("--baud", type=int, default=None)
    ap.add_argument("--check", action="store_true", help="probe hardware and exit")
    ap.add_argument("--hef", default=None, help="HEF path override for --check")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(name)s %(levelname)s: %(message)s")
    if args.check:
        run_check(args)
        return 0

    cfg = load_config(args.config)
    if args.port:
        cfg.setdefault("server", {})["port"] = args.port

    from fc_link import FCLink
    from cameras import CameraRig
    from detector import get_detector
    from server import create_app, create_server
    from mission import Mission

    fc = FCLink()
    dev = fc.connect(device=args.device or cfg.get("fc", {}).get("conn"),
                     bauds=(args.baud,) if args.baud else (115200, 57600, 921600))
    print("FC link: %s" % dev)

    rig = CameraRig(cfg)
    rig.detect()
    rig.start_all()
    if not rig.cams:
        print("WARNING: no cameras — mission will failsafe at search")

    from streamm1 import StreamManager
    streams = StreamManager(rig, cfg.get("stream", {}))
    streams.start_all()

    det = get_detector(cfg.get("detector", {}))
    print("detector: %s" % det.name)

    mission = Mission(fc, rig, det, hub=None, cfg=cfg)  # hub attached below
    app = create_app(rig, mission, fc, streams)
    mission.hub = app.state.hub
    port = int(cfg.get("server", {}).get("port", 8000))
    server = create_server(app, "0.0.0.0", port)
    srv = threading.Thread(target=server.run, name="http", daemon=True)
    srv.start()
    print("UI: http://<this-pi>:%d  (paste as Pi link)" % port)
    print("UI streams: /api/camera/stream/cam1|2 (%s)" % streams.describe())
    try:
        mission.run()
        # The Pi link must outlive the mission: the UI needs cameras +
        # status after FAILSAFE/DONE too (post-flight inspection, re-runs).
        print("mission ended (%s) — server keeps running for the UI (Ctrl-C to stop)"
              % getattr(mission, "phase", "?"))
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("stopped by user")
    finally:
        try:
            server.should_exit = True
            srv.join(timeout=6.0)
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
            det.close()
        except Exception:
            pass
        try:
            fc.close()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
