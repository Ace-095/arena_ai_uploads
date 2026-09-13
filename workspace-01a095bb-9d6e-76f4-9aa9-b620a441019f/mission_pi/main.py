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
    print("[fc] probing serial ...")
    try:
        from fc_link import find_fc
        conn, dev = find_fc(device=args.device)
        print("[fc] OK: %s (sysid=%d)" % (dev, conn.target_system))
        conn.close()
    except Exception as e:
        print("[fc] FAIL: %s" % e)
    print("[cam] probing CSI ...")
    try:
        from cameras import list_cameras
        cams = list_cameras()
        print("[cam] %d camera(s): %s" % (len(cams), [(i, m) for i, m, _ in cams]))
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
    ap.add_argument("--device", default=None, help="FC serial (default: auto-detect)")
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
    from server import create_app, serve_forever
    from mission import Mission

    fc = FCLink()
    dev = fc.connect(device=args.device,
                     bauds=(args.baud,) if args.baud else (115200, 57600, 921600))
    print("FC link: %s" % dev)

    rig = CameraRig(cfg)
    rig.detect()
    rig.start_all()
    if not rig.cams:
        print("WARNING: no cameras — mission will failsafe at search")

    det = get_detector(cfg.get("detector", {}))
    print("detector: %s" % det.name)

    mission = Mission(fc, rig, det, hub=None, cfg=cfg)  # hub attached below
    app = create_app(rig, mission, fc)
    mission.hub = app.state.hub
    port = int(cfg.get("server", {}).get("port", 8000))
    srv = threading.Thread(target=serve_forever, args=(app, "0.0.0.0", port),
                           name="http", daemon=True)
    srv.start()
    print("UI: http://<this-pi>:%d  (paste as Pi link)" % port)
    try:
        mission.run()
    except KeyboardInterrupt:
        print("stopped by user")
    finally:
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
