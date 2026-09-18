#!/usr/bin/env python3
"""Regression test: the QR preset REST endpoints (UI + curl surface).

Covers the exact contract the mission-ui QR PRESET card and
`curl POST /api/qr/preset` rely on:
  * GET  /api/qr/presets -> active preset, ordered names, presets table,
    live filter dict (must reflect config.yaml qr_presets/qr_filter);
  * POST /api/qr/preset {"preset": "dark_qr_boost"} -> alias resolves to
    "dark", mission state actually mutates (cue strictness on, conf set),
    and the filter dict in the response reflects the new state;
  * POST unknown preset -> 404 with the available names;
  * GET /api/fsm/status exposes qr_preset + qr_filter.

Self-skips (exit 0) when fastapi/uvicorn are not installed — the full
suite must stay runnable on a bare sandbox python.
"""
import json
import os
import socket
import sys
import threading
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.path.join(HERE, "..")
sys.path.insert(0, os.path.abspath(PI))

try:
    import fastapi  # noqa: F401
    import uvicorn
except Exception:
    print("SKIP test_qr_preset_api (fastapi/uvicorn not installed)")
    sys.exit(0)

FAILS = []


def check(name, cond, extra=""):
    print(("PASS " if cond else "FAIL ") + name, extra if not cond else "")
    if not cond:
        FAILS.append(name)


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def get(url, timeout=5.0):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.status, json.loads(r.read().decode())


def post(url, body, timeout=5.0):
    req = urllib.request.Request(
        url, data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read().decode())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode() or "{}")


class _StubFC:
    def link_ok(self):
        return False

    def get_position(self, timeout=2.0):
        return {"lat": 0.0, "lon": 0.0, "alt_rel": 15.0}


class _StubCam:
    facing = "bottom"
    name = "cam2"

    def apply_qr_profile(self, p):
        pass


class _StubRig:
    def __init__(self):
        self.cams = {"cam2": _StubCam()}

    def get(self, name):
        return self.cams.get(name)

    def status(self):
        return {}


class _StubHub:
    def push(self, chan, msg):
        pass


def main():
    import yaml
    from detector import get_detector
    from mission import Mission
    from server import create_app

    with open(os.path.join(PI, "config.yaml")) as f:
        cfg = yaml.safe_load(f)

    det = get_detector({"kind": "classical"})
    mission = Mission(_StubFC(), _StubRig(), det, _StubHub(), cfg)
    app = create_app(rig=_StubRig(), mission=mission, fc=_StubFC())

    port = free_port()
    cfg_srv = uvicorn.Config(app, host="127.0.0.1", port=port, log_level="error")
    srv = uvicorn.Server(cfg_srv)
    srv.install_signal_handlers = lambda: None
    th = threading.Thread(target=srv.run, daemon=True)
    th.start()
    t0 = time.time()
    while time.time() - t0 < 10:
        try:
            urllib.request.urlopen("http://127.0.0.1:%d/health" % port, timeout=1)
            break
        except Exception:
            time.sleep(0.1)
    else:
        print("FAIL server did not come up")
        sys.exit(1)
    base = "http://127.0.0.1:%d" % port
    try:
        # ---- GET presets -------------------------------------------------
        st, d = get(base + "/api/qr/presets")
        check("presets 200", st == 200, str(d))
        check("active=day at boot", d.get("active") == "day", str(d.get("active")))
        check("7 presets, key order",
              d.get("names") == ["day", "far", "dark", "kabaddi", "low",
                                 "aggressive", "bench"], str(d.get("names")))
        f = d.get("filter") or {}
        check("filter defaults", f.get("min_conf") == 0.35
              and f.get("aspect_min") == 0.4 and f.get("aspect_max") == 2.5,
              str(f))
        check("dark preset strict",
              (d.get("presets", {}).get("dark") or {}).get(
                  "require_decode_for_cue") is True)

        # ---- POST alias switch ------------------------------------------
        st, d = post(base + "/api/qr/preset", {"preset": "dark_qr_boost"})
        check("switch 200", st == 200, str(d))
        check("alias resolved to dark", d.get("preset") == "dark", str(d.get("preset")))
        check("applied dict", (d.get("applied") or {}).get("conf_thr") == 0.30)
        check("mission state mutated",
              mission.preset_name == "dark"
              and mission.qf.require_decode_for_cue is True
              and mission.full_decode_every == 2,
              "preset=%s strict=%s every=%s" % (
                  mission.preset_name, mission.qf.require_decode_for_cue,
                  mission.full_decode_every))
        check("response filter reflects switch",
              (d.get("filter") or {}).get("require_decode_for_cue") is True)

        # ---- status exposure ---------------------------------------------
        st, d = get(base + "/api/fsm/status")
        check("fsm status 200", st == 200)
        check("fsm exposes qr_preset", d.get("qr_preset") == "dark",
              str(d.get("qr_preset")))
        check("fsm exposes qr_filter strict",
              (d.get("qr_filter") or {}).get("require_decode_for_cue") is True)

        # ---- unknown preset ------------------------------------------------
        st, d = post(base + "/api/qr/preset", {"preset": "bogus"})
        check("unknown preset 404", st == 404, "st=%s %s" % (st, d))
        check("404 lists available", "kabaddi" in str(d.get("detail", "")),
              str(d.get("detail")))

        # ---- switch back ----------------------------------------------------
        st, d = post(base + "/api/qr/preset", {"preset": "day"})
        check("switch back to day", st == 200 and d.get("preset") == "day"
              and mission.qf.require_decode_for_cue is False)
    finally:
        srv.should_exit = True
        th.join(timeout=5)

    if FAILS:
        print("%d FAILURE(S): %s" % (len(FAILS), FAILS))
        sys.exit(1)
    print("test_qr_preset_api OK")


if __name__ == "__main__":
    main()
