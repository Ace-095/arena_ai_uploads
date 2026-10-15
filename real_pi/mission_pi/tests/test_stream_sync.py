#!/usr/bin/env python3
"""Stdlib tests for streamm1 — the MJPEG fan-out the UI's camera panes use.

Locks down the ordering fix from 2026-09-14: main.py builds the StreamManager
BEFORE the cameras exist (the HTTP server has to bind first), so the manager
starts empty and must adopt cameras later via sync(). Without sync() the UI's
/api/camera/stream/<cam> returned 404 for the whole flight even though the
cameras were fine — and nothing in /health explained why.

Also covers eviction (a camera that disappears must stop streaming, not leak a
thread) and the encode-once/serve-many contract.
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import streamm1 as S  # noqa

FAILS = []


def check(name, cond, extra=""):
    print(("PASS " if cond else "FAIL ") + name, extra if not cond else "")
    if not cond:
        FAILS.append(name)


# a real 1x1 JPEG if OpenCV is around, else a stand-in (the stream only needs
# truthy bytes — it never parses them)
try:
    import cv2
    import numpy as np
    JPEG = cv2.imencode(".jpg", np.zeros((8, 8, 3), np.uint8))[1].tobytes()
    HAVE_CV2 = True
except Exception:                                           # noqa: BLE001
    JPEG = b"\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\xff\xd9"
    HAVE_CV2 = False


class FakeCam:
    def __init__(self, name, facing="bottom", fails=False):
        self.name = name
        self.facing = facing
        self.calls = 0
        self.fails = fails
        self.last_kwargs = None

    def jpeg(self, max_width=960, quality=70):
        self.calls += 1
        self.last_kwargs = {"max_width": max_width, "quality": quality}
        if self.fails:
            raise RuntimeError("V4L2: no buffer available")
        return JPEG


class FakeRig:
    def __init__(self, cams=None):
        self.cams = dict(cams or {})


# ---- construction before the cameras exist (the server-first ordering) ----
rig = FakeRig()
sm = S.StreamManager(rig, {"fps": 8, "width": 640, "quality": 60})
check("starts empty when the rig has no cams yet", sm.streams == {},
      sm.streams)
check("describe() survives an empty manager", sm.describe() == "8fps 640w q60 x0",
      sm.describe())
check("status() on an empty manager is a dict", isinstance(sm.status(), dict))
check("get() on a missing cam is None", sm.get("bottom") is None)
sm.start_all()                                   # must not raise
sm.stop_all()

# ---- sync adopts cameras discovered later --------------------------------
rig.cams["bottom"] = FakeCam("bottom")
n = sm.sync(rig)
check("sync adopts a new camera", n == 1 and "bottom" in sm.streams, sm.streams)
check("adopted stream keeps the configured fps/width/quality",
      (sm.streams["bottom"].fps, sm.streams["bottom"].width,
       sm.streams["bottom"].quality) == (8, 640, 60),
      sm.streams["bottom"].status())
check("sync is idempotent", sm.sync(rig) == 1 and len(sm.streams) == 1)
first = sm.streams["bottom"]
sm.sync(rig)
check("sync keeps the existing stream object (no restart churn)",
      sm.streams["bottom"] is first)

rig.cams["front"] = FakeCam("front", facing="front")
check("sync adopts a second camera", sm.sync(rig) == 2
      and set(sm.streams) == {"bottom", "front"}, sorted(sm.streams))

# ---- eviction: a vanished camera stops and is dropped --------------------
front = sm.streams["front"]
front.start()
check("stream actually starts", front.running is True)
del rig.cams["front"]
n = sm.sync(rig)
check("vanished camera is dropped", n == 1 and "front" not in sm.streams,
      sorted(sm.streams))
check("vanished camera's thread was stopped (no leak)",
      front.running is False and front._thread is None)
check("the surviving camera is untouched", sm.streams["bottom"] is first)

# ---- encode once, serve many; frames flow --------------------------------
bottom_cam = rig.cams["bottom"]
st = sm.streams["bottom"]
jpg0, seq0 = st.latest()
check("no frame before start", jpg0 is None and seq0 == 0, (jpg0, seq0))
st.start()
t_end = time.time() + 3.0
while time.time() < t_end and st.latest()[1] < 2:
    time.sleep(0.02)
jpg, seq = st.latest()
check("frames are being encoded", bool(jpg) and seq >= 2, (bool(jpg), seq))
check("encoded bytes are what the camera produced", jpg == JPEG)
check("camera got the configured width/quality",
      bottom_cam.last_kwargs == {"max_width": 640, "quality": 60},
      bottom_cam.last_kwargs)
stat = st.status()
check("status reports a live stream",
      stat["running"] is True and stat["seq"] >= 2 and stat["bytes"] == len(JPEG)
      and stat["age_s"] is not None and stat["age_s"] < 2.0, stat)
a, sa = st.latest()
b, sb = st.latest()
check("latest() is repeatable without re-encoding", sa == sb and a == b)
st.stop()
check("stop halts encoding", st.running is False)
seq_after_stop = st.latest()[1]
time.sleep(0.35)
check("no frames after stop", st.latest()[1] == seq_after_stop)

# ---- a broken camera must not kill the manager --------------------------
rig.cams["bad"] = FakeCam("bad", fails=True)
sm.sync(rig)
sm.start_all()
time.sleep(0.4)
check("a raising camera leaves its stream alive but empty",
      sm.streams["bad"].running is True and sm.streams["bad"].latest()[0] is None)
check("the healthy camera keeps streaming while another fails",
      sm.streams["bottom"].running is True)
check("status() covers every camera", set(sm.status()) >= {"bottom", "bad"},
      sorted(sm.status()))
sm.stop_all()
check("stop_all stops everything",
      all(not x.running for x in sm.streams.values()))

# ---- parameter clamping --------------------------------------------------
sm2 = S.StreamManager(FakeRig({"c": FakeCam("c")}),
                      {"fps": 999, "width": 99999, "quality": 1})
c = sm2.streams["c"]
check("fps clamped to <=30", c.fps == 30, c.fps)
check("width clamped to <=1920", c.width == 1920, c.width)
check("quality clamped to >=30", c.quality == 30, c.quality)
sm3 = S.StreamManager(FakeRig({"c": FakeCam("c")}), None)
check("defaults applied when cfg is None",
      (sm3.fps, sm3.width, sm3.quality) == (S.DEFAULTS["fps"],
                                           S.DEFAULTS["width"],
                                           S.DEFAULTS["quality"]),
      (sm3.fps, sm3.width, sm3.quality))
sm4 = S.StreamManager(FakeRig({"c": FakeCam("c")}), {"fps": "nonsense"})
check("garbage cfg falls back to defaults instead of raising",
      sm4.fps == S.DEFAULTS["fps"], sm4.fps)

print("opencv available: %s" % HAVE_CV2)
print("FAILS: %s" % (", ".join(FAILS) if FAILS else "none"))
sys.exit(1 if FAILS else 0)
