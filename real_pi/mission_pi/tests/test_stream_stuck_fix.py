#!/usr/bin/env python3
"""Stdlib tests for the real_pi video-stuck fix (streamm1 + SendGate).

The bug: the MJPEG endpoint only emitted bytes when a NEW frame seq
appeared. Before the first frame (CSI start, AE/AWB settle, YOLO load —
seconds on a Pi 5) it emitted NOTHING: the browser <img> held an open
multipart connection with zero bytes = the tile "always stuck".

Locked down here:
  * SendGate: first frame always sent; new seq sent; same seq within the
    window NOT re-sent (no bandwidth waste); same seq after the window
    RE-sent (liveness — a stuck camera can't freeze the tile);
  * CamStream.first_frame: bounded wait, True on frame, False on timeout;
  * CamStream watchdog: a camera that goes dark is restarted automatically
    (bounded, with cooldown) and the restart is counted;
  * adaptive: an overloaded encoder degrades the fps/width/quality ladder;
    a light encoder recovers.
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                ".."))
import streamm1 as S  # noqa

FAILS = []


def check(name, cond, extra=""):
    print(("PASS " if cond else "FAIL ") + name, extra if not cond else "")
    if not cond:
        FAILS.append(name)


JPEG = b"\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\xff\xd9"


class DarkCam:
    """A camera that can go dark on demand (and records restarts)."""

    def __init__(self, name="cam1"):
        self.name = name
        self.running = True
        self.dark = False
        self.restarts = 0
        self._ts = 0.0
        self._count = 0

    def latest(self, raw=False):
        if not self.dark:
            self._ts = time.time()
            self._count += 1
        return b"\x00", self._ts, self._count

    def jpeg(self, max_width=960, quality=70, **kw):
        if self.dark:
            return None
        return JPEG

    def stop(self):
        self.running = False
        self.restarts += 1  # stand-in for the real restart work

    def start(self):
        self.running = True
        self.dark = False  # the reopen heals it
        self._ts = time.time()


class SlowCam(DarkCam):
    """jpeg() burns most of the tick — the encoder-overload case."""

    def __init__(self, name="cam1", ms=150):
        super().__init__(name)
        self.ms = ms

    def jpeg(self, max_width=960, quality=70, **kw):
        time.sleep(self.ms / 1000.0)
        return JPEG


# ---------------------------------------------------------------- SendGate --
def test_send_gate():
    g = S.SendGate(resend_after=1.0)
    t0 = 1000.0
    check("gate: first frame always sent", g.should_send(1, t0) is True)
    check("gate: same seq inside window suppressed",
          g.should_send(1, t0 + 0.2) is False)
    check("gate: new seq sent", g.should_send(2, t0 + 0.3) is True)
    check("gate: same seq after window re-sent (liveness)",
          g.should_send(2, t0 + 1.5) is True)
    check("gate: state tracked", g.last_seq == 2)


def test_send_gate_minimum_window():
    g = S.SendGate(resend_after=0.01)  # absurdly small -> clamped to 0.25
    check("gate: window clamped to >= 0.25s", g.resend_after == 0.25)


# ------------------------------------------------------------ first_frame --
def test_first_frame():
    cam = DarkCam()
    st = S.CamStream(cam, fps=30, width=320, quality=50)
    st.start()
    ok = st.first_frame(2.0)
    st.stop()
    check("first_frame: True once the camera produces frames", ok is True)

    dark = DarkCam()
    dark.dark = True
    st2 = S.CamStream(dark, fps=30, width=320, quality=50)
    st2.start()
    t0 = time.time()
    ok2 = st2.first_frame(0.6)
    took = time.time() - t0
    st2.stop()
    check("first_frame: False on a dark camera", ok2 is False)
    check("first_frame: wait is bounded (~0.6s, got %.2fs)" % took,
          0.4 <= took <= 2.0)


# ---------------------------------------------------------------- watchdog --
def test_watchdog_restarts_dark_camera():
    cam = DarkCam()
    st = S.CamStream(cam, fps=10, width=320, quality=50, watchdog_s=2.0)
    st._restart_cooldown_s = 0.5  # test speed
    st.start()
    time.sleep(0.3)          # one healthy frame first
    cam.dark = True          # camera goes dark
    deadline = time.time() + 8.0
    while time.time() < deadline and cam.restarts < 1:
        time.sleep(0.1)
    st.stop()
    check("watchdog: dark camera restarted (restarts=%d, stream=%d)"
          % (cam.restarts, st._restarts), cam.restarts >= 1
          and st._restarts >= 1)
    check("watchdog: status reports restarts",
          st.status().get("restarts", 0) >= 1)


def test_watchdog_cooling_prevents_restart_storm():
    class AlwaysDark(DarkCam):
        def jpeg(self, **kw):
            return None

        def latest(self, raw=False):
            # a camera that HAD frames and then died: stale non-zero ts
            return None, time.time() - 100.0, 0

    bad = AlwaysDark()
    st = S.CamStream(bad, fps=10, width=320, quality=50, watchdog_s=1.0)
    st._restart_cooldown_s = 3.0
    st.start()
    time.sleep(5.0)  # restart + 0.3s heal + cooldown: at most 2 fires
    st.stop()
    check("watchdog: cooldown bounds restarts (got %d in 5s)"
          % st._restarts, 1 <= st._restarts <= 2)


# ----------------------------------------------------------------- adaptive --
def test_adaptive_degrades_under_load():
    cam = SlowCam(ms=150)  # 150ms encode vs 83ms tick @12fps -> overloaded
    st = S.CamStream(cam, fps=12, width=960, quality=70, adaptive=True)
    st.start()
    deadline = time.time() + 6.0
    while time.time() < deadline and st._level == 0:
        time.sleep(0.1)
    st.stop()
    check("adaptive: overloaded encoder degrades (level=%d, %dfps %dw q%d)"
          % (st._level, st.fps, st.width, st.quality), st._level >= 1)
    check("adaptive: degraded below base rate",
          (st.fps, st.width, st.quality) < (12, 960, 70)
          or st.fps < 12 or st.width < 960 or st.quality < 70)


def test_adaptive_disabled_stays_put():
    cam = SlowCam(ms=150)
    st = S.CamStream(cam, fps=12, width=960, quality=70, adaptive=False)
    st.start()
    time.sleep(1.0)
    st.stop()
    check("adaptive: disabled -> no degradation", st._level == 0
          and st.fps == 12 and st.width == 960 and st.quality == 70)


# ------------------------------------------------------------------ manager --
def test_manager_config_passthrough():
    cam = DarkCam()
    rig = type("Rig", (), {"cams": {"cam1": cam}})()
    m = S.StreamManager(rig, {"fps": 9, "width": 640, "quality": 60,
                              "watchdog_s": 3.0, "adaptive": False,
                              "first_frame_wait_s": 4.0})
    s = m.get("cam1")
    check("manager: cfg -> CamStream", s.fps == 9 and s.width == 640
          and s.quality == 60 and s.watchdog_s == 3.0
          and s.adaptive is False)
    check("manager: first_frame_wait_s stored",
          m.first_frame_wait_s == 4.0)
    m2 = S.StreamManager(rig, {})  # defaults
    s2 = m2.get("cam1")
    check("manager: defaults sane", s2.fps == 12 and s2.width == 960
          and s2.watchdog_s == S.DEFAULTS["watchdog_s"]
          and s2.adaptive is True
          and m2.first_frame_wait_s == S.DEFAULTS["first_frame_wait_s"])


if __name__ == "__main__":
    t0 = time.time()
    test_send_gate()
    test_send_gate_minimum_window()
    test_first_frame()
    test_watchdog_restarts_dark_camera()
    test_watchdog_cooling_prevents_restart_storm()
    test_adaptive_degrades_under_load()
    test_adaptive_disabled_stays_put()
    test_manager_config_passthrough()
    print("-" * 60)
    if FAILS:
        print("FAILED %d/%d: %s" % (len(FAILS), 8, ", ".join(FAILS)))
        sys.exit(1)
    print("all stream-stuck-fix tests passed (%.1fs)" % (time.time() - t0))
