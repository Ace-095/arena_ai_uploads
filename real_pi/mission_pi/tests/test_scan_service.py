"""ScanService + /api/scan — the UI button that starts QR scanning with NO
mission and NO DO_SPRAYER command in the flight plan.

Contract under test (server.py ScanService):
  * perception only: status always says drives_flight=false;
  * no camera / no detector -> running=false + error TEXT, never an exception;
  * start twice = idempotent; stop when stopped = no-op;
  * a mission camera that is already running is REUSED and never stopped by us;
  * a camera we opened ourselves IS stopped on scan stop;
  * detector failure after opening a camera still releases that camera;
  * the HTTP endpoints answer with the state itself (400 only for a bad body).
"""
import json
import socket
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

PI = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PI))

import cv2
import numpy as np
import httpx
import uvicorn

from detector import BBox
from server import ScanService, create_app


class CountingCam:
    """Minimal camera: counts start/stop so the tests can prove who owns it."""
    def __init__(self, facing="bottom", running=False, frame=None):
        self.facing = facing
        self.running = running
        self.starts = 0
        self.stops = 0
        self.frame = (frame if frame is not None
                      else np.full((120, 160, 3), 140, np.uint8))

    def start(self):
        self.starts += 1
        self.running = True

    def stop(self):
        self.stops += 1
        self.running = False

    def latest(self, raw=False, detect=False):
        return self.frame, time.time(), 1

    def read(self):
        return True, self.frame


class FakeRig:
    def __init__(self, cams=None, detect_raises=False):
        self.cams = cams or {}
        self.detect_raises = detect_raises
        self.detect_calls = 0

    def detect(self):
        self.detect_calls += 1
        if self.detect_raises:
            raise RuntimeError("simulated probe failure")
        return self.cams


class FakeDet:
    def __init__(self, boxes=None):
        self.boxes = boxes if boxes is not None else [
            BBox(10, 10, 40, 40, 0.9, "fake")]
        self.calls = 0

    def detect(self, frame):
        self.calls += 1
        return list(self.boxes)


def _svc(rig, mission=None):
    return ScanService(rig, mission=mission)


class ScanServiceUnit(unittest.TestCase):
    def test_status_is_perception_only(self):
        st = _svc(FakeRig()).status()
        self.assertFalse(st["running"])
        self.assertTrue(st["perception_only"])
        self.assertFalse(st["drives_flight"])

    def test_no_camera_is_error_text_not_exception(self):
        svc = _svc(FakeRig())
        st = svc.start()
        self.assertFalse(st["running"])
        self.assertIn("no camera", st["error"])
        # and stop() after a failed start is a clean no-op
        st = svc.stop()
        self.assertFalse(st["running"])

    def test_detect_crash_is_error_text(self):
        svc = _svc(FakeRig(detect_raises=True))
        st = svc.start()
        self.assertFalse(st["running"])
        self.assertIn("camera", st["error"])

    def test_reuses_running_mission_camera_and_never_stops_it(self):
        cam = CountingCam(running=True)  # mission already owns it
        svc = _svc(FakeRig(cams={"cam": cam}))
        st = svc.start()
        self.assertTrue(st["running"], st.get("error"))
        self.assertFalse(st["opened_camera"])
        self.assertEqual(cam.starts, 0)
        svc.stop()
        self.assertFalse(svc.status()["running"])
        self.assertEqual(cam.stops, 0)  # NOT ours to stop
        self.assertTrue(cam.running)

    def test_opens_then_closes_its_own_camera(self):
        cam = CountingCam(running=False)
        svc = _svc(FakeRig(cams={"bottom": cam}))
        with mock.patch("detector.get_detector", return_value=FakeDet([])):
            st = svc.start()
        self.assertTrue(st["running"], st.get("error"))
        self.assertTrue(st["opened_camera"])
        self.assertEqual(cam.starts, 1)
        st = svc.stop()
        self.assertFalse(st["running"])
        self.assertEqual(cam.stops, 1)

    def test_start_twice_is_idempotent(self):
        cam = CountingCam(running=True)
        svc = _svc(FakeRig(cams={"cam": cam}))
        svc.start()
        first_thread = svc.thread
        st = svc.start()
        self.assertTrue(st["running"])
        self.assertIs(svc.thread, first_thread)  # no second loop
        self.assertEqual(cam.starts, 0)
        svc.stop()

    def test_detector_failure_releases_opened_camera(self):
        cam = CountingCam(running=False)
        svc = _svc(FakeRig(cams={"bottom": cam}))
        with mock.patch("detector.get_detector",
                        side_effect=RuntimeError("no backend")):
            st = svc.start()
        self.assertFalse(st["running"])
        self.assertIn("detector", st["error"])
        self.assertEqual(cam.stops, 1)  # opened, then released

    def test_detector_none_is_error_text(self):
        cam = CountingCam(running=False)
        svc = _svc(FakeRig(cams={"bottom": cam}))
        with mock.patch("detector.get_detector", return_value=None):
            st = svc.start()
        self.assertFalse(st["running"])
        self.assertIn("detector", st["error"])
        self.assertEqual(cam.stops, 1)

    def test_tiled_detection_matches_the_mission_preset(self):
        """No mission -> 'day' preset -> 3x3 tiles: the detector must see
        NINE tiles per frame, not one full-frame pass (an A3 QR at 10 m is
        ~28-40 px in 2028x1520 - invisible to a single 640-px downscale)."""
        cam = CountingCam(running=True)
        svc = _svc(FakeRig(cams={"cam": cam}))
        fake = FakeDet([])
        with mock.patch("detector.get_detector", return_value=fake):
            svc.start()
            self.assertEqual(svc.status()["tiles"], "3x3")
            deadline = time.monotonic() + 4.0
            while svc.status()["frames"] < 1 and time.monotonic() < deadline:
                time.sleep(0.05)
            svc.stop()
        self.assertGreaterEqual(fake.calls, 9)  # tiles really ran

    def test_loop_counts_frames_and_records_payload(self):
        cam = CountingCam(running=True)
        svc = _svc(FakeRig(cams={"cam": cam}))
        fake = FakeDet()
        # single-pass detect here (tiles have their own test above) so the
        # box-count assertion stays about the loop, not the tile merge
        with mock.patch("detector.get_detector", return_value=fake), \
                mock.patch("detector.detect_tiles",
                           side_effect=lambda det, frame, **kw: det.detect(frame)), \
                mock.patch("decoder.decode_frame", return_value="PAY-1"):
            svc.start()
            deadline = time.monotonic() + 4.0
            while svc.status()["frames"] < 2 and time.monotonic() < deadline:
                time.sleep(0.05)
            st = svc.status()
            svc.stop()
        self.assertTrue(st["frames"] >= 2, st)
        self.assertEqual(st["payload"], "PAY-1")
        self.assertEqual(st["boxes"], 1)
        self.assertGreater(st["fps"], 0.0)
        self.assertGreaterEqual(fake.calls, 2)

    def test_frame_source_dying_does_not_kill_the_loop(self):
        cam = CountingCam(running=True)
        cam.latest = lambda **kw: (_ for _ in ()).throw(RuntimeError("stall"))
        cam.read = lambda: (_ for _ in ()).throw(RuntimeError("stall"))
        svc = _svc(FakeRig(cams={"cam": cam}))
        with mock.patch("detector.get_detector", return_value=FakeDet([])):
            st = svc.start()
            self.assertTrue(st["running"], st.get("error"))
            time.sleep(0.3)
            self.assertTrue(svc.status()["running"])  # survived
            st = svc.stop()
        self.assertFalse(st["running"])


class ScanApiLive(unittest.TestCase):
    """Real uvicorn + real httpx against the real endpoints."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cv2.imwrite(cls.tmp.name + "/frame.png",
                    np.full((240, 320, 3), 150, np.uint8))
        cls.rig = FakeRig()
        app = create_app(cls.rig, None, None)
        sock = socket.socket()
        sock.bind(("127.0.0.1", 0))
        cls.port = sock.getsockname()[1]
        cls.base = "http://127.0.0.1:%d" % cls.port
        cls.srv = uvicorn.Server(uvicorn.Config(app, log_level="error"))
        cls.thread = threading.Thread(
            target=lambda: cls.srv.run(sockets=[sock]), daemon=True)
        cls.thread.start()
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            try:
                httpx.get(cls.base + "/health", timeout=0.5)
                break
            except Exception:
                time.sleep(0.1)
        cls.client = httpx.Client(base_url=cls.base, timeout=8)

    @classmethod
    def tearDownClass(cls):
        cls.client.close()
        cls.srv.should_exit = True
        cls.thread.join(timeout=5)
        cls.tmp.cleanup()

    def test_get_state_shape(self):
        r = self.client.get("/api/scan")
        self.assertEqual(r.status_code, 200)
        d = r.json()
        self.assertFalse(d["running"])
        self.assertTrue(d["perception_only"])
        self.assertFalse(d["drives_flight"])

    def test_bad_bodies_are_400(self):
        for body in ({}, {"on": "maybe"}, {"on": None}):
            r = self.client.post("/api/scan", json=body)
            self.assertEqual(r.status_code, 400, body)
        r = self.client.post("/api/scan", content=b"not json",
                             headers={"content-type": "application/json"})
        self.assertEqual(r.status_code, 400)

    def test_start_without_camera_reports_error_in_state(self):
        r = self.client.post("/api/scan", json={"on": True})
        self.assertEqual(r.status_code, 200)  # state, not a 500
        d = r.json()
        self.assertFalse(d["running"])
        self.assertIn("camera", d["error"])
        # still stopped afterwards
        self.assertFalse(self.client.get("/api/scan").json()["running"])

    def test_full_cycle_over_http(self):
        from cameras import FileCamera
        cam = FileCamera("scan-cam", self.tmp.name, (320, 240), 66, "bottom",
                         fps=12)
        self.rig.cams = {"scan-cam": cam}
        # single-pass for HTTP-test speed; tiled behaviour is unit-proven above
        patcher = mock.patch(
            "detector.detect_tiles",
            side_effect=lambda det, frame, **kw: det.detect(frame))
        patcher.start()
        self.addCleanup(patcher.stop)
        try:
            r = self.client.post("/api/scan", json={"on": True})
            d = r.json()
            self.assertTrue(d["running"], d)
            self.assertTrue(d["opened_camera"])
            deadline = time.monotonic() + 6.0
            frames = 0
            while time.monotonic() < deadline:
                d = self.client.get("/api/scan").json()
                frames = d["frames"]
                if frames >= 2:
                    break
                time.sleep(0.15)
            self.assertGreaterEqual(frames, 2)
            r = self.client.post("/api/scan", json={"on": True})  # idempotent
            self.assertTrue(r.json()["running"])
            r = self.client.post("/api/scan", json={"on": False})
            self.assertFalse(r.json()["running"])
            self.assertFalse(cam.running)  # we opened it, we stopped it
        finally:
            self.client.post("/api/scan", json={"on": False})
            self.rig.cams = {}


if __name__ == "__main__":
    unittest.main(verbosity=2)
