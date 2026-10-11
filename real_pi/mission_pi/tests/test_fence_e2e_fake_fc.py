"""END-TO-END: browser HTTP -> Pi /api/fence -> FCLink -> MAVLink2 -> "Pixhawk".

Nothing on the path is mocked:
  * the Pi runs its REAL FastAPI app (server.create_app, same as main.py),
  * the REAL production FCLink (fc_link.FCLink, the USB driver class),
  * REAL MAVLink 2 bytes over a REAL TCP socket (pymavlink codec, heartbeat
    thread, reader thread, subscribe fanout, watchdog off),
  * a REAL mission-protocol transaction: MISSION_COUNT -> MISSION_REQUEST_INT
    -> MISSION_ITEM_INT -> MISSION_ACK, then a full download for readback.

Only the flight controller is simulated (tools/fake_sitl.py, in-process on an
ephemeral port). The fake is deliberately STRICT — it rejects cmd != 5001,
param1 != vertex count, wrong frames and bad sequences exactly like ArduPilot —
so a green run here means the upload really is a valid ArduPilot polygon fence.

The fake can also LIE on demand (fence_fault) to prove the Pi/UI never report
success on a hand-wave: acked-then-discarded, shifted geometry, denial, stall.

Run:  python3 -m unittest tests.test_fence_e2e_fake_fc -v
"""

import os
import socket
import sys
import threading
import time
import unittest

os.environ["MAVLINK20"] = "1"          # mission_type FENCE needs MAVLink 2

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.path.dirname(HERE)             # mission_pi/
sys.path.insert(0, PI)
sys.path.insert(0, os.path.join(PI, "tools"))

import httpx                          # noqa: E402
import uvicorn                        # noqa: E402

from fc_link import FCLink            # noqa: E402
import fake_sitl                      # noqa: E402
from server import create_app         # noqa: E402

# ~11 m x ~11 m square near the fake's home (Hubballi).
HOME = (15.36470, 75.12400)
VERTS = [(15.3647000, 75.1240000), (15.3647000, 75.1241000),
         (15.3648000, 75.1241000), (15.3648000, 75.1240000)]


class StubRig:
    """create_app only asks the rig for a status snapshot."""

    def status(self):
        return {}


class StubMission:
    """create_app only reads .fence / .phase / .detail on this path."""

    def __init__(self):
        self.fence = []
        self.phase = "IDLE"
        self.detail = "fence e2e"


class FenceE2ETest(unittest.TestCase):
    """One fake vehicle + one real FCLink + one real HTTP server per test."""

    def setUp(self):
        # 1. the "Pixhawk" — strict, writable, in-process, ephemeral port
        self.veh = fake_sitl.FakeVehicle(HOME, fence_m=0.0)   # starts with NO fence
        self.srv_sock = fake_sitl.serve_port(self.veh, 0, host="127.0.0.1")
        self.stop_ticker = fake_sitl.start_ticker(self.veh)
        self.fc_port = self.srv_sock.getsockname()[1]
        self.device = "tcp:127.0.0.1:%d" % self.fc_port

        # 2. the real Pi FC link over a real socket
        self.fc = FCLink()
        self.fc.connect(device=self.device, supervise=False)
        self.assertTrue(self.fc.link_ok(), "FCLink did not adopt the fake FC")

        # 3. the real Pi HTTP app, exactly as main.py builds it
        self.mission = StubMission()
        app = create_app(StubRig(), self.mission, self.fc, None)

        # test-only fault injection, never installed in production create_app
        from fastapi import Request

        @app.post("/__test/fault")
        async def fault(req: Request):
            body = await req.json()
            # Accept the UI-fixture vocabulary too, so one browser test script
            # can run against either fixture.
            want = body.get("fault", body.get("fail"))
            name = {"upload": "reject", "readback": "discard",
                    "clear": "reject"}.get(want, want)
            self.veh.fence_fault = name
            return {"ok": True, "fault": self.veh.fence_fault}

        sock = socket.socket()
        sock.bind(("127.0.0.1", 0))
        self.http_port = sock.getsockname()[1]
        self.base = "http://127.0.0.1:%d" % self.http_port
        self.uv = uvicorn.Server(uvicorn.Config(app, log_level="error"))
        self.thread = threading.Thread(
            target=lambda: self.uv.run(sockets=[sock]), daemon=True)
        self.thread.start()
        deadline = time.monotonic() + 10
        while not self.uv.started and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(self.uv.started, "uvicorn did not start")
        self.client = httpx.Client(base_url=self.base, timeout=30)
        self.addCleanup(self._teardown)

    def _teardown(self):
        self.stop_ticker.set()
        self.client.close()
        self.uv.should_exit = True
        self.thread.join(5)
        try:
            self.fc._teardown()
        except Exception:
            pass
        try:
            self.srv_sock.close()
        except Exception:
            pass

    # -- helpers ---------------------------------------------------------
    def upload(self, verts=None, **kw):
        return self.client.post("/api/fence", json={"vertices": VERTS if verts is None else verts, **kw})

    def wait_armed(self, want=True, timeout=6.0):
        """Arm/disarm the fake and wait for the Pi's heartbeat view to match."""
        self.veh.arm(want)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if bool(self.fc.armed) is want:
                return True
            time.sleep(0.05)
        return False

    # ==================================================================
    # 0. The contract the UI depends on
    # ==================================================================
    def test_00_health_identifies_the_pi_and_the_fc_device(self):
        """/health must let the UI prove "this origin IS the Pi" and show which
        device the fence will be written through."""
        h = self.client.get("/health").json()
        self.assertEqual(h.get("service"), "mission_pi",
                         "UI writer detection keys off service=mission_pi")
        self.assertTrue(h["link"], "no FC link reported over a live TCP FC")
        self.assertEqual(h["fc"]["device"], self.device)
        self.assertIs(h["fc"]["armed"], False)

    # ==================================================================
    # 1. The happy path the user asked for
    # ==================================================================
    def test_01_upload_from_ui_reaches_the_fc_and_readback_confirms(self):
        """POST /api/fence -> real MAVLink upload -> verified FC readback."""
        r = self.upload()
        self.assertEqual(r.status_code, 200, r.text)
        st = r.json()
        self.assertTrue(st["confirmed"], st)
        self.assertEqual(st["source"], "pi")
        self.assertEqual(st["vertex_count"], 4)
        self.assertEqual(st["fc_count"], 4)
        # THE proof: the vehicle itself now holds these vertices, not the Pi.
        self.assertEqual(len(self.veh.fence), 4)
        for (la, lo), (ea, el) in zip(self.veh.fence, VERTS):
            self.assertAlmostEqual(la, ea, places=6)
            self.assertAlmostEqual(lo, el, places=6)
        self.assertEqual(self.veh.fence_writes, 1)
        # and the Pi's own mission view was adopted from the FC, not the request
        self.assertEqual(len(self.mission.fence), 4)

    def test_02_upload_uses_ardupilot_legal_items(self):
        """Every item the Pi sent was a legal inclusion vertex (the strict fake
        would have ACK-failed otherwise); assert the wire form explicitly."""
        self.assertEqual(self.upload().status_code, 200)
        self.assertIsNone(self.veh.last_fence_error)
        # re-read through the real download path and compare geometry
        got = self.fc.read_fence(timeout=5.0)
        self.assertEqual(len(got), 4)
        for (la, lo), (ea, el) in zip(got, VERTS):
            self.assertAlmostEqual(la, ea, places=6)
            self.assertAlmostEqual(lo, el, places=6)

    def test_03_closed_ring_and_reversed_winding_are_normalised(self):
        """A drawn polygon often repeats the first point / is wound backwards;
        the FC must still receive exactly 4 vertices."""
        closed = VERTS + [VERTS[0]]
        r = self.upload(list(reversed(closed)))
        self.assertEqual(r.status_code, 200, r.text)
        self.assertTrue(r.json()["confirmed"])
        self.assertEqual(len(self.veh.fence), 4, "closing vertex must not be sent")
        self.assertNotEqual(self.veh.fence[0], self.veh.fence[-1])

    def test_04_refresh_reads_the_fc_not_the_ui_cache(self):
        """If Mission Planner (or anything else) changes the fence, a UI refresh
        must show the FC's fence — the FC is the source of truth."""
        self.assertEqual(self.upload().status_code, 200)
        moved = [(15.3650, 75.1230), (15.3650, 75.1234),
                 (15.3654, 75.1234), (15.3654, 75.1230)]
        self.veh.fence = list(moved)                     # somebody else wrote it
        st = self.client.get("/api/fence").json()
        self.assertTrue(st["confirmed"])
        self.assertEqual(len(st["vertices_latlon"]), 4)
        self.assertAlmostEqual(st["vertices_latlon"][0]["lat"], 15.3650, places=6)
        self.assertAlmostEqual(st["vertices_latlon"][0]["lon"], 75.1230, places=6)

    def test_05_clear_reaches_the_fc(self):
        self.assertEqual(self.upload().status_code, 200)
        r = self.client.delete("/api/fence")
        self.assertEqual(r.status_code, 200, r.text)
        st = r.json()
        self.assertTrue(st["confirmed"])
        self.assertTrue(st["cleared"])
        self.assertFalse(st["loaded"])
        self.assertEqual(self.veh.fence, [], "FC still holds a fence after clear")
        self.assertEqual(self.veh.fence_clears, 1)
        self.assertEqual(self.client.get("/api/fence").json()["vertex_count"], 0)

    def test_06_uploading_twice_replaces_instead_of_appending(self):
        self.assertEqual(self.upload().status_code, 200)
        smaller = VERTS[:3] + [(15.3647500, 75.1240500)]
        self.assertEqual(self.upload(smaller).status_code, 200)
        self.assertEqual(len(self.veh.fence), 4)
        self.assertAlmostEqual(self.veh.fence[3][0], 15.3647500, places=6)
        self.assertEqual(self.veh.fence_writes, 2)

    # ==================================================================
    # 2. An FC that lies — ACK is NOT proof
    # ==================================================================
    def test_10_acked_but_discarded_is_reported_as_failure(self):
        """The single most important test: the FC accepts the mission and ACKs,
        but keeps the old fence. Only a readback comparison catches this."""
        self.assertEqual(self.upload().status_code, 200)
        before = list(self.veh.fence)
        self.veh.fence_fault = "discard"
        r = self.upload([(15.3655, 75.1255), (15.3655, 75.1259),
                         (15.3659, 75.1259), (15.3659, 75.1255)])
        self.assertEqual(r.status_code, 502, r.text)
        st = r.json()
        self.assertFalse(st["confirmed"], "acked-but-discarded must not be 'confirmed'")
        self.assertFalse(st["ok"])
        self.assertEqual(self.veh.fence, before, "the fake's fence really did not change")
        self.assertEqual(len(self.mission.fence), 4,
                         "Pi must keep the last FC-verified fence, not the request")
        self.assertEqual(self.mission.fence[0], before[0])

    def test_11_shifted_geometry_is_detected_by_readback(self):
        self.veh.fence_fault = "corrupt"
        r = self.upload()
        self.assertEqual(r.status_code, 502, r.text)
        self.assertFalse(r.json()["confirmed"])
        self.assertEqual(self.mission.fence, [])

    def test_12_denied_upload_is_reported_with_the_reason(self):
        self.veh.fence_fault = "reject"
        r = self.upload()
        self.assertEqual(r.status_code, 502, r.text)
        st = r.json()
        self.assertFalse(st["confirmed"])
        self.assertIn("ack", (st["reason"] + st["detail"]).lower())
        self.assertEqual(self.veh.fence, [])

    def test_13_denied_clear_is_reported(self):
        self.assertEqual(self.upload().status_code, 200)
        self.veh.fence_fault = "reject"
        r = self.client.delete("/api/fence")
        self.assertEqual(r.status_code, 502, r.text)
        self.assertFalse(r.json()["confirmed"])
        self.assertEqual(len(self.veh.fence), 4, "fence must survive a denied clear")

    def test_14_silent_fc_times_out_honestly(self):
        """No MISSION_REQUEST at all: the UI gets a failure, not a spinner."""
        self.veh.fence_fault = "stall"
        t0 = time.monotonic()
        r = self.upload()
        dt = time.monotonic() - t0
        self.assertEqual(r.status_code, 502, r.text)
        self.assertFalse(r.json()["confirmed"])
        self.assertLess(dt, 25, "must fail fast enough to be usable in the UI (%ss)" % dt)
        self.assertEqual(self.veh.fence, [])

    # ==================================================================
    # 3. Safety gates
    # ==================================================================
    def test_20_armed_blocks_upload_and_clear_without_touching_the_fc(self):
        self.assertEqual(self.upload().status_code, 200)
        frozen = list(self.veh.fence)
        self.assertTrue(self.wait_armed(True), "fake never reported armed to the Pi")

        r = self.upload([(15.3655, 75.1255), (15.3655, 75.1259),
                         (15.3659, 75.1259), (15.3659, 75.1255)])
        self.assertEqual(r.status_code, 409, r.text)
        self.assertFalse(r.json()["confirmed"])
        self.assertIn("armed", (r.json()["reason"] + r.json()["detail"]).lower())
        d = self.client.delete("/api/fence")
        self.assertEqual(d.status_code, 409, d.text)
        # nothing may have been written to the flight controller
        self.assertEqual(self.veh.fence, frozen)
        self.assertEqual(self.veh.fence_writes, 1)
        self.assertEqual(self.veh.fence_clears, 0)
        # a read is still allowed while armed (observation is not an edit)
        self.assertEqual(self.client.get("/api/fence").status_code, 200)

        self.assertTrue(self.wait_armed(False), "fake never reported disarmed")
        self.assertEqual(self.upload().status_code, 200)

    def test_21_no_fc_link_is_503_and_never_claims_success(self):
        dev = self.base + "/api/fence"
        self.fc._teardown()                     # Pixhawk unplugged
        self.assertFalse(self.fc.link_ok())
        r = self.client.post(dev, json={"vertices": VERTS})
        self.assertEqual(r.status_code, 503, r.text)
        self.assertFalse(r.json()["confirmed"])
        self.assertIn("link", (r.json()["reason"] + r.json()["detail"]).lower())
        self.assertEqual(self.client.delete(dev).status_code, 503)
        self.assertEqual(self.veh.fence, [])
        self.assertEqual(self.veh.fence_writes, 0)

    # ==================================================================
    # 4. Bad input and double clicks
    # ==================================================================
    def test_30_invalid_geometry_never_reaches_the_fc(self):
        bad = [
            [],                                          # empty
            [[15.3647, 75.1240], [15.3648, 75.1241]],     # 2 vertices
            [[15.3647, 75.1240], [15.3648, 75.1241], [15.3649, 75.1242]],  # collinear
            [[15.3647, 75.1240], [15.3648, 75.1241], [15.3647, 75.1241],
             [15.3648, 75.1240]],                         # self-intersecting bowtie
            [[15.3647, 75.1240], [15.3648, 75.1241], [15.3647, 75.1241],
             [15.3647, 75.1241]],                         # duplicate vertex
            [[91.0, 75.1240], [15.3648, 75.1241], [15.3647, 75.1241]],  # lat range
            [{"lat": 15.3647}, [15.3648, 75.1241], [15.3647, 75.1241]],  # no lon
            "not-a-polygon",
        ]
        for verts in bad:
            with self.subTest(verts=verts):
                r = self.client.post("/api/fence", json={"vertices": verts})
                self.assertEqual(r.status_code, 400, r.text)
        self.assertEqual(self.veh.fence_writes, 0, "bad geometry reached the FC")
        self.assertEqual(self.veh.fence, [])

    def test_31_malformed_bodies_are_rejected(self):
        r = self.client.post("/api/fence", content=b"{not json",
                             headers={"Content-Type": "application/json"})
        self.assertIn(r.status_code, (400, 422))
        r = self.client.post("/api/fence", json={"vertices": [[None, None],
                                                             [15.3648, 75.1241],
                                                             [15.3647, 75.1241]]})
        self.assertEqual(r.status_code, 400, r.text)
        self.assertEqual(self.veh.fence_writes, 0)

    def test_32_double_click_never_interleaves_two_transfers(self):
        """Two 'UPLOAD FENCE' clicks at once must never put two mission
        transfers on the wire together (the FC holds one fence; interleaved
        MISSION_ITEM streams from two writers corrupt it).

        A second click that arrives after the first finished is NOT an error —
        uploading the same polygon twice is idempotent — so the invariant is
        "no overlap, no lost work, honest answers", not "exactly one write".
        """
        results = []
        barrier = threading.Barrier(2)

        def click():
            barrier.wait()
            with httpx.Client(base_url=self.base, timeout=30) as c:
                results.append(c.post("/api/fence", json={"vertices": VERTS}))

        threads = [threading.Thread(target=click) for _ in range(2)]
        [t.start() for t in threads]
        [t.join(30) for t in threads]
        self.assertEqual(len(results), 2)
        codes = sorted(r.status_code for r in results)
        self.assertTrue(all(c in (200, 409) for c in codes), [r.text for r in results])
        self.assertIn(200, codes, [r.text for r in results])
        self.assertEqual(self.veh.concurrent_uploads, 0,
                         "two fence transfers were in flight at the same time")
        # every "200" must be a real, readback-confirmed fence
        for r in results:
            if r.status_code == 200:
                self.assertTrue(r.json()["confirmed"], r.text)
        # the FC holds exactly the polygon that was asked for, whatever the
        # interleaving was
        self.assertEqual(len(self.veh.fence), 4)
        for (la, lo), (ea, el) in zip(self.veh.fence, VERTS):
            self.assertAlmostEqual(la, ea, places=6)
            self.assertAlmostEqual(lo, el, places=6)

    # ==================================================================
    # 4b. Negative control: prove the fake really refuses an illegal fence,
    #     so a green e2e run cannot be a false positive.
    # ==================================================================
    def test_35_fake_fc_rejects_illegal_uploads(self):
        """Send malformed MISSION_ITEM_INTs straight down a raw MAVLink socket.
        If the fake accepted these, every test above would prove nothing."""
        from pymavlink import mavutil as _mu

        def raw_upload(param1, command=5001, frame=None, count=4, items=4):
            conn = _mu.mavlink_connection(self.device, source_system=42,
                                          source_component=191, baud=None)
            try:
                conn.wait_heartbeat(timeout=5)
                conn.mav.mission_count_send(1, 1, count, 1)
                ack_type = None
                deadline = time.time() + 8
                sent = 0
                while time.time() < deadline and ack_type is None:
                    m = conn.recv_match(timeout=1)
                    if m is None:
                        continue
                    t = m.get_type()
                    if t in ("MISSION_REQUEST", "MISSION_REQUEST_INT"):
                        seq = m.seq
                        fr = frame if frame is not None else (
                            _mu.mavlink.MAV_FRAME_GLOBAL_INT
                            if t == "MISSION_REQUEST_INT" else _mu.mavlink.MAV_FRAME_GLOBAL)
                        lat, lon = VERTS[seq % 4]
                        if t == "MISSION_REQUEST_INT":
                            conn.mav.mission_item_int_send(
                                1, 1, seq, fr, command, 0, 1, param1, 0, 0, 0,
                                int(lat * 1e7), int(lon * 1e7), 0.0, 1)
                        else:
                            conn.mav.mission_item_send(
                                1, 1, seq, fr, command, 0, 1, param1, 0, 0, 0,
                                lat, lon, 0.0, 1)
                        sent += 1
                    elif t == "MISSION_ACK":
                        ack_type = m.type
                return ack_type, sent
            finally:
                conn.close()

        self.assertIsNone(self.veh.fence_fault)
        # param1 must equal the TOTAL vertex count — the classic ArduPilot
        # fence-upload trap. The fake must say no.
        ack, _ = raw_upload(param1=1)
        self.assertEqual(ack, _mu.mavlink.MAV_MISSION_INVALID_PARAM1,
                         "fake accepted a fence whose param1 != vertex count")
        # a non-fence command is not a polygon vertex
        ack, _ = raw_upload(param1=4, command=16)      # NAV_WAYPOINT
        self.assertEqual(ack, _mu.mavlink.MAV_MISSION_UNSUPPORTED)
        # wrong frame for an int item
        ack, _ = raw_upload(param1=4, frame=_mu.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
        self.assertEqual(ack, _mu.mavlink.MAV_MISSION_UNSUPPORTED)
        # ...and the vehicle's fence is untouched by all three
        self.assertEqual(self.veh.fence, [])
        self.assertEqual(self.veh.fence_writes, 0)
        # control: the same uploader with a legal item set IS accepted
        ack, sent = raw_upload(param1=4)
        self.assertEqual(ack, _mu.mavlink.MAV_MISSION_ACCEPTED)
        self.assertEqual(sent, 4)
        self.assertEqual(len(self.veh.fence), 4)

    # ==================================================================
    # 4c. The browser itself: real index.html + app.js button clicks, and the
    #     flight controller's own state afterwards. Closes the whole loop.
    # ==================================================================
    def test_45_ui_button_clicks_change_the_fc_fence(self):
        import shutil
        import subprocess

        if shutil.which("node") is None:
            self.skipTest("node not installed")
        ui = os.path.join(PI, "..", "mission-ui")
        probe = subprocess.run(["node", "-e", "require('jsdom')"],
                               capture_output=True, cwd=ui)
        if probe.returncode:
            self.skipTest("optional UI DOM test: npm install in real_pi/mission-ui")

        r = subprocess.run(
            ["node", os.path.join(ui, "tests", "test_fence_dom_to_fake_fc.cjs"), self.base],
            capture_output=True, text=True, timeout=120, cwd=ui)
        print(r.stdout)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn("DONE fence-dom", r.stdout)

        # --- and what the FLIGHT CONTROLLER did while the browser clicked ---
        # The DOM script uploaded the drawn polygon, hit two failure modes and
        # then cleared. The fake FC is the record of truth for all of it.
        self.assertEqual(self.veh.fence, [],
                         "the last UI action was 'clear', so the FC must be empty")
        self.assertEqual(self.veh.fence_clears, 1)
        self.assertEqual(self.veh.concurrent_uploads, 0,
                         "the browser caused overlapping fence transfers")
        # 3 uploads reached the FC: the good one, the 'reject' one (refused at
        # MISSION_COUNT, no write) and the 'discard' one (written, then lost by
        # the fake on purpose). A fence write happened for the first (stored)
        # and the third (acked-but-dropped) => 2 committed writes.
        self.assertGreaterEqual(self.veh.fence_writes, 1)
        # the UI must never have left the FC holding a half-written polygon
        self.assertIn(len(self.veh.fence), (0, 4))

    # ==================================================================
    # 5. The exact bytes the browser sends
    # ==================================================================
    def test_40_ui_json_shape_is_accepted_verbatim(self):
        """Whatever map.js builds ({vertices:[{lat,lon},...]}) must work as-is."""
        body = {"vertices": [{"lat": a, "lon": b} for a, b in VERTS]}
        r = self.client.post("/api/fence", json=body)
        self.assertEqual(r.status_code, 200, r.text)
        self.assertTrue(r.json()["confirmed"])
        self.assertEqual(len(self.veh.fence), 4)
        # and the reply carries everything renderFence() reads
        st = r.json()
        for key in ("ok", "source", "loaded", "confirmed", "reason",
                    "vertex_count", "vertices_latlon", "fc_count"):
            self.assertIn(key, st)


if __name__ == "__main__":
    unittest.main(verbosity=2)
