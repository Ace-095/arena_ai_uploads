"""SITL integration: fly the REAL mission FSM against a REAL vehicle.

Run directly:   python3 tests/test_sitl_flight.py

Unlike every other test in this folder, these do NOT stub the vehicle. They
open the real `fc_link` to a live ArduCopter SITL (or a real Pixhawk — nothing
here is SITL-specific) and assert what the mission actually does to it:

  * the link comes up and the vehicle's own mode/sysid are readable
  * SNAPSHOT reads home / fence / plan off the vehicle
  * the mission sits in WAIT_TRIGGER and sends NO arming command while waiting
  * a takeover command really puts the vehicle into GUIDED
  * the perception tap observes the live mission without disturbing it

WHEN IT SKIPS
-------------
No SITL listening -> every test here SKIPS with a message, so the normal
`tools/run_tests.sh` run on a laptop stays green. That is deliberate: a test
that silently passes when the vehicle is missing is worse than no test.

HOW TO RUN IT FOR REAL
----------------------
  # terminal 1 - real ArduCopter SITL
  cd ~/ardupilot && Tools/autotest/sim_vehicle.py -v ArduCopter -f quad \
      --no-mavproxy -w
  # (or the repo's own fake vehicle: python3 tools/fake_sitl.py --ports 5760,5762)

  # terminal 2
  cd real_pi/mission_pi
  MISSION_PI_SITL=tcp:127.0.0.1:5762 python3 tests/test_sitl_flight.py

SAFETY: this test emits real MAVLink. Point it at a SITL or a PROPELLER-OFF
bench vehicle only. It commands GUIDED and (after takeover) arms, because that
is exactly the behaviour under test.
"""
import os
import sys
import threading
import time
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_ROOT, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

#: Point this at any vehicle. Default is the repo's documented SITL serial1 port.
SITL = os.environ.get("MISSION_PI_SITL", "tcp:127.0.0.1:5762")
TIMEOUT = float(os.environ.get("MISSION_PI_SITL_TIMEOUT", "30"))


#: Real ArduPilot SITL only streams on SERIAL1 (tcp 5762) once SERIAL0
#: (tcp 5760) has had a client - VERIFIED against a real arducopter build:
#: connecting straight to 5762 yields no heartbeat at all, and holding a GCS on
#: 5760 makes 5762 answer immediately. Mission Planner normally provides that
#: client. This stands in for it so the harness works unattended.
#: fake_sitl binds both ports up front, so priming is a harmless no-op there.
_PRIMER = None


def _prime_serial0(sitl=SITL):
    """Hold a throwaway GCS connection on the SERIAL0 port, once per process."""
    global _PRIMER
    if _PRIMER is not None:
        return _PRIMER
    try:
        from pymavlink import mavutil
    except Exception:
        return None
    if not str(sitl).startswith("tcp:"):
        return None
    try:
        host, _, port = str(sitl)[4:].partition(":")
        port = int(port or 5762)
    except Exception:
        return None
    if port != 5762:                 # only SERIAL1 needs the nudge
        return None
    try:
        conn = mavutil.mavlink_connection("tcp:%s:5760" % host,
                                          source_system=255)
        if conn.wait_heartbeat(timeout=5):
            _PRIMER = conn
            # SITL brings SERIAL1 up a beat after SERIAL0 gets its client; a
            # connect that lands in that gap sees "no heartbeat" and looks like
            # a broken link. Give it a moment before anyone tries 5762.
            time.sleep(1.5)
            import atexit
            atexit.register(lambda: _safe_close(conn))
            return conn
        _safe_close(conn)
    except Exception:
        pass
    return None


def _safe_close(conn):
    try:
        conn.close()
    except Exception:
        pass


def _wait_ready(fc, timeout=10.0):
    """Wait until the link has actually decoded a vehicle heartbeat.

    `connect()` returns as soon as ONE heartbeat is seen, so `link_ok()` is
    already true - but `mode` and `hb_sources()` fill in only once the reader
    thread has processed messages. Production waits for this in WAIT_LINK.
    Returns True if the vehicle identified itself before the timeout.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        if getattr(fc, "mode", None) and fc.hb_sources():
            return True
        time.sleep(0.2)
    return False


def _connect(retries=4, delay=2.0):
    """FCLink.connect with retries, and the SERIAL0 primer held.

    A single failed connect must not fail the run: SITL drops and re-binds
    SERIAL1 as clients come and go, so one retry is normal operation.
    """
    from fc_link import FCLink
    last = None
    for attempt in range(retries):
        _prime_serial0()
        fc = FCLink()
        try:
            fc.connect(device=SITL, supervise=True)
            _wait_ready(fc)
            return fc
        except Exception as e:
            last = e
            try:
                fc.close()
            except Exception:
                pass
            time.sleep(delay)
    raise RuntimeError("could not connect to %s after %d attempts: %s"
                       % (SITL, retries, last))


def _reset_vehicle(fc, mode="STABILIZE"):
    """Put the vehicle back to a known, disarmed, non-GUIDED state.

    Two reasons this is not optional:
      * the mission has a mid-flight-reboot guard: booting into a FRESH GUIDED
        puts it straight into FAILSAFE+RTL (correct, and tested deliberately
        below). A previous test that set GUIDED would trip it by accident.
      * NOTE fake_sitl AUTO-ARMS on AUTO/GUIDED ("auto-armed for %s"); real
        ArduPilot does not. So a vehicle being armed after a mode change is a
        fake-vehicle artifact, not a mission bug - reset before asserting.
    """
    try:
        fc.set_mode(mode, timeout=5.0)
    except Exception:
        try:
            fc.set_mode(mode)
        except Exception:
            pass
    try:
        if getattr(fc, "armed", False):
            fc.arm(False)
    except Exception:
        pass
    deadline = time.time() + 6.0
    while time.time() < deadline and getattr(fc, "armed", False):
        time.sleep(0.2)


def _skip_reason():
    """None if a vehicle answers, else a human sentence explaining the skip."""
    _prime_serial0()
    try:
        from fc_link import find_fc
    except Exception as e:                                        # pragma: no cover
        return "fc_link unavailable (%r)" % (e,)
    try:
        conn, dev = find_fc(device=SITL, hb_timeout=3.0)
    except Exception as e:
        return ("no vehicle answered at %s (%s) — start SITL or fake_sitl.py, "
                "or set MISSION_PI_SITL" % (SITL, e))
    try:
        conn.close()
    except Exception:
        pass
    return None


SKIP = _skip_reason()
CFG_PATH = os.path.join(_ROOT, "config.yaml")


def _load_cfg(**overrides):
    """The mission's real config, with the pieces a bench test must change."""
    import yaml
    with open(CFG_PATH) as fh:
        cfg = yaml.safe_load(fh) or {}
    # never fly a test mission at full altitude, and keep the trigger window short
    cfg.setdefault("mission", {})
    cfg["mission"].setdefault("flight", {})
    cfg["mission"]["max_alt_m"] = overrides.get("max_alt_m", 3.0)
    cfg["mission"]["sweep_alt_m"] = overrides.get("sweep_alt_m", 3.0)
    return cfg


class _Harness(object):
    """A live FC + the mission objects, torn down even if the test fails."""

    def __init__(self, cfg=None, rig=None, reset=True, fc=None):
        from mission import Mission
        # SITL's SERIAL1 serves ONE client at a time (verified against a real
        # arducopter build: a second connection gets no heartbeat until the
        # first is closed). So a caller that already holds a link MUST pass it
        # in rather than letting the harness open a rival one.
        self.cfg = cfg or _load_cfg()
        self.mode_calls = []
        # FCLink is the wrapper mission.py actually talks to (link_ok/mode/
        # set_mode/subscribe). find_fc returns the RAW pymavlink connection and
        # has none of those - using it here was a bug in the first draft.
        # supervise=True mirrors production: the watchdog is part of the
        # behaviour under test, not scaffolding around it.
        self.fc = fc if fc is not None else _connect()
        self.dev = self.fc.device
        self._owns_fc = fc is None
        if reset:
            _reset_vehicle(self.fc)     # never inherit GUIDED from an earlier test
        # Record every mode command the MISSION issues. The mission cannot arm
        # at all (FCLink has no arm() - see the structural test below), so this
        # list is the complete set of flight-affecting commands it can emit
        # before TAKEOVER.
        _orig_set_mode = self.fc.set_mode

        def _recording_set_mode(mode, *a, **k):
            self.mode_calls.append(mode)
            return _orig_set_mode(mode, *a, **k)
        self.fc.set_mode = _recording_set_mode
        self.hub = _Hub()
        self.rig = rig if rig is not None else _StubRig()
        self.mission = Mission(self.fc, self.rig, None, hub=self.hub, cfg=self.cfg)
        self.thread = None

    def start_mission(self):
        self.thread = threading.Thread(target=self.mission.run, name="mission",
                                       daemon=True)
        self.thread.start()

    def wait_phase(self, *wanted, timeout=TIMEOUT):
        """Block until the mission reaches one of `wanted`. Returns the phase."""
        deadline = time.time() + timeout
        seen = []
        while time.time() < deadline:
            p = self.mission.phase
            if p not in seen:
                seen.append(p)
            if p in wanted:
                return p
            time.sleep(0.2)
        return None

    def close(self):
        try:
            self.mission.stop()
        except Exception:
            pass
        if self.thread:
            self.thread.join(timeout=5.0)
        if getattr(self, "_owns_fc", True):
            try:
                self.fc.close()
            except Exception:
                pass


class _Hub(object):
    """Captures exactly what the real websocket hub would have pushed."""

    def __init__(self):
        self.events = []
        self._lock = threading.Lock()

    def push(self, kind, payload=None):
        with self._lock:
            self.events.append((kind, payload))

    def logs(self):
        with self._lock:
            return [p.get("msg", "") for k, p in self.events if k == "log"]

    def phases(self):
        with self._lock:
            return [p.get("phase") for k, p in self.events if k == "fsm"]


class _StubRig(object):
    """No cameras: this class tests the FC/mission contract, not vision."""

    cams = {}

    def get(self, name):
        return None

    def __iter__(self):
        return iter(())


@unittest.skipIf(SKIP, SKIP or "")
class TestVehicleLink(unittest.TestCase):
    """The link layer, against a real vehicle."""

    def setUp(self):
        self.fc = _connect()
        self.dev = self.fc.device
        self.ready = bool(getattr(self.fc, "mode", None))
        _reset_vehicle(self.fc)

    def tearDown(self):
        try:
            self.fc.close()
        except Exception:
            pass

    def test_link_is_up_and_heartbeats_are_fresh(self):
        self.assertTrue(self.fc.link_ok(), "link_ok() false against a live vehicle")
        age = time.time() - getattr(self.fc, "last_hb", time.time())
        self.assertLess(age, 3.0, "heartbeat older than the 3 s link_ok threshold")

    def test_vehicle_identifies_itself(self):
        self.assertIsNotNone(self.fc.target_system)
        self.assertGreaterEqual(self.fc.target_system, 1)
        self.assertLessEqual(self.fc.target_system, 255)

    def test_mode_is_readable(self):
        """A blank/None mode means the mode channel never got decoded, which
        would silently disable the AUTO gate on the trigger."""
        self.assertTrue(self.ready,
                        "the vehicle never identified itself (mode/hb_sources "
                        "still empty after 10 s) — the link is up but the "
                        "reader is not decoding, which would silently disable "
                        "the trigger's AUTO gate")
        self.assertTrue(self.fc.mode,
                        "fc.mode is empty — mode decode broken (trigger AUTO gate "
                        "and every mode assertion depend on it)")

    def test_hb_sources_reports_the_vehicle(self):
        src = self.fc.hb_sources()
        self.assertTrue(src, "hb_sources() empty despite a live link")
        self.assertGreaterEqual(max(src.values()), 1)

    def test_set_mode_round_trips(self):
        """Command GUIDED, read it back. If this fails the whole takeover path
        is broken and nothing downstream can be trusted."""
        self.fc.set_mode("GUIDED", timeout=5.0)
        deadline = time.time() + 8.0
        while time.time() < deadline and self.fc.mode != "GUIDED":
            time.sleep(0.2)
        self.assertEqual(self.fc.mode, "GUIDED",
                         "set_mode('GUIDED') did not take effect on the vehicle")

    def test_00_vehicle_is_disarmed_after_reset(self):
        """Runs FIRST (name sorts first) and guards the harness: if a previous
        run left the vehicle armed, every arming assertion below passes
        vacuously. _reset_vehicle() must have disarmed it."""
        self.assertFalse(getattr(self.fc, "armed", False),
                         "vehicle still ARMED after _reset_vehicle() — either the "
                         "reset is broken or something re-armed it")


@unittest.skipIf(SKIP, SKIP or "")
class TestMissionSnapshotAgainstSitl(unittest.TestCase):
    """SNAPSHOT + WAIT_TRIGGER against the real vehicle."""

    @classmethod
    def setUpClass(cls):
        cls.h = _Harness()
        cls.h.start_mission()
        cls.phase = cls.h.wait_phase("WAIT_TRIGGER", "FAILSAFE", "TAKEOVER")
        cls.logs = cls.h.hub.logs()

    @classmethod
    def tearDownClass(cls):
        cls.h.close()

    def test_reaches_wait_trigger(self):
        self.assertEqual(
            self.phase, "WAIT_TRIGGER",
            "mission did not reach WAIT_TRIGGER (got %r). Logs:\n  %s"
            % (self.phase, "\n  ".join(self.logs[-15:])))

    def test_snapshot_read_home_off_the_vehicle(self):
        self.assertIsNotNone(self.h.mission.home,
                             "home is None after SNAPSHOT")
        lat, lon = self.h.mission.home[0], self.h.mission.home[1]
        self.assertTrue(-90 <= lat <= 90 and -180 <= lon <= 180,
                        "home looks like garbage: %r" % (self.h.mission.home,))

    def test_phase_order_is_the_documented_one(self):
        """WAIT_LINK must precede SNAPSHOT must precede WAIT_TRIGGER."""
        phases = self.h.hub.phases()
        for first, second in (("WAIT_LINK", "SNAPSHOT"), ("SNAPSHOT", "WAIT_TRIGGER")):
            self.assertIn(first, phases, "%s never announced (saw %s)" % (first, phases))
            self.assertIn(second, phases, "%s never announced (saw %s)" % (second, phases))
            self.assertLess(phases.index(first), phases.index(second),
                            "%s came after %s — FSM order is wrong" % (first, second))

    def test_no_mode_command_while_waiting_for_the_trigger(self):
        """THE safety property of WAIT_TRIGGER: it is a bystander.

        It reads the plan and the fence and obeys nothing else. The vehicle is
        normally ARMED here and that is CORRECT — it is flying its own AUTO
        sprayer mission and mission_pi is watching. What must not happen is
        mission_pi ISSUING a command: no mode change, no arm.

        (The original version of this test asserted the vehicle was disarmed.
        That was wrong: it would have failed in the field on a correctly
        behaving system, mislabelling normal operation as a safety breach.)
        """
        self.assertEqual(
            self.h.mode_calls, [],
            "mission issued mode command(s) %r while merely WAITING for a "
            "trigger — it must stay a bystander until TAKEOVER"
            % (self.h.mode_calls,))

    def test_mission_structurally_cannot_arm(self):
        """mission_pi must have no way to arm the vehicle at all.

        This is stronger than a behavioural check: FCLink exposes no arm/arm()
        method, and mission.py contains no arming code path. If someone adds
        one, this test fails and the change gets the review it deserves.
        """
        from fc_link import FCLink
        for forbidden in ("arm", "disarm", "arming", "takeoff"):
            self.assertFalse(
                hasattr(FCLink, forbidden),
                "FCLink grew a %r method — mission_pi could now arm the "
                "vehicle, which the additive/no-flight-control design forbids"
                % forbidden)
        with open(os.path.join(_ROOT, "mission.py")) as fh:
            body = fh.read().lower()
        for forbidden in ("component_arm_disarm", "mav_cmd_nav_takeoff"):
            self.assertNotIn(
                forbidden, body,
                "mission.py now references %r — it must never command arm or "
                "takeoff (the operator/GCS owns those)" % forbidden)

    def test_link_is_still_healthy(self):
        self.assertTrue(self.h.fc.link_ok(),
                        "the link died while the mission was in WAIT_TRIGGER")


@unittest.skipIf(SKIP, SKIP or "")
class TestTakeoverCommandsGuided(unittest.TestCase):
    """TAKEOVER must actually command the vehicle, and be observable."""

    def setUp(self):
        self.h = _Harness()
        self.h.start_mission()
        self.assertIsNotNone(
            self.h.wait_phase("WAIT_TRIGGER", "FAILSAFE"),
            "never reached WAIT_TRIGGER; cannot test takeover")

    def tearDown(self):
        self.h.close()

    def test_manual_takeover_moves_the_mission_out_of_wait_trigger(self):
        self.h.mission.request_takeover()
        got = self.h.wait_phase("TAKEOVER", "SWEEP_YAW", "FAILSAFE", timeout=TIMEOUT)
        self.assertIsNotNone(got, "mission never left WAIT_TRIGGER after request_takeover()")
        self.assertNotEqual(got, "FAILSAFE",
                            "takeover drove the mission into FAILSAFE: %s"
                            % self.h.mission.detail)

    def test_takeover_sets_guided_on_the_vehicle(self):
        self.h.mission.request_takeover()
        self.h.wait_phase("TAKEOVER", "SWEEP_YAW", "FAILSAFE", timeout=TIMEOUT)
        deadline = time.time() + 10.0
        while time.time() < deadline and self.h.fc.mode != "GUIDED":
            time.sleep(0.2)
        self.assertEqual(self.h.fc.mode, "GUIDED",
                         "TAKEOVER did not put the vehicle into GUIDED (mode=%r)"
                         % self.h.fc.mode)

    def test_abort_after_takeover_does_not_crash(self):
        """request_abort() is what an operator hits in a panic. It must land the
        FSM somewhere sane and never raise out of the mission thread."""
        self.h.mission.request_takeover()
        self.h.wait_phase("TAKEOVER", "SWEEP_YAW", "FAILSAFE", timeout=TIMEOUT)
        self.h.mission.request_abort()
        deadline = time.time() + 15.0
        while time.time() < deadline and self.h.mission.phase not in (
                "FAILSAFE", "DONE"):
            time.sleep(0.2)
        # the mission thread must have exited, not spun on
        self.h.thread.join(timeout=10.0)
        self.assertFalse(self.h.thread.is_alive(),
                        "mission thread still running after abort() (phase=%s)"
                        % self.h.mission.phase)


@unittest.skipIf(SKIP, SKIP or "")
class TestGuidedAtBootSafetyGuard(unittest.TestCase):
    """A mission_pi that boots into a FRESH GUIDED must RTL, not carry on.

    This is the "a prior run died mid-takeover" guard in Mission._wait_trigger.
    It was found by accident (a test left the vehicle in GUIDED and the next
    mission went straight to FAILSAFE) — which is exactly why it is pinned here
    rather than left as a surprise for the field.
    """

    def setUp(self):
        self.fc = _connect()
        self.dev = self.fc.device
        # put the vehicle in a FRESH GUIDED - the state a dead run leaves behind
        self.fc.set_mode("GUIDED", timeout=5.0)
        deadline = time.time() + 6.0
        while time.time() < deadline and self.fc.mode != "GUIDED":
            time.sleep(0.2)

    def tearDown(self):
        try:
            _reset_vehicle(self.fc)
            self.fc.close()
        except Exception:
            pass

    def test_fresh_guided_at_boot_drives_failsafe(self):
        if self.fc.mode != "GUIDED":
            self.skipTest("could not put the vehicle into GUIDED (%r)" % self.fc.mode)
        # reset=False is the whole point: the guard under test triggers on a
        # vehicle that is ALREADY in a fresh GUIDED when mission_pi boots.
        # reuse THIS test's link - SITL will not serve a second one on 5762
        h = _Harness(reset=False, fc=self.fc)
        # the mission must observe GUIDED on its own link, not just on ours
        deadline = time.time() + 8.0
        while time.time() < deadline and h.fc.mode != "GUIDED":
            time.sleep(0.2)
        self.assertEqual(h.fc.mode, "GUIDED",
                         "the second connection did not see the fresh GUIDED "
                         "(mode=%r) - the guard cannot be tested" % h.fc.mode)
        try:
            h.start_mission()
            got = h.wait_phase("FAILSAFE", "WAIT_TRIGGER", timeout=TIMEOUT)
            self.assertEqual(
                got, "FAILSAFE",
                "booting into a fresh GUIDED must FAILSAFE+RTL, but the mission "
                "went on to %r" % got)
            logs = " ".join(h.hub.logs())
            self.assertIn("GUIDED", logs,
                          "FAILSAFE happened but not for the GUIDED reason: %s" % logs)
        finally:
            h.close()


@unittest.skipIf(SKIP, SKIP or "")
class TestPerceptionTapUnderLiveMission(unittest.TestCase):
    """The additive layer, on a real mission thread, against a real vehicle."""

    def setUp(self):
        import perception_tap
        self.pt = perception_tap
        self.was = self.pt.INSTALLER

    def tearDown(self):
        try:
            self.pt.uninstall()
        except Exception:
            pass
        if self.was is not None:
            self.was = None

    def test_install_survives_a_mission_that_is_already_running(self):
        """Installing mid-mission must not disturb the running task."""
        h = _Harness()
        try:
            h.start_mission()
            self.assertIsNotNone(h.wait_phase("WAIT_TRIGGER", "FAILSAFE"))
            before = h.mission.phase
            self.assertIsNotNone(self.pt.install(_load_cfg()),
                                 "install() returned None with perception enabled")
            time.sleep(1.0)
            self.assertEqual(h.mission.phase, before,
                             "installing the tap moved the mission out of %s" % before)
            self.assertTrue(h.fc.link_ok(), "link dropped after installing the tap")
        finally:
            self.pt.uninstall()
            h.close()

    def test_uninstall_restores_the_original_functions(self):
        import detector
        import decoder
        orig_tiles = detector.detect_tiles
        orig_decode = decoder.decode_frame
        self.pt.install(_load_cfg())
        self.assertIsNot(detector.detect_tiles, orig_tiles,
                         "install() did not wrap detect_tiles")
        self.pt.uninstall()
        self.assertIs(detector.detect_tiles, orig_tiles,
                      "uninstall() did not restore detect_tiles by identity")
        self.assertIs(decoder.decode_frame, orig_decode,
                      "uninstall() did not restore decode_frame by identity")

    def test_perception_never_drives_the_vehicle(self):
        """The contract the whole design rests on."""
        self.pt.install(_load_cfg())
        snap = self.pt.snapshot()
        self.assertIs(snap.get("drives_flight"), False,
                      "perception reports drives_flight=%r — it must always be False"
                      % snap.get("drives_flight"))
        self.assertIs(snap.get("perception_only"), True)


if __name__ == "__main__":
    if SKIP:
        print("SKIP: %s" % SKIP)
        print("These tests need a live vehicle (SITL or a props-off bench FC).")
        print("Start one, e.g.:  python3 tools/fake_sitl.py --ports 5760,5762")
        print("and re-run:       MISSION_PI_SITL=tcp:127.0.0.1:5762 "
              "python3 tests/test_sitl_flight.py")
    unittest.main(verbosity=2)
