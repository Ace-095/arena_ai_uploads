"""MAVLink2 wire regression: production FCLink + deterministic simulated FC.
No hardware. Replies arrive synchronously inside send (worst ACK race).
"""
import os
import sys
import unittest
from types import SimpleNamespace
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from fc_link import FCLink, FCError, mavutil
from fence import normalize, matches, quant_tolerance

VERTS = [(0, 0), (0, .001), (.001, .001), (.001, 0)]


class Peer:
    def __init__(self, fc):
        self.fc = fc
        self.parser = mavutil.mavlink.MAVLink(None)
        self.encoder = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
        self.client_parser = mavutil.mavlink.MAVLink(None)
        self.items = {}
        self.wire = []
        self.requests = [2, 0, 0, 3, 1]  # re-request and out of order
        self.reject = False
        self.silent = False
        self.legacy = False

    def reply(self, msg):
        decoded = self.client_parser.parse_char(msg.pack(self.encoder))
        for q in self.fc._subs.get(decoded.get_type(), []):
            q.put_nowait(decoded)

    def request(self):
        if self.requests:
            seq = self.requests.pop(0)
            ctor = (mavutil.mavlink.MAVLink_mission_request_message if self.legacy
                    else mavutil.mavlink.MAVLink_mission_request_int_message)
            self.reply(ctor(51, 191, seq, 1))
        else:
            self.reply(mavutil.mavlink.MAVLink_mission_ack_message(51, 191, 0, 1))

    def write(self, data):
        m = self.parser.parse_char(data)
        self.wire.append(m)
        if self.silent:
            return
        typ = m.get_type()
        if typ == 'MISSION_COUNT':
            if self.reject:
                self.reply(mavutil.mavlink.MAVLink_mission_ack_message(51, 191, 14, 1))
            else:
                self.request()
        elif typ in ('MISSION_ITEM_INT', 'MISSION_ITEM'):
            self.items[m.seq] = m
            self.request()
        elif typ == 'MISSION_CLEAR_ALL':
            if not self.reject:
                self.items.clear()
            self.reply(mavutil.mavlink.MAVLink_mission_ack_message(51, 191, 14 if self.reject else 0, 1))
        elif typ in ('PARAM_REQUEST_READ', 'PARAM_SET'):
            self.reply(mavutil.mavlink.MAVLink_param_value_message(
                b'FENCE_ALT_MAX', m.param_value if typ == 'PARAM_SET' else 15,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32, 1, 0))
        elif typ == 'MISSION_REQUEST_LIST':
            self.reply(mavutil.mavlink.MAVLink_mission_count_message(51, 191, len(self.items), 1))
        elif typ == 'MISSION_REQUEST_INT':
            self.reply(self.items[m.seq])


@unittest.skipIf(mavutil is None, 'needs pymavlink')
class ProtocolTest(unittest.TestCase):
    def setUp(self):
        self.fc = FCLink()
        self.peer = Peer(self.fc)
        self.fc.conn = SimpleNamespace(mav=mavutil.mavlink.MAVLink(self.peer, srcSystem=51, srcComponent=191))

    def test_upload_wire_and_fast_readback(self):
        self.assertEqual(self.fc.upload_fence(VERTS + [VERTS[0]], timeout=.3), 4)
        items = [m for m in self.peer.wire if m.get_type() == 'MISSION_ITEM_INT']
        self.assertEqual([m.seq for m in items], [2, 0, 0, 3, 1])
        for m in items:
            self.assertEqual(m.mission_type, 1)
            self.assertEqual(m.frame, mavutil.mavlink.MAV_FRAME_GLOBAL_INT)
            self.assertEqual(m.command, 5001)
            self.assertEqual(m.param1, 4)
            self.assertEqual((m.x, m.y), tuple(round(v*1e7) for v in VERTS[m.seq]))
        self.assertTrue(matches(VERTS, self.fc.read_fence(timeout=.3)))
        self.assertTrue(self.fc.clear_fence(timeout=.3))
        self.assertEqual(self.fc.read_fence(timeout=.3), [])
        self.assertFalse(any(self.fc._subs.values()))

    def test_immediate_parameter_echo(self):
        self.assertEqual(self.fc.get_param('FENCE_ALT_MAX', timeout=.1), 15)
        self.assertEqual(self.fc.set_param('FENCE_ALT_MAX', 12, timeout=.1), 12)
        self.assertFalse(any(self.fc._subs.values()))

    def test_legacy_request(self):
        self.peer.legacy = True
        self.fc.upload_fence(VERTS, timeout=.3)
        m = self.peer.items[1]
        self.assertEqual(m.get_type(), 'MISSION_ITEM')
        self.assertEqual(m.frame, mavutil.mavlink.MAV_FRAME_GLOBAL)
        self.assertEqual(m.mission_type, 1)
        self.assertAlmostEqual(m.y, .001)

    def test_rejection_and_timeout(self):
        self.peer.reject = True
        with self.assertRaisesRegex(FCError, 'rejected'):
            self.fc.upload_fence(VERTS, timeout=.1)
        with self.assertRaisesRegex(FCError, 'rejected'):
            self.fc.clear_fence(timeout=.1)
        self.peer.silent = True
        with self.assertRaisesRegex(FCError, 'timed out'):
            self.fc.clear_fence(timeout=.03)
        self.assertFalse(any(self.fc._subs.values()))

    def test_bad_sequence(self):
        self.peer.requests = [65535]
        with self.assertRaisesRegex(FCError, 'sequence'):
            self.fc.upload_fence(VERTS, timeout=.1)

    def test_wrong_sender_ignored(self):
        self.peer.encoder = mavutil.mavlink.MAVLink(None, srcSystem=99, srcComponent=1)
        with self.assertRaisesRegex(FCError, 'timed out'):
            self.fc.clear_fence(timeout=.03)


class ValidationTest(unittest.TestCase):
    def test_zero_coordinates_closure_and_rotation(self):
        self.assertEqual(normalize([{'lat': a, 'lon': b} for a,b in VERTS]), VERTS)
        self.assertEqual(normalize(VERTS + [VERTS[0]]), VERTS)
        self.assertTrue(matches(VERTS, list(reversed(VERTS[1:] + VERTS[:1]))))
        self.assertFalse(matches(VERTS, [(a+.1,b) for a,b in VERTS]))

    def test_invalid(self):
        for bad in (None, 'bad', [[0,0],[1,1]], [[0,0],[1,1],[2,2]],
                    [[0,0],[1,1],[0,1],[1,0]], VERTS + [VERTS[1]],
                    [[0,0],[91,1],[1,0]], [[0,0],[1,float('nan')],[1,0]],
                    [[0,0],{'lat':1},[1,0]]):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                normalize(bad)


class ReadbackToleranceTest(unittest.TestCase):
    """The readback check must survive the VEHICLE's own storage, and still
    reject a different fence.

    MEASURED against a real ArduCopter SITL (ArduPilot 4.8.0, see
    tests/test_sitl_flight.py): uploading lat 15.3647000 / lon 75.1240800 came
    back as 15.3647008 / 75.1240768. That is exactly the float32 rounding of
    153647000 and 751240800, because AC_Fence holds polygon vertices as
    Vector2f - the vehicle cannot store them any better. The old fixed
    tolerance of 1.1e-7 deg (~1.2 cm) is ~30x smaller than that, so it failed a
    fence the FC had stored perfectly; on a real Pixhawk every upload answered
    502 "FC did not store the fence we sent". The bench fake stores exact
    integers, which is why the fake-based tests never showed it. THIS test runs
    without any vehicle, so the regression cannot come back unnoticed.
    """

    #: the exact coordinates the real vehicle returned in that measurement
    SENT = [(15.36470, 75.12400), (15.36470, 75.12408),
            (15.36478, 75.12408), (15.36478, 75.12400)]
    REAL_READBACK = [(15.3647008, 75.124), (15.3647008, 75.1240768),
                     (15.3647808, 75.1240768), (15.3647808, 75.124)]

    def test_the_measured_real_readback_is_accepted(self):
        self.assertTrue(matches(self.SENT, self.REAL_READBACK),
                        "a real vehicle's readback of a correctly stored fence "
                        "must verify")

    def test_tolerance_is_derived_from_float32_storage(self):
        """~2.4e-6 deg lat / 6.4e-6 deg lon at these coordinates."""
        self.assertGreater(quant_tolerance(15.3647), 1.1e-7)
        self.assertGreater(quant_tolerance(75.124), quant_tolerance(15.3647))
        # the bound is small in real terms: under a metre, so a moved fence is
        # still caught (see the rejections below)
        self.assertLess(quant_tolerance(75.124) * 111000, 1.0)
        # and it never goes below the old floor
        self.assertGreaterEqual(quant_tolerance(0.0), 1.1e-7)

    def test_float32_storage_is_the_documented_cause(self):
        """Pin the arithmetic: this IS why the tolerance must be ~1e-6."""
        import struct
        def f32(x):
            return struct.unpack("<f", struct.pack("<f", float(x)))[0]
        self.assertEqual(int(f32(153647000)), 153647008)   # lat, +8 units
        self.assertIn(int(f32(751240800)), (751240768, 751240832))  # lon, +-32

    def test_a_genuinely_different_fence_is_still_rejected(self):
        for label, other in (
                ("shifted 10 m", [(a + 0.00009, b) for a, b in self.SENT]),
                ("shifted 1 deg", [(a + 1.0, b) for a, b in self.SENT]),
                ("one vertex moved 5 m",
                 [self.SENT[0], self.SENT[1],
                  (self.SENT[2][0] + 0.000045, self.SENT[2][1]), self.SENT[3]]),
                ("wrong vertex count", self.SENT[:3])):
            with self.subTest(label=label):
                self.assertFalse(matches(self.SENT, other), label)
                self.assertFalse(matches(other, self.SENT), label)

    def test_winding_and_start_point_still_do_not_matter(self):
        self.assertTrue(matches(self.SENT, list(reversed(self.SENT))))
        self.assertTrue(matches(self.SENT, self.SENT[1:] + self.SENT[:1]))
        self.assertTrue(matches(self.SENT, self.SENT))

    def test_an_explicit_tolerance_still_wins(self):
        """Callers that want the strict old behaviour can still ask for it."""
        self.assertFalse(matches(self.SENT, self.REAL_READBACK,
                                 tolerance=1.1e-7))
        self.assertTrue(matches(self.SENT, self.SENT, tolerance=1e-9))

    def test_the_fake_vehicle_path_is_unaffected(self):
        """The tolerance only ever LOOSENS above the old floor, so the exact
        integers the bench fake returns keep passing unchanged."""
        self.assertTrue(matches(VERTS, VERTS))


if __name__ == '__main__':
    unittest.main()
