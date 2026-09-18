"""MAVLink2 wire regression: production FCLink + deterministic simulated FC.
No hardware. Replies arrive synchronously inside send (worst ACK race).
"""
import os
import sys
import unittest
from types import SimpleNamespace
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from fc_link import FCLink, FCError, mavutil
from fence import normalize, matches

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


if __name__ == '__main__':
    unittest.main()
