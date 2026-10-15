"""Mission steering -> real FCLink -> real MAVLink2 UDP packets.

Loopback only: a packet receiver supplies synthetic heartbeat/position/battery.
No USB discovery, arming, takeoff, physical FC or simulated vehicle dynamics.
Unlike test_detection_response, goto_global/condition_yaw/_send are NOT mocked.
"""
from pathlib import Path
import queue
import socket
import sys
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

PI = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PI))
from fc_link import FCLink, FCError, mavutil
from mission import Mission
from detector import BBox
import mission as mission_module
import geo

M = mavutil.mavlink


class Receiver:
    """Deterministic telemetry/ACK peer; deliberately not an autopilot emulator."""
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 0))
        self.sock.settimeout(.01)
        self.port = self.sock.getsockname()[1]
        self.addr = None
        self.stop = threading.Event()
        self.messages = queue.Queue()
        self.errors = []
        self.heading = 0
        self.mode = 4
        self.ack_result = 0
        self.ack_enabled = True
        self.mav = M.MAVLink(self, srcSystem=17, srcComponent=1)
        self.parser = M.MAVLink(None)
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def write(self, data):
        if self.addr:
            self.sock.sendto(data, self.addr)

    def run(self):
        next_telem = 0
        try:
            while not self.stop.is_set():
                try:
                    data, self.addr = self.sock.recvfrom(65535)
                    for msg in self.parser.parse_buffer(data) or []:
                        self.messages.put(msg)
                        if msg.get_type() == 'COMMAND_LONG' and self.ack_enabled:
                            if msg.command == M.MAV_CMD_DO_SET_MODE and self.ack_result == 0:
                                self.mode = int(msg.param2)
                            self.mav.command_ack_send(msg.command, self.ack_result,
                                0, 0, msg.get_srcSystem(), msg.get_srcComponent())
                except socket.timeout:
                    pass
                if self.addr and time.monotonic() >= next_telem:
                    self.mav.heartbeat_send(M.MAV_TYPE_QUADROTOR, M.MAV_AUTOPILOT_ARDUPILOTMEGA,
                        M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, self.mode, M.MAV_STATE_ACTIVE)
                    self.mav.global_position_int_send(1000,129000000,775000000,
                        915000,15000,0,0,0,int(self.heading*100))
                    self.mav.battery_status_send(0,0,0,2500,[12000]+[65535]*9,
                                                0,0,0,90)
                    next_telem = time.monotonic()+.02
        except Exception as e:
            self.errors.append(e)

    def receive(self, kind, timeout=1):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            msg = self.messages.get(timeout=max(.001,deadline-time.monotonic()))
            if msg.get_type() == kind:
                return msg
        raise AssertionError('No %s packet received' % kind)

    def drain(self):
        while True:
            try: self.messages.get_nowait()
            except queue.Empty: return

    def close(self):
        self.stop.set()
        self.thread.join(2)
        self.sock.close()


class EndIteration(Exception):
    pass


class MovementWireTest(unittest.TestCase):
    def setUp(self):
        self.peer = Receiver()
        self.fc = FCLink()
        self.fc.conn = mavutil.mavlink_connection('udpout:127.0.0.1:%d' % self.peer.port,
            source_system=51, source_component=191)
        self.fc.target_system, self.fc.target_component = 17, 1
        self.fc._reader = threading.Thread(target=self.fc._read_loop,daemon=True)
        self.fc._reader.start()
        self.fc.conn.mav.heartbeat_send(M.MAV_TYPE_ONBOARD_CONTROLLER,
            M.MAV_AUTOPILOT_INVALID,0,0,M.MAV_STATE_ACTIVE)
        hb = self.fc.wait_for(['HEARTBEAT'], lambda m:m.get_srcSystem()==17, 2)
        self.assertIsNotNone(hb)
        self.assertEqual(self.fc.mode,'GUIDED')
        self.assertTrue(self.fc.link_ok())
        self.peer.drain()

    def tearDown(self):
        self.fc._teardown()
        self.peer.close()
        self.assertFalse(self.peer.errors)

    def mission(self, facing='bottom', px=900, py=360):
        cam = SimpleNamespace(name='cam2',facing=facing,size=(1280,720),hfov_deg=66.,rotation_deg=0)
        rig = SimpleNamespace(cams={'cam2':cam},get=lambda name:cam if name=='cam2' else None)
        cfg = {'detector':{'cue_frames':2},'mission':{'approach_stair_m':[12,9,7]},
               'relay':{'announce':False,'on_confirm':False,'store_forward':False}}
        mission = Mission(self.fc,rig,None,SimpleNamespace(push=lambda *a:None),cfg)
        mission.home = (12.9,77.5,0)
        mission.fence = [(12.8998,77.4998),(12.8998,77.5002),
                         (12.9002,77.5002),(12.9002,77.4998)]
        box = BBox(px-40,py-40,80,80,.9,'wire-test')
        mission._on_cue(cam,box)
        mission._on_cue(cam,box)
        return mission

    def track_once(self, mission):
        # Real telemetry waits, real guidance, real wire. Stop only AFTER the
        # first tracking iteration has sent its packets (instead of looping).
        def done(_):
            raise EndIteration()
        with patch.object(mission_module,'time',SimpleNamespace(time=time.time,sleep=done)):
            with self.assertRaises(EndIteration):
                mission._track_and_approach()
        return self.peer.receive('SET_POSITION_TARGET_GLOBAL_INT')

    def assert_position_packet(self, msg):
        self.assertEqual(msg.get_srcSystem(),51)
        self.assertEqual(msg.get_srcComponent(),191)
        self.assertEqual((msg.target_system,msg.target_component),(17,1))
        self.assertEqual(msg.coordinate_frame,6)  # GLOBAL_RELATIVE_ALT_INT
        self.assertEqual(msg.type_mask,3576)       # independently expected 0x0DF8
        self.assertEqual(msg.type_mask & 7,0)      # position fields not ignored
        self.assertEqual(msg.type_mask & 512,0)    # not acceleration-as-force
        self.assertEqual((msg.vx,msg.vy,msg.vz,msg.afx,msg.afy,msg.afz),(0,)*6)

    def test_raw_goto_coordinates_altitude_and_packet_fields(self):
        for lat,lon in [(12.9000123,77.5000456),(-35.3621474,149.1651746)]:
            with self.subTest(lat=lat):
                self.fc.goto_global(lat,lon,12.5)
                msg = self.peer.receive('SET_POSITION_TARGET_GLOBAL_INT')
                self.assert_position_packet(msg)
                self.assertAlmostEqual(msg.lat_int/1e7,lat,delta=1.01e-7)
                self.assertAlmostEqual(msg.lon_int/1e7,lon,delta=1.01e-7)
                self.assertEqual(msg.alt,12.5)

    def test_bottom_forward_back_left_right_at_four_headings(self):
        pixels = {'right':(900,360),'left':(380,360),'forward':(640,100),'back':(640,620)}
        # Expected east/north signs, independent of production projection.
        expected = {0: {'right':(1,0),'left':(-1,0),'forward':(0,1),'back':(0,-1)},
                    90:{'right':(0,-1),'left':(0,1),'forward':(1,0),'back':(-1,0)},
                    180:{'right':(-1,0),'left':(1,0),'forward':(0,-1),'back':(0,1)},
                    270:{'right':(0,1),'left':(0,-1),'forward':(-1,0),'back':(1,0)}}
        for heading in expected:
            self.peer.heading = heading
            self.assertIsNotNone(self.fc.wait_for(['GLOBAL_POSITION_INT'],lambda m:m.hdg==heading*100,1))
            for direction,(px,py) in pixels.items():
                with self.subTest(heading=heading,direction=direction):
                    msg = self.track_once(self.mission(px=px,py=py))
                    self.assert_position_packet(msg)
                    east,north = geo.latlon_to_enu(msg.lat_int/1e7,msg.lon_int/1e7,12.9,77.5)
                    for value,sign in zip((east,north),expected[heading][direction]):
                        if sign: self.assertGreater(value*sign,3)
                        else: self.assertLess(abs(value),.03)
                    self.assertEqual(msg.alt,15.)
                    print('WIRE heading=%3d %-7s -> east=%+.2fm north=%+.2fm alt=%.1fm' %
                          (heading,direction,east,north,msg.alt))

    def test_centered_bottom_sends_relative_altitude_descent(self):
        msg = self.track_once(self.mission(px=640,py=360))
        self.assert_position_packet(msg)
        self.assertAlmostEqual(msg.lat_int/1e7,12.9,delta=1e-7)
        self.assertAlmostEqual(msg.lon_int/1e7,77.5,delta=1e-7)
        self.assertEqual(msg.alt,12.)

    def test_front_left_and_right_send_yaw_and_forward_targets(self):
        for heading,px,turn in [(0,380,-1),(0,900,1),(10,380,-1),(350,900,1)]:
            with self.subTest(heading=heading,px=px):
                self.peer.heading=heading
                self.fc.wait_for(['GLOBAL_POSITION_INT'],lambda m:m.hdg==heading*100,1)
                mission=self.mission('front',px=px)
                # Collect both packets in order; track_once would discard yaw.
                def done(_): raise EndIteration()
                with patch.object(mission_module,'time',SimpleNamespace(time=time.time,sleep=done)):
                    with self.assertRaises(EndIteration): mission._track_and_approach()
                yaw=self.peer.receive('COMMAND_LONG')
                pos=self.peer.receive('SET_POSITION_TARGET_GLOBAL_INT')
                self.assert_position_packet(pos)
                self.assertEqual(yaw.command,M.MAV_CMD_CONDITION_YAW)
                self.assertEqual((yaw.target_system,yaw.target_component),(17,1))
                self.assertEqual(yaw.param2,25.)
                self.assertEqual(yaw.param3,turn)   # left must not force clockwise
                self.assertEqual(yaw.param4,0.)    # absolute heading
                self.assertGreater(((yaw.param1-heading+180)%360-180)*turn,0)
                east,north=geo.latlon_to_enu(pos.lat_int/1e7,pos.lon_int/1e7,12.9,77.5)
                self.assertAlmostEqual((east*east+north*north)**.5,3.,delta=.03)
                self.assertEqual(pos.alt,15.)
                print('WIRE front hdg=%d: yaw %.2fdeg direction=%+d + 3m target' %
                      (heading,yaw.param1,yaw.param3))

    def test_relative_yaw_sweep_packet(self):
        self.fc.condition_yaw(30,speed_deg_s=20,relative=True)
        msg=self.peer.receive('COMMAND_LONG')
        self.assertEqual(msg.command,M.MAV_CMD_CONDITION_YAW)
        self.assertEqual((msg.param1,msg.param2,msg.param3,msg.param4),(30,20,1,1))

    def test_guided_command_ack_over_real_receive_loop(self):
        self.assertEqual(self.fc.set_mode('GUIDED',timeout=1),0)
        msg=self.peer.receive('COMMAND_LONG')
        self.assertEqual(msg.command,M.MAV_CMD_DO_SET_MODE)
        self.assertEqual((msg.param1,msg.param2),(1,4))
        self.peer.ack_result=M.MAV_RESULT_DENIED
        self.assertEqual(self.fc._cmd_long(M.MAV_CMD_DO_SET_MODE,1,4,ack_timeout=.3),M.MAV_RESULT_DENIED)

    def test_command_ack_cannot_race_subscription(self):
        self.peer.ack_enabled=False
        original_send=self.fc._send
        def send_and_reply(*args,**kwargs):
            original_send(*args,**kwargs)
            # Worst-case: parsed ACK is delivered before _send returns.
            msg=M.MAVLink_command_ack_message(M.MAV_CMD_DO_SET_MODE,0,0,0,51,191)
            msg=M.MAVLink(None).parse_char(msg.pack(M.MAVLink(None,srcSystem=17,srcComponent=1)))
            for q in self.fc._subs.get('COMMAND_ACK',[]): q.put_nowait(msg)
        with patch.object(self.fc,'_send',side_effect=send_and_reply):
            self.assertEqual(self.fc._cmd_long(M.MAV_CMD_DO_SET_MODE,1,4,ack_timeout=.05),0)
        self.assertFalse(any(self.fc._subs.values()))

    def test_foreign_ack_does_not_confirm_our_command(self):
        self.peer.ack_enabled=False
        original_send=self.fc._send
        def send_and_wrong_replies(*args,**kwargs):
            original_send(*args,**kwargs)
            for source,recipient,command in [(99,51,M.MAV_CMD_DO_SET_MODE),
                                             (17,255,M.MAV_CMD_DO_SET_MODE),
                                             (17,51,M.MAV_CMD_CONDITION_YAW)]:
                msg=M.MAVLink_command_ack_message(command,0,0,0,recipient,191)
                msg=M.MAVLink(None).parse_char(msg.pack(M.MAVLink(None,srcSystem=source,srcComponent=1)))
                for q in self.fc._subs.get('COMMAND_ACK',[]): q.put_nowait(msg)
        with patch.object(self.fc,'_send',side_effect=send_and_wrong_replies):
            with self.assertRaises(FCError):
                self.fc._cmd_long(M.MAV_CMD_DO_SET_MODE,1,4,ack_timeout=.05)
        self.assertFalse(any(self.fc._subs.values()))

    def test_in_progress_ack_waits_for_final_result(self):
        self.peer.ack_enabled=False
        original_send=self.fc._send
        def send_and_replies(*args,**kwargs):
            original_send(*args,**kwargs)
            for result in [M.MAV_RESULT_IN_PROGRESS,M.MAV_RESULT_DENIED]:
                msg=M.MAVLink_command_ack_message(M.MAV_CMD_DO_SET_MODE,result,0,0,51,191)
                msg=M.MAVLink(None).parse_char(msg.pack(M.MAVLink(None,srcSystem=17,srcComponent=1)))
                for q in self.fc._subs.get('COMMAND_ACK',[]): q.put_nowait(msg)
        with patch.object(self.fc,'_send',side_effect=send_and_replies):
            self.assertEqual(self.fc._cmd_long(M.MAV_CMD_DO_SET_MODE,1,4,ack_timeout=.05),M.MAV_RESULT_DENIED)

    def test_command_ack_timeout_is_an_error(self):
        self.peer.ack_enabled=False
        with self.assertRaises(FCError):
            self.fc._cmd_long(M.MAV_CMD_DO_SET_MODE,1,4,ack_timeout=.05)


if __name__ == '__main__':
    unittest.main(verbosity=2)
