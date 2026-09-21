"""Real YOLO output -> real mission decision/steering, simulated FC only.

The bundled ONNX model detects a synthetic QR once. Its actual output is replayed
through Mission._worker with controlled decode outcomes. Virtual time and a
recording FC exercise search/tracking without sleeping or connecting to a vehicle.
Run: python tests/test_detection_response.py (vision dependencies required).
"""
from pathlib import Path
import copy
import queue
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch, Mock

PI = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PI))
import cv2
import numpy as np
import qrcode
import yaml
import geo
import mission as mission_module
from mission import Mission
from detector import BBox
from detectors.yolo_onnx import YoloOnnxDetector


class StopAtCommand(Exception):
    """Test harness stops at the first TRACK/APPROACH movement command."""


class Clock:
    def __init__(self):
        self.now = 1000.0
        self.on_sleep = None
    def time(self):
        return self.now
    def sleep(self, seconds):
        self.now += seconds
        if self.now > 1030:
            raise AssertionError('test failed to reach a decision in 30 virtual seconds')
        if self.on_sleep:
            self.on_sleep()


class RecordingFC:
    mode = 'GUIDED'
    def __init__(self):
        self.commands = []
        self.pos = dict(lat=12.9, lon=77.5, alt_rel=15., hdg=0.)
        self.mission = None
        self.battery = 90
        self.arrive = False
    def get_position(self, **kw):
        return dict(self.pos)
    def get_battery(self):
        return {'pct': self.battery}
    def goto_global(self, lat, lon, alt):
        self.commands.append(('goto', self.mission.phase, lat, lon, alt))
        if self.mission.phase in ('TRACK', 'APPROACH'):
            raise StopAtCommand()
        if self.arrive:
            self.pos.update(lat=lat, lon=lon, alt_rel=alt)
    def condition_yaw(self, yaw, **kw):
        self.commands.append(('yaw', self.mission.phase, yaw, kw))
    def set_mode(self, mode, **kw):
        self.commands.append(('mode', mode))
        self.mode = mode
    def link_ok(self):
        return True
    def mode_age(self):
        return 0.
    def start_streams(self):
        pass
    def get_home(self, **kw):
        return (self.pos['lat'], self.pos['lon'], 0.)
    def read_fence(self, **kw):
        return list(self.mission.fence)
    def read_plan(self, **kw):
        return []
    def find_sprayer_seqs(self, *a):
        return []


class DetectionResponseTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = yaml.safe_load((PI/'config.yaml').read_text())
        cls.frame = np.full((720,1280,3), (70,100,70), np.uint8)
        qr = np.array(qrcode.make('STEER-TEST-001', box_size=8, border=4).convert('RGB'))
        cls.frame[280:440,820:980] = cv2.resize(qr,(160,160),interpolation=cv2.INTER_NEAREST)
        model = YoloOnnxDetector(str(PI/'models/qr_yolov8n.onnx'),conf_thr=.35)
        boxes = model.detect(cls.frame)
        cls.box = next((b for b in boxes if 800 < b.x+b.w/2 < 1000),None)
        assert cls.box is not None, 'bundled model failed to detect synthetic QR'
        print('Actual YOLO box: x=%d y=%d w=%d h=%d confidence=%.3f' %
              (cls.box.x,cls.box.y,cls.box.w,cls.box.h,cls.box.conf))

    def setUp(self):
        self.clock = Clock()
        self.time_patch = patch.object(mission_module,'time',self.clock)
        self.time_patch.start()
        self.addCleanup(self.time_patch.stop)
        cfg = copy.deepcopy(self.cfg)
        cfg['detector'].update(tile_bottom=False,tile_front=False,cue_frames=2)
        cfg['mission'].update(coverage_mode='legacy',approach_timeout_s=10,search_timeout_s=30)
        cfg['relay'].update(announce=False,on_confirm=False,store_forward=False)
        self.fc = RecordingFC()
        self.events = []
        self.cam = SimpleNamespace(name='cam2',facing='bottom',size=(1280,720),
            hfov_deg=66.,rotation_deg=0,latest=lambda:(self.frame,self.clock.time(),1),
            set_overlay=lambda *a:None)
        cams = {'cam2':self.cam}
        self.rig = SimpleNamespace(cams=cams,get=lambda name:cams.get(name))
        self.detector = SimpleNamespace(name='yolo_onnx')
        self.m = Mission(self.fc,self.rig,self.detector,
                         SimpleNamespace(push=lambda c,d:self.events.append((c,d))),cfg)
        self.fc.mission = self.m
        self.m.home = (12.9,77.5,0.)
        self.m.fence = [(12.8998,77.4998),(12.8998,77.5002),
                        (12.9002,77.5002),(12.9002,77.4998)]
        self.rows = [(12.9001,77.5),(12.90015,77.5)]
        self.m._blob_observe = Mock()
        self.m._cover_tick = Mock()

    def feed(self, boxes=None, decode=None, real_decode=False, cam=None):
        """One captured frame; boxes are actual model output unless overridden."""
        boxes = [self.box] if boxes is None else boxes
        def detect(frame):
            self.assertIs(frame,self.frame)
            self.m._stop.set()  # finish after this single worker iteration
            return boxes
        self.detector.detect = detect
        self.m._stop.clear()
        try:
            if real_decode:
                self.m._worker(cam or self.cam)
            else:
                with patch('decoder.decode_frame',return_value=decode):
                    self.m._worker(cam or self.cam)
        finally:
            self.m._stop.clear()

    def sweep(self):
        with patch('geo.lawnmower_rows',return_value=self.rows):
            return self.m._sweep_grid()

    def test_one_box_must_not_steer_before_configured_threshold(self):
        self.feed()
        self.assertIsNone(self.m._fresh_cue())
        self.fc.arrive = True
        self.sweep()
        self.assertEqual([c[1] for c in self.fc.commands if c[0]=='goto'],['SWEEP_GRID','SWEEP_GRID'])

    def test_bottom_detection_interrupts_current_leg(self):
        injected = False
        def while_flying():
            nonlocal injected
            if not injected:
                injected = True
                self.feed(); self.feed()
        self.clock.on_sleep = while_flying
        with self.assertRaises(StopAtCommand):
            self.sweep()
        gotos = [c for c in self.fc.commands if c[0]=='goto']
        self.assertEqual([c[1] for c in gotos],['SWEEP_GRID','TRACK'])
        east,north = geo.latlon_to_enu(gotos[-1][2],gotos[-1][3],12.9,77.5)
        self.assertGreater(east,1.5)  # QR lies right of center, heading north
        self.assertLess(abs(north),.5)
        self.assertEqual(gotos[-1][4],15.)  # off-center: translate first, not descend
        self.assertNotEqual((gotos[-1][2],gotos[-1][3]),self.rows[0])
        print('Grid interrupted: first waypoint -> TRACK east %.2fm, north %.2fm, altitude %.1fm' %
              (east,north,gotos[-1][4]))

    def test_qualified_detection_interrupts_yaw_sweep(self):
        self.feed(); self.feed()
        with self.assertRaises(StopAtCommand):
            self.m._sweep_yaw()
        self.assertFalse(any(c[0]=='yaw' for c in self.fc.commands))
        self.assertEqual(self.fc.commands[-1][1],'TRACK')

    def test_permissive_presets_accept_undecoded_candidates(self):
        for name in ['day','far','aggressive','bench']:
            with self.subTest(preset=name):
                self.m.set_preset(name)
                self.feed(); self.feed()
                self.assertIsNotNone(self.m._fresh_cue())

    def test_configured_cue_count_controls_steering(self):
        self.m.cue_frames=3
        self.feed(); self.feed()
        self.assertIsNone(self.m._fresh_cue())
        self.feed()
        self.assertIsNotNone(self.m._fresh_cue())
        self.m.set_preset('day')
        self.m.cue_frames=1
        self.feed()
        self.assertIsNotNone(self.m._fresh_cue())

    def test_front_detection_yaws_and_steps_forward(self):
        self.cam.facing = 'front'
        self.feed(); self.feed()
        with self.assertRaises(StopAtCommand):
            self.m._track_and_approach()
        yaw = next(c for c in self.fc.commands if c[0]=='yaw')
        goto = next(c for c in self.fc.commands if c[0]=='goto')
        self.assertGreater(yaw[2],0)
        self.assertFalse(yaw[3]['relative'])
        self.assertAlmostEqual(geo.haversine_m(12.9,77.5,goto[2],goto[3]),3.,delta=.03)
        self.assertEqual(goto[4],15.)

    def test_centered_bottom_detection_starts_descent(self):
        b = BBox(600,320,80,80,.9,'test')
        self.feed([b]); self.feed([b])
        with self.assertRaises(StopAtCommand):
            self.m._track_and_approach()
        goto = self.fc.commands[-1]
        self.assertEqual(goto[1],'APPROACH')
        self.assertEqual(goto[4],self.m.approach_stair[0])
        self.assertLess(goto[4],15.)

    def test_strict_presets_ignore_undecoded_yolo_box(self):
        for name in ['dark','kabaddi','low']:
            with self.subTest(preset=name):
                self.m.set_preset(name)
                self.feed(); self.feed(); self.feed()
                self.assertIsNone(self.m._fresh_cue())
                self.fc.arrive = True
                self.fc.commands.clear()
                self.sweep()
                self.assertTrue(all(c[1]=='SWEEP_GRID' for c in self.fc.commands if c[0]=='goto'))

    def test_strict_decoded_box_can_steer_before_consensus(self):
        self.m.set_preset('dark')
        self.feed(real_decode=True); self.feed(real_decode=True)
        self.assertIsNone(self.m.payload)  # configured confirmation streak is 3
        self.assertIsNotNone(self.m._fresh_cue())
        with self.assertRaises(StopAtCommand):
            self.sweep()

    def test_confirmed_payload_finishes_instead_of_chasing_box(self):
        self.m.set_preset('dark')
        for _ in range(3):
            self.feed(real_decode=True)
        self.assertEqual(self.m.payload,'STEER-TEST-001')
        with patch.object(self.m,'_transmit_and_finish',return_value=False) as finish:
            self.sweep()
        finish.assert_called_once()
        self.assertFalse(self.fc.commands)

    def test_filter_rejected_box_never_steers(self):
        bad = BBox(self.box.x,self.box.y,self.box.w,self.box.h,.16,'yolo')
        self.feed([bad]); self.feed([bad])
        self.assertIsNone(self.m._fresh_cue())

    def test_miss_and_stale_gap_reset_pending_streak(self):
        self.feed(); self.feed([]); self.feed()
        self.assertIsNone(self.m._fresh_cue())
        self.clock.now += 3
        self.feed()
        self.assertIsNone(self.m._fresh_cue())
        self.feed()
        self.assertIsNotNone(self.m._fresh_cue())

    def test_cameras_do_not_share_confirmation_streak(self):
        front = copy.copy(self.cam)
        front.name,front.facing = 'cam1','front'
        self.rig.cams['cam1'] = front
        self.feed(cam=front); self.feed(cam=self.cam)
        self.assertIsNone(self.m._fresh_cue())
        self.feed(cam=front)
        self.assertEqual(self.m._fresh_cue()[0],'cam1')

    def test_preset_change_discards_old_undecoded_cue(self):
        self.feed(); self.feed()
        self.assertIsNotNone(self.m._fresh_cue())
        self.m.set_preset('dark')
        self.assertIsNone(self.m._fresh_cue())

    def test_inflight_old_preset_result_cannot_restore_cue(self):
        old_epoch = self.m._cue_epoch
        self.m.set_preset('dark')
        self.m._on_cue(self.cam,self.box,old_epoch)
        self.m._on_cue(self.cam,self.box,old_epoch)
        self.assertIsNone(self.m._fresh_cue())

    def test_strict_undecoded_frame_breaks_pending_streak(self):
        self.m.set_preset('dark')
        self.feed(real_decode=True)
        self.feed(decode=None)
        self.feed(real_decode=True)
        self.assertIsNone(self.m._fresh_cue())

    def test_removed_camera_cue_is_not_used(self):
        self.feed(); self.feed()
        self.rig.cams.clear()
        self.assertIsNone(self.m._fresh_cue())

    def test_already_relayed_payload_holds_then_finishes(self):
        self.m.payload='STEER-TEST-001'
        self.m._confirmed_relayed=True
        self.m.cfg['relay']['window_s']=.2
        with patch('qr_relay.relay_qr') as relay:
            self.assertFalse(self.m._transmit_and_finish())
        relay.assert_not_called()  # no duplicate relay
        self.assertTrue(any(c[0]=='goto' and c[1]=='TRANSMIT' for c in self.fc.commands))
        self.assertEqual(self.fc.commands[-1],('mode','RTL'))
        self.assertEqual(self.m.phase,'DONE')

    def test_lost_cue_returns_to_search(self):
        self.feed(); self.feed()
        self.clock.now += 3
        with patch.object(self.m,'_sweep_grid',return_value='resume') as resume:
            self.assertEqual(self.m._track_and_approach(),'resume')
        resume.assert_called_once()
        self.assertFalse(self.fc.commands)

    def test_target_is_clamped_inside_fence(self):
        self.m.fence = [(12.89999,77.49999),(12.89999,77.50001),
                        (12.90001,77.50001),(12.90001,77.49999)]
        self.feed(); self.feed()
        with self.assertRaises(StopAtCommand):
            self.m._track_and_approach()
        goto = self.fc.commands[-1]
        self.assertTrue(geo.point_in_polygon(goto[2],goto[3],self.m.fence))

    def test_abort_and_low_battery_do_not_chase(self):
        for reason in ['abort','battery']:
            self.feed(); self.feed()
            if reason=='abort': self.m.request_abort()
            else: self.fc.battery=5; self.m._abort.clear()
            self.m._track_and_approach()
        self.assertFalse(any(c[0]=='goto' for c in self.fc.commands))
        self.assertIn(('mode','RTL'),self.fc.commands)

    def test_sustained_external_mode_does_not_get_new_targets(self):
        self.feed(); self.feed()
        self.fc.mode='RTL'
        self.m._offmode_n=2  # already observed the first two stale reads
        self.m._track_and_approach()
        self.assertFalse(any(c[0]=='goto' for c in self.fc.commands))

    def test_detection_alone_does_not_bypass_mission_trigger(self):
        self.feed(); self.feed()
        self.fc.mode='AUTO'
        self.fc.hb_sources=lambda:{}
        def empty(**kw):
            self.clock.now += 1
            raise queue.Empty()
        self.fc.subscribe=lambda *a:SimpleNamespace(get=empty)
        self.fc.unsubscribe=Mock()
        self.m.trigger_timeout_s=2
        # Run real startup and the actual trigger loop, not a fake gate.
        # Only background worker startup/speed probing are stubbed.
        with patch.object(mission_module.threading,'Thread'), \
             patch.object(self.m,'_snapshot_speed'):
            self.m._run()
        self.assertEqual(self.m.phase,'FAILSAFE')
        self.assertEqual(self.m.detail,'trigger timeout')
        self.assertFalse(self.fc.commands)
        self.fc.unsubscribe.assert_called_once()


if __name__ == '__main__':
    unittest.main(verbosity=2)
