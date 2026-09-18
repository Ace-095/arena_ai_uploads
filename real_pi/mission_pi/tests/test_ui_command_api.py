"""Live HTTP command + continuous MJPEG regression, with a simulated FC.
Optional actual DOM/app.js click test: npm install in real_pi/mission-ui.
Run from any directory with the mission_pi venv; no hardware is commanded.
"""
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace

PI = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PI))
import cv2
import numpy as np
import httpx
import uvicorn
import yaml
from cameras import CameraRig, FileCamera
from detector import get_detector
from mission import Mission
from server import create_app
from streamm1 import StreamManager

VERTS = [[0,0], [0,.001], [.001,.001], [.001,0]]


class SimFC:
    armed = False
    connected = True
    fail = None
    delay = 0
    def __init__(self):
        self.vertices = []
        self.calls = []
        self.entered = threading.Event()
    def link_ok(self):
        return self.connected
    def get_position(self, **kw):
        return dict(lat=0, lon=0, alt_rel=15)
    def upload_fence(self, vertices, **kw):
        self.calls.append(('upload', vertices))
        self.entered.set()
        time.sleep(self.delay)
        if self.fail == 'upload':
            raise RuntimeError('simulated FC rejection')
        self.vertices = list(vertices)
        return len(vertices)
    def read_fence(self, **kw):
        if self.fail == 'readback':
            return [(a+.1,b) for a,b in self.vertices]
        return self.vertices
    def clear_fence(self, **kw):
        self.calls.append(('clear',))
        if self.fail == 'clear':
            return False
        self.vertices = []
        return True
    def set_param(self, name, value, **kw):
        self.calls.append(('param', name, value))
        if self.fail == 'param':
            raise RuntimeError('no parameter echo')
        return value


class CommandsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cv2.imwrite(cls.tmp.name + '/frame.png', np.full((480,640,3), 150, np.uint8))
        cfg = yaml.safe_load((PI/'config.yaml').read_text())
        cls.fc = SimFC()
        cls.rig = CameraRig(cfg)
        cls.rig.cams = {n: FileCamera(n, cls.tmp.name, (640,480), 66, facing, fps=12)
                        for n,facing in [('cam1','front'),('cam2','bottom')]}
        cls.mission = Mission(cls.fc, cls.rig, get_detector({'kind':'classical'}),
                              SimpleNamespace(push=lambda *a: None), cfg)
        cls.streams = StreamManager(cls.rig, dict(fps=12, width=640, quality=70))
        cls.rig.start_all()
        cls.streams.sync(cls.rig)
        cls.streams.start_all()
        app = create_app(cls.rig, cls.mission, cls.fc, cls.streams)
        # Test-only fault injection, never installed in production create_app.
        from fastapi import Request
        @app.post('/__test/fault')
        async def fault(req: Request):
            cls.fc.fail = (await req.json()).get('fail')
            return {'ok': True}
        sock = socket.socket()
        sock.bind(('127.0.0.1', 0))
        cls.port = sock.getsockname()[1]
        cls.base = 'http://127.0.0.1:%d' % cls.port
        cls.srv = uvicorn.Server(uvicorn.Config(app, log_level='error'))
        cls.thread = threading.Thread(target=lambda: cls.srv.run(sockets=[sock]), daemon=True)
        cls.thread.start()
        deadline = time.monotonic()+10
        while not cls.srv.started and time.monotonic() < deadline:
            time.sleep(.01)
        assert cls.srv.started

    @classmethod
    def tearDownClass(cls):
        cls.srv.should_exit = True
        cls.thread.join(5)
        cls.streams.stop_all()
        cls.rig.stop_all()
        cls.tmp.cleanup()

    def setUp(self):
        self.fc.connected, self.fc.armed = True, False
        self.fc.fail, self.fc.delay = None, 0
        self.fc.entered.clear()
        self.client = httpx.Client(base_url=self.base, timeout=6)
        self.addCleanup(self.client.close)

    def upload(self):
        return self.client.post('/api/fence', json={'vertices': VERTS})

    def test_fence_roundtrip(self):
        r = self.upload()
        self.assertEqual(r.status_code, 200, r.text)
        self.assertTrue(r.json()['confirmed'])
        self.assertEqual(self.mission.fence, [tuple(v) for v in VERTS])
        st = self.client.get('/api/fence').json()
        self.assertEqual(st['vertex_count'], 4)
        self.assertEqual(st['vertices_latlon'][0], {'lat':0, 'lon':0})
        r = self.client.delete('/api/fence')
        self.assertTrue(r.json()['confirmed'])
        self.assertTrue(r.json()['cleared'])
        self.assertFalse(r.json()['loaded'])
        self.assertEqual(self.mission.fence, [])

    def test_failures_never_claim_confirmed(self):
        self.upload()
        before = list(self.mission.fence)
        for fault, method in [('upload','POST'), ('readback','POST'), ('clear','DELETE')]:
            self.fc.fail = fault
            r = (self.upload() if method == 'POST' else self.client.delete('/api/fence'))
            self.assertEqual(r.status_code, 502, r.text)
            self.assertFalse(r.json()['confirmed'])
            self.assertEqual(self.mission.fence, before)
        self.fc.connected = False
        self.assertEqual(self.upload().status_code, 503)
        self.assertEqual(self.client.delete('/api/fence').status_code, 503)
        self.fc.connected, self.fc.armed = True, True
        self.assertEqual(self.upload().status_code, 409)
        self.assertEqual(self.client.delete('/api/fence').status_code, 409)

    def test_invalid_input_is_atomic(self):
        count = len(self.fc.calls)
        for verts in ([], [[0,0],[1,1],[2,2]], [[0,0],[1,1],[0,1],[1,0]],
                      [[0,0], {'lat':1}, [1,0]], [[0,0],[91,0],[1,1]]):
            self.assertEqual(self.client.post('/api/fence', json={'vertices':verts}).status_code, 400)
        self.assertEqual(len(self.fc.calls), count)

    def test_altitude_and_fc_echo(self):
        r = self.client.post('/api/config', json={'max_alt_m':10, 'sweep_alt_m':15})
        self.assertTrue(r.json()['fc_updated'])
        self.assertEqual(self.mission.sweep_alt, 10)
        self.assertEqual(self.client.get('/api/config').json()['max_alt_m'], 10)
        self.client.post('/api/config', json={'max_alt_m':8})
        self.assertEqual(self.mission.sweep_alt, 8)  # lowering ceiling clamps existing sweep
        self.fc.fail = 'param'
        r = self.client.post('/api/config', json={'max_alt_m':9})
        self.assertFalse(r.json()['fc_updated'])
        for invalid in (0, 51, 'NaN', 'Infinity'):
            self.assertEqual(self.client.post('/api/config', json={'max_alt_m':invalid}).status_code, 400)
        self.assertEqual(self.mission.max_alt_m, 9)

    def test_camera_controls_presets_and_reset(self):
        self.assertEqual(self.client.post('/api/camera/controls', json={'cam':'cam3'}).status_code, 404)
        r = self.client.post('/api/camera/controls', json={'cam':'cam1','exposure_us':7000,'contrast':1.5})
        self.assertEqual(r.json()['controls']['exposure_us'], 7000)
        r = self.client.post('/api/camera/status', json={'cam':'cam1'})
        self.assertEqual(r.json()['controls']['contrast'], 1.5)
        for preset, mode in [('dark','qr_boost_lowlight'),('bench','normal')]:
            self.client.post('/api/qr/preset', json={'preset':preset}).raise_for_status()
            c = self.rig.get('cam2')
            self.assertEqual(c.qr_boost['qr_boost_mode'], mode)
            self.assertEqual(c.qr_boost['qr_boost_enabled'], preset != 'bench')
        self.assertEqual(self.client.post('/api/qr/preset', json={'preset':'invalid'}).status_code, 404)

    def test_slow_fence_does_not_block_continuous_stream(self):
        self.fc.delay = 1.2
        result = []
        def upload():
            with httpx.Client(base_url=self.base, timeout=6) as c:
                result.append(c.post('/api/fence', json={'vertices':VERTS}))
        with self.client.stream('GET', '/api/camera/stream/cam2') as r:
            self.assertEqual(r.status_code, 200)
            # httpx removes HTTP chunk encoding. Retain split SOI markers.
            t0, times, tail = time.monotonic(), [], b''
            thread = None
            for chunk in r.iter_raw():
                data = tail + chunk
                times.extend([time.monotonic()] * data.count(b'\xff\xd8\xff'))
                tail = data[-2:]
                if len(times) >= 2 and thread is None:
                    thread = threading.Thread(target=upload)
                    thread.start()
                    self.assertTrue(self.fc.entered.wait(1))
                    busy = self.client.delete('/api/fence')
                    self.assertEqual(busy.status_code, 409)
                if time.monotonic()-t0 > 2.5:
                    break
            thread.join(5)
        self.assertEqual(result[0].status_code, 200)
        gap = max(b-a for a,b in zip(times,times[1:]))
        print('continuous MJPEG during 1.2s FC upload: %d frames, max gap %.3fs' % (len(times),gap))
        self.assertGreater(len(times), 15)
        self.assertLess(gap, .65)

    def test_ui_dom_clicks_to_live_pi(self):
        script = PI.parent/'mission-ui/tests/test_commands_dom.cjs'
        probe = subprocess.run(['node','-e', "require('jsdom')"], capture_output=True,
                               cwd=script.parent.parent)
        if probe.returncode:
            self.skipTest('optional UI DOM test: npm install in real_pi/mission-ui')
        r = subprocess.run(['node',str(script),self.base], capture_output=True, text=True,
                           timeout=40, cwd=script.parent.parent)
        print(r.stdout)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)


if __name__ == '__main__':
    unittest.main()
