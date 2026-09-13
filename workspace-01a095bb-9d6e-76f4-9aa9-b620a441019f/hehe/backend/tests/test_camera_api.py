"""
test_camera_api.py — Integration tests for Camera API endpoints (Phase 5).
"""

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import asyncio
import unittest
from fastapi import HTTPException

import app
from models import CameraModeRequest, CameraFocusRequest


class TestCameraAPI(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        app.init_camera_manager()

    def test_get_camera_status_endpoint(self):
        st = asyncio.run(app.get_camera_status())
        self.assertIn("mode", st)
        self.assertIn("sources", st)
        self.assertIn("aiortc_available", st)

    def test_set_camera_mode_endpoint(self):
        st = asyncio.run(app.set_camera_mode(CameraModeRequest(mode="cam1")))
        self.assertEqual(st["mode"], "cam1")
        self.assertTrue(st["sources"]["cam1"]["active"])

        st = asyncio.run(app.set_camera_mode(CameraModeRequest(mode="dual")))
        self.assertEqual(st["mode"], "dual")
        self.assertTrue(st["sources"]["cam1"]["active"])
        self.assertTrue(st["sources"]["cam2"]["active"])

        with self.assertRaises(Exception):
            asyncio.run(app.set_camera_mode(CameraModeRequest(mode="invalid")))

    def test_set_camera_focus_endpoint(self):
        res = asyncio.run(app.set_camera_focus(CameraFocusRequest(position=0.4)))
        self.assertEqual(res, {"status": "ok", "position": 0.4})

        with self.assertRaises(Exception):
            asyncio.run(app.set_camera_focus(CameraFocusRequest(position=1.5)))

    def test_get_camera_frame_endpoint(self):
        asyncio.run(app.set_camera_mode(CameraModeRequest(mode="cam1")))

        resp = asyncio.run(app.get_camera_frame("cam1"))
        self.assertEqual(resp.media_type, "image/jpeg")
        self.assertGreater(len(resp.body), 100)

        asyncio.run(app.set_camera_mode(CameraModeRequest(mode="none")))
        with self.assertRaises(HTTPException) as cm:
            asyncio.run(app.get_camera_frame("cam1"))
        self.assertEqual(cm.exception.status_code, 404)


if __name__ == "__main__":
    unittest.main()
