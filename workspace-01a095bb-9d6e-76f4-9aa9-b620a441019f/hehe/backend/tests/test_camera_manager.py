"""
test_camera_manager.py — Unit tests for CameraManager component (Phase 5).
"""

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import unittest
import numpy as np

from camera_source import SyntheticQRSource
from camera_manager import CameraManager


class TestCameraManager(unittest.TestCase):
    def test_camera_manager_initialization(self):
        cam1 = SyntheticQRSource(payload="CAM1", width=320, height=240, fps=10)
        cam2 = SyntheticQRSource(payload="CAM2", width=320, height=240, fps=10)
        mgr = CameraManager({"cam1": cam1, "cam2": cam2})

        st = mgr.status()
        self.assertEqual(st["mode"], "none")
        self.assertIn("cam1", st["sources"])
        self.assertIn("cam2", st["sources"])
        self.assertFalse(st["sources"]["cam1"]["active"])
        self.assertFalse(st["sources"]["cam2"]["active"])

    def test_camera_manager_mode_transitions(self):
        cam1 = SyntheticQRSource(payload="CAM1", width=320, height=240, fps=10)
        cam2 = SyntheticQRSource(payload="CAM2", width=320, height=240, fps=10)
        mgr = CameraManager({"cam1": cam1, "cam2": cam2})

        # Test mode: cam1
        mgr.set_active("cam1")
        st = mgr.status()
        self.assertEqual(st["mode"], "cam1")
        self.assertTrue(st["sources"]["cam1"]["active"])
        self.assertFalse(st["sources"]["cam2"]["active"])

        # Test mode: cam2
        mgr.set_active("cam2")
        st = mgr.status()
        self.assertEqual(st["mode"], "cam2")
        self.assertFalse(st["sources"]["cam1"]["active"])
        self.assertTrue(st["sources"]["cam2"]["active"])

        # Test mode: dual
        mgr.set_active("dual")
        st = mgr.status()
        self.assertEqual(st["mode"], "dual")
        self.assertTrue(st["sources"]["cam1"]["active"])
        self.assertTrue(st["sources"]["cam2"]["active"])

        # Test mode: none
        mgr.set_active("none")
        st = mgr.status()
        self.assertEqual(st["mode"], "none")
        self.assertFalse(st["sources"]["cam1"]["active"])
        self.assertFalse(st["sources"]["cam2"]["active"])

    def test_camera_manager_invalid_mode(self):
        cam1 = SyntheticQRSource(payload="CAM1")
        mgr = CameraManager({"cam1": cam1})
        with self.assertRaises(ValueError):
            mgr.set_active("invalid_mode")

    def test_camera_manager_get_frame(self):
        cam1 = SyntheticQRSource(payload="CAM1", width=320, height=240, fps=10)
        mgr = CameraManager({"cam1": cam1})
        mgr.set_active("cam1")

        frame = mgr.get_frame("cam1")
        self.assertIsInstance(frame, np.ndarray)
        self.assertEqual(frame.shape, (240, 320, 3))
        self.assertEqual(frame.dtype, np.uint8)

        mgr.set_active("none")
        self.assertIsNone(mgr.get_frame("cam1"))

    def test_camera_manager_set_focus(self):
        cam1 = SyntheticQRSource(payload="CAM1")
        mgr = CameraManager({"cam1": cam1})
        mgr.set_active("cam1")

        mgr.set_focus(0.75)

        with self.assertRaises(ValueError):
            mgr.set_focus(1.5)


if __name__ == "__main__":
    unittest.main()
