"""Resolution contracts: network downscale, native decode input, live tile options."""
from pathlib import Path
import sys
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from detector import BBox, _letterbox, detect_tiles
from mission import Mission


class ResolutionTest(unittest.TestCase):
    def test_cam3_network_scale_and_native_tiles(self):
        frame = np.zeros((2592,4608,3), np.uint8)
        net, scale, padding = _letterbox(frame,640)
        self.assertEqual(net.shape, (640,640,3))
        self.assertAlmostEqual(scale, 640/4608)
        shapes = []
        def detect(tile):
            shapes.append(tile.shape)
            self.assertTrue(np.shares_memory(frame,tile))
            return []
        detect_tiles(SimpleNamespace(detect=detect),frame,3,3,.25)
        self.assertEqual(len(shapes),9)
        self.assertEqual(min(s[1] for s in shapes),1920)
        self.assertEqual(max(s[1] for s in shapes),2304)
        self.assertEqual(frame.shape,(2592,4608,3))

    def test_worker_uses_original_frame_and_configured_tiling(self):
        for facing, front, detname, want_tiled in [
            ('front',False,'yolo_onnx',False),('front',True,'yolo_onnx',True),
            ('bottom',False,'yolo_onnx',True),('bottom',True,'classical',False)]:
            with self.subTest(facing=facing,front=front,detector=detname):
                frame = np.zeros((480,640,3),np.uint8)
                cam = SimpleNamespace(name='cam1',facing=facing,
                    latest=lambda:(frame,time.time(),1),set_overlay=lambda *a:None)
                cfg = {'detector':{'tile_front':front,'tile_bottom':True,'tile_overlap':.1},
                       'qr_filter':{'require_decode_for_cue':True}}
                det = SimpleNamespace(name=detname)
                m = Mission(SimpleNamespace(),SimpleNamespace(cams={}),det,
                            SimpleNamespace(push=lambda *a:None),cfg)
                def once(*args,**kwargs):
                    m._stop.set()
                    return [BBox(50,50,80,80,.9,'test')]
                det.detect = once
                with patch('detector.detect_tiles',side_effect=once) as tiles, \
                     patch('decoder.decode_frame',return_value=None) as decode:
                    m._worker(cam)
                self.assertEqual(tiles.called,want_tiled)
                if want_tiled:
                    self.assertEqual(tiles.call_args.kwargs['overlap'],.1)
                self.assertIs(decode.call_args.args[0],frame)
                self.assertEqual(decode.call_args.args[1],(50,50,80,80))


if __name__ == '__main__':
    unittest.main()
