#!/usr/bin/env python3
"""Repeatable synthetic resolution probe; NOT a field-range guarantee.
Runs production classical, YOLO full-frame and tiled detection, then decodes
boxes on the ORIGINAL frame, exactly as the mission does. No FC connection.
Example: python tools/qr_resolution_probe.py --width 4608 --height 2592
Requires OpenCV, numpy, qrcode, onnxruntime. Times describe this host only.
"""
import argparse
import json
from pathlib import Path
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import cv2
import numpy as np
import qrcode
from detector import ClassicalDetector, detect_tiles
from detectors.yolo_onnx import YoloOnnxDetector
from decoder import decode_frame
from qr_filter import QrFilter


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--width', type=int, default=4608)
    p.add_argument('--height', type=int, default=2592)
    p.add_argument('--sizes', type=int, nargs='+', default=[48,96,144])
    args = p.parse_args()
    payload = 'QR-TEST-001'
    qr = np.array(qrcode.make(payload, box_size=10, border=4).convert('RGB'))
    yolo = YoloOnnxDetector(conf_thr=.35)
    classical = ClassicalDetector()
    gate = QrFilter()
    results = []
    for size in args.sizes:
        frame = np.full((args.height,args.width,3), (70,100,70), np.uint8)
        # Center placement: conservative tile scale (center tiles overlap on both sides).
        x, y = (args.width-size)//2, (args.height-size)//2
        frame[y:y+size,x:x+size] = cv2.resize(qr,(size,size),interpolation=cv2.INTER_AREA)
        result = {'qr_outer_px':size, 'frame':[args.width,args.height]}
        for name, detect in [('classical',classical.detect), ('yolo_full',yolo.detect),
                             ('yolo_3x3',lambda f:detect_tiles(yolo,f,3,3,.25))]:
            t = time.monotonic()
            boxes = gate.filter_boxes(detect(frame))
            # Require box overlapping the known QR (not just any green-ground false positive).
            hits = [b for b in boxes if b.x < x+size and b.y < y+size and b.x+b.w > x and b.y+b.h > y]
            found = any(decode_frame(frame,(b.x,b.y,b.w,b.h)) == payload for b in hits[:3])
            result[name] = {'detected':bool(hits),'decoded_from_native_roi':found,
                            'elapsed_s':round(time.monotonic()-t,3)}
        # Known-location ROI checks optics/sampling separately from detector success.
        result['known_native_roi_decodes'] = decode_frame(frame[y-12:y+size+12,x-12:x+size+12]) == payload
        results.append(result)
        print(json.dumps(result), flush=True)
    return results


if __name__ == '__main__':
    main()
