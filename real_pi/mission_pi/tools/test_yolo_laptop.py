#!/usr/bin/env python3
"""
Test YOLO on laptop (Pop!_OS) — webcam, image, or Gazebo MJPEG
Shows live detection result with bounding boxes

Usage:
  # Webcam (laptop)
  python3 tools/test_yolo_laptop.py --source 0 --model models/qr_yolov8n.pt --conf 0.25

  # Image file
  python3 tools/test_yolo_laptop.py --source /path/to/qr.jpg --model models/qr_yolov8n.pt

  # Gazebo MJPEG (SITL+Gazebo)
  python3 tools/test_yolo_laptop.py --source http://127.0.0.1:8099/bottom.mjpg --model models/qr_yolov8n.pt --tiled

  # ONNX version (lighter, no torch)
  pip install onnxruntime
  python3 tools/test_yolo_laptop.py --source 0 --model models/qr_yolov8n.onnx --onnx --conf 0.25

  # Save result
  python3 tools/test_yolo_laptop.py --source test.jpg --model models/qr_yolov8n.pt --save result.jpg
"""
import argparse
import os
import sys
import time

try:
    import cv2
    import numpy as np
except Exception as e:
    sys.exit(f"need opencv: {e}")

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def load_detector(model_path, conf_thr=0.25, use_onnx=False):
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if not os.path.isabs(model_path):
        cand = os.path.join(base_dir, model_path)
        if os.path.isfile(cand):
            model_path = cand
    if use_onnx or model_path.endswith(".onnx"):
        from detectors.yolo_onnx import YoloOnnxDetector
        return YoloOnnxDetector(model_path, conf_thr=conf_thr)
    else:
        from detectors.yolo_ultralytics import YoloUltralyticsDetector
        return YoloUltralyticsDetector(model_path, conf_thr=conf_thr)

def draw_boxes(frame, boxes):
    out = frame.copy()
    for b in boxes:
        x,y,w,h = b.x, b.y, b.w, b.h
        conf = b.conf
        cv2.rectangle(out, (x,y), (x+w, y+h), (0,255,0), 2)
        cv2.putText(out, f"QR {conf:.2f}", (x, max(10,y-5)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
    return out

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0", help="0=webcam, path=image, http://=mjpeg")
    ap.add_argument("--model", default="models/qr_yolov8n.pt")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--onnx", action="store_true")
    ap.add_argument("--tiled", action="store_true", help="use 3x3 tiled detection (for 15m)")
    ap.add_argument("--save", default="", help="save result image path")
    ap.add_argument("--no-show", action="store_true", help="don't show window (headless)")
    args = ap.parse_args()

    print(f"Loading YOLO: {args.model} conf={args.conf} onnx={args.onnx}")
    det = load_detector(args.model, conf_thr=args.conf, use_onnx=args.onnx)
    print(f"Detector ready: {det.name} model={det.model_path}")

    # Source handling
    source = args.source
    is_webcam = source == "0" or source.isdigit()
    is_mjpeg = source.startswith("http")
    is_image = os.path.isfile(source)

    if is_image:
        print(f"Testing image: {source}")
        frame = cv2.imread(source)
        if frame is None:
            sys.exit(f"failed to read {source}")
        if args.tiled:
            from detector import detect_tiles
            boxes = detect_tiles(det, frame, rows=3, cols=3, overlap=0.25)
        else:
            boxes = det.detect(frame)
        print(f"Detected {len(boxes)} boxes: {boxes}")
        vis = draw_boxes(frame, boxes)
        if args.save:
            cv2.imwrite(args.save, vis)
            print(f"Saved {args.save}")
        if not args.no_show:
            cv2.imshow("YOLO QR", vis)
            cv2.waitKey(0)
        return

    if is_webcam:
        cam_id = int(source) if source.isdigit() else 0
        print(f"Opening webcam {cam_id} — press q to quit")
        cap = cv2.VideoCapture(cam_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    elif is_mjpeg:
        print(f"Opening MJPEG {source} — press q to quit")
        cap = cv2.VideoCapture(source)
    else:
        sys.exit(f"unknown source: {source}")

    if not cap.isOpened():
        sys.exit(f"failed to open source {source}")

    fps_t = time.time()
    frame_n = 0
    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                print("no frame")
                time.sleep(0.1)
                continue
            frame_n += 1
            t0 = time.time()
            if args.tiled:
                from detector import detect_tiles
                boxes = detect_tiles(det, frame, rows=3, cols=3, overlap=0.25)
            else:
                boxes = det.detect(frame)
            t1 = time.time()
            vis = draw_boxes(frame, boxes)
            # FPS overlay
            fps = frame_n / (time.time() - fps_t + 1e-6)
            cv2.putText(vis, f"{det.name} {len(boxes)} boxes {1000*(t1-t0):.0f}ms {fps:.1f}fps conf={args.conf} tiled={args.tiled}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
            if not args.no_show:
                cv2.imshow("YOLO QR Test", vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                if frame_n % 30 == 0:
                    print(f"frame {frame_n} {len(boxes)} boxes {1000*(t1-t0):.0f}ms")
                if frame_n > 100:
                    break
    finally:
        cap.release()
        if not args.no_show:
            cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
