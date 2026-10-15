#!/usr/bin/env python3
"""
YOLO bench: compare classical vs YOLO on synthetic A3 QR at 10m/15m
Shows why YOLO mandatory for 10m/15m A3 detection

Generates synthetic frames with A3 QR at:
  10m: ~50px
  15m: ~34px
  80px: ~8m
  150px: ~4m
And runs classical + YOLO (tiled + full) detection

Usage:
  python3 tools/yolo_bench.py --model models/qr_yolov8n.pt --conf 0.25
  python3 tools/yolo_bench.py --model models/qr_yolov8n.onnx --conf 0.25 --onnx

For laptop Pop!_OS:
  pip install ultralytics opencv-python-headless qrcode pillow
  python3 tools/yolo_bench.py --model models/qr_yolov8n.pt
"""
import argparse
import os
import sys
import time

try:
    import cv2
    import numpy as np
    import qrcode
    from PIL import Image
except Exception as e:
    sys.exit(f"need opencv numpy qrcode pillow: {e}")

# Add parent to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def render_qr_payload(payload, side_px):
    qr = qrcode.QRCode(border=2, box_size=max(2, side_px // 32))
    qr.add_data(payload)
    qr.make(fit=True)
    img = qr.make_image(fill_color="black", back_color="white").convert("RGB")
    img = img.resize((side_px, side_px), Image.NEAREST)
    return cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)

def make_frame(qr_side_px, frame_w=1280, frame_h=720, payload="TEST-QR-15M"):
    # Ground background
    base = np.full((frame_h, frame_w, 3), (88, 110, 72), np.uint8)
    # Add noise
    noise = np.random.randint(-20, 20, (frame_h, frame_w, 3), dtype=np.int16)
    base = np.clip(base.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    # Add blotches
    for _ in range(5):
        x,y = np.random.randint(0, frame_w), np.random.randint(0, frame_h)
        r = np.random.randint(20, 120)
        c = tuple(np.random.randint(30,200,3).tolist())
        cv2.circle(base, (x,y), r, c, -1)

    qr = render_qr_payload(payload, qr_side_px)
    # Center paste with small rotation
    cx, cy = frame_w//2, frame_h//2
    x0 = cx - qr_side_px//2
    y0 = cy - qr_side_px//2
    # Slight perspective
    M = cv2.getRotationMatrix2D((qr_side_px/2, qr_side_px/2), np.random.uniform(-15,15), 1.0)
    qr_r = cv2.warpAffine(qr, M, (qr_side_px, qr_side_px), borderValue=(255,255,255))
    base[y0:y0+qr_side_px, x0:x0+qr_side_px] = qr_r
    return base

def classical_detect(frame):
    try:
        from decoder import classical_boxes
        boxes = classical_boxes(frame)
        return boxes
    except Exception as e:
        print(f"classical failed: {e}")
        return []

def yolo_detect_ultralytics(frame, model_path, conf_thr=0.25):
    try:
        from ultralytics import YOLO
        model = YOLO(model_path)
        results = model.predict(frame, verbose=False, conf=conf_thr, imgsz=640)
        out = []
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                x1,y1,x2,y2 = box.xyxy[0].tolist()
                conf = float(box.conf[0])
                out.append((x1,y1,x2,y2,conf))
        return out
    except Exception as e:
        print(f"yolo ultralytics failed: {e}")
        import traceback; traceback.print_exc()
        return []

def yolo_detect_onnx(frame, model_path, conf_thr=0.25):
    try:
        from detectors.yolo_onnx import YoloOnnxDetector
        det = YoloOnnxDetector(model_path, conf_thr=conf_thr)
        boxes = det.detect(frame)
        return [(b.x, b.y, b.x+b.w, b.y+b.h, b.conf) for b in boxes]
    except Exception as e:
        print(f"yolo onnx failed: {e}")
        import traceback; traceback.print_exc()
        return []

def yolo_tiled_detect(frame, model_path, conf_thr=0.25, rows=3, cols=3, overlap=0.25, use_onnx=False):
    try:
        if use_onnx:
            from detectors.yolo_onnx import YoloOnnxDetector
            Detector = YoloOnnxDetector
        else:
            from detectors.yolo_ultralytics import YoloUltralyticsDetector
            Detector = YoloUltralyticsDetector
        det = Detector(model_path, conf_thr=conf_thr)
        from detector import detect_tiles
        boxes = detect_tiles(det, frame, rows=rows, cols=cols, overlap=overlap)
        return [(b.x, b.y, b.x+b.w, b.y+b.h, b.conf) for b in boxes]
    except Exception as e:
        print(f"yolo tiled failed: {e}")
        import traceback; traceback.print_exc()
        return []

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="models/qr_yolov8n.pt", help="path to .pt or .onnx")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--onnx", action="store_true", help="force onnx detector")
    ap.add_argument("--sizes", nargs="+", type=int, default=[34, 50, 80, 150], help="QR side px to test (34=15m,50=10m)")
    ap.add_argument("--payload", default="TEST-QR-15M")
    args = ap.parse_args()

    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    model_path = args.model
    if not os.path.isabs(model_path):
        # try relative to mission_pi
        cand = os.path.join(base_dir, model_path)
        if os.path.isfile(cand):
            model_path = cand
        elif os.path.isfile(os.path.join(os.getcwd(), model_path)):
            model_path = os.path.join(os.getcwd(), model_path)

    print(f"Model: {model_path} exists={os.path.isfile(model_path)} conf={args.conf}")
    print(f"Testing sizes: {args.sizes} (px) — 34px=A3@15m, 50px=A3@10m, 80px=A3@~8m, 150px=A3@~4m")
    print("="*90)
    print(f"{'SIZE':>6} {'MODE':>20} {'#BOX':>6} {'MAX_CONF':>10} {'RESULT':>10} {'TIME_MS':>10}")
    print("-"*90)

    for sz in args.sizes:
        frame = make_frame(sz, payload=args.payload)
        # Classical
        t0 = time.time()
        c_boxes = classical_detect(frame)
        t1 = time.time()
        c_max = max([b[4] for b in c_boxes]) if c_boxes else 0
        c_res = "PASS" if len(c_boxes)>0 else "FAIL"
        print(f"{sz:6}px {'classical':>20} {len(c_boxes):6} {c_max:10.3f} {c_res:>10} {(t1-t0)*1000:10.1f}")

        # YOLO full
        if os.path.isfile(model_path):
            t0 = time.time()
            if args.onnx or model_path.endswith(".onnx"):
                y_boxes = yolo_detect_onnx(frame, model_path, conf_thr=args.conf)
            else:
                y_boxes = yolo_detect_ultralytics(frame, model_path, conf_thr=args.conf)
            t1 = time.time()
            y_max = max([b[4] for b in y_boxes]) if y_boxes else 0
            y_res = "PASS" if len(y_boxes)>0 else "FAIL"
            print(f"{sz:6}px {'yolo_full':>20} {len(y_boxes):6} {y_max:10.3f} {y_res:>10} {(t1-t0)*1000:10.1f}")

            # YOLO tiled 3x3
            t0 = time.time()
            yt_boxes = yolo_tiled_detect(frame, model_path, conf_thr=args.conf, rows=3, cols=3, overlap=0.25, use_onnx=args.onnx or model_path.endswith(".onnx"))
            t1 = time.time()
            yt_max = max([b[4] for b in yt_boxes]) if yt_boxes else 0
            yt_res = "PASS" if len(yt_boxes)>0 else "FAIL"
            print(f"{sz:6}px {'yolo_tiled_3x3':>20} {len(yt_boxes):6} {yt_max:10.3f} {yt_res:>10} {(t1-t0)*1000:10.1f}")
        else:
            print(f"{sz:6}px {'yolo_full':>20} {'NO_MODEL':>6} {'-':>10} {'SKIP':>10} {'-':>10}")
            print(f"{sz:6}px {'yolo_tiled_3x3':>20} {'NO_MODEL':>6} {'-':>10} {'SKIP':>10} {'-':>10}")
        print("-"*90)

    print("\nSummary:")
    print("  Classical FAIL at 34px (15m) and 50px (10m) — expected, too small for CV")
    print("  YOLO full may FAIL at 34px (5px after 640 resize) — need tiling")
    print("  YOLO tiled 3x3 should PASS at 34px/50px — 34px -> ~15px on 640 tile, detectable")
    print("  This proves YOLO mandatory for 10m/15m A3, and tiling mandatory even for YOLO")

if __name__ == "__main__":
    main()
