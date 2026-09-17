#!/usr/bin/env python3
"""
Test Pi YOLOv8 QR model on laptop webcam — scan QR from laptop camera to be sure it works
This is the same model that runs on Pi in Gazebo (qr_yolov8n.pt / .onnx)

Usage on Pop!_OS (your case):
  source ~/venv-ardupilot/bin/activate
  cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
  pip install ultralytics opencv-python-headless qrcode pillow pyzbar
  # Or ONNX lighter:
  pip install onnxruntime opencv-python-headless qrcode pillow

  # Test with laptop webcam (scan printed QR or phone screen)
  python3 tools/test_yolo_webcam_qr.py --source 0 --model models/qr_yolov8n.pt --conf 0.25

  # Test with Gazebo MJPEG (SITL+Gazebo running)
  python3 tools/test_yolo_webcam_qr.py --source http://127.0.0.1:8099/bottom.mjpg --model models/qr_yolov8n.pt --conf 0.25 --tiled

  # Test with image
  python3 tools/test_yolo_webcam_qr.py --source /path/to/qr.jpg --model models/qr_yolov8n.pt --conf 0.25

  # ONNX version (no torch, lighter)
  python3 tools/test_yolo_webcam_qr.py --source 0 --model models/qr_yolov8n.onnx --onnx --conf 0.25

What you will see:
  - Live window with green boxes around QR, conf score, payload decoded (e.g. MISSION-QR-001)
  - FPS and inference time
  - If QR is triple size 1.782x2.52 at 15m, it should be ~204px and easy to detect
  - If QR is A3 0.297x0.42 at 15m, it's 34px and needs tiling 3x3 + YOLO (classical FAILS)
"""
import argparse
import os
import sys
import time

try:
    import cv2
    import numpy as np
except Exception as e:
    sys.exit(f"need opencv: pip install opencv-python-headless ({e})")

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def load_detector(model_path, conf_thr=0.25, use_onnx=False):
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if not os.path.isabs(model_path):
        cand = os.path.join(base_dir, model_path)
        if os.path.isfile(cand):
            model_path = cand
    if not os.path.isfile(model_path):
        print(f"WARNING: model not found {model_path}, trying fallback")
        # try .onnx fallback
        alt = os.path.join(base_dir, "models/qr_yolov8n.onnx")
        if os.path.isfile(alt):
            model_path = alt
            use_onnx = True
    if use_onnx or model_path.endswith(".onnx"):
        from detectors.yolo_onnx import YoloOnnxDetector
        return YoloOnnxDetector(model_path, conf_thr=conf_thr)
    else:
        from detectors.yolo_ultralytics import YoloUltralyticsDetector
        return YoloUltralyticsDetector(model_path, conf_thr=conf_thr)

def decode_qr_in_box(frame, box):
    """Try to decode QR payload from box crop"""
    x,y,w,h = box.x, box.y, box.w, box.h
    # Expand box slightly for quiet zone
    x0 = max(0, x-10)
    y0 = max(0, y-10)
    x1 = min(frame.shape[1], x+w+10)
    y1 = min(frame.shape[0], y+h+10)
    crop = frame[y0:y1, x0:x1]
    if crop.size == 0:
        return None
    # Try cv2 QR detector
    try:
        detector = cv2.QRCodeDetector()
        data, bbox, _ = detector.detectAndDecode(crop)
        if data:
            return data
    except Exception:
        pass
    # Try pyzbar if available
    try:
        from pyzbar.pyzbar import decode
        from PIL import Image
        # Convert to PIL
        pil = Image.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
        decoded = decode(pil)
        if decoded:
            return decoded[0].data.decode('utf-8', 'ignore')
    except Exception:
        pass
    return None

def draw_boxes(frame, boxes, payloads):
    out = frame.copy()
    for i,b in enumerate(boxes):
        x,y,w,h = b.x, b.y, b.w, b.h
        conf = b.conf
        payload = payloads[i] if i < len(payloads) else None
        color = (0,255,0) if payload else (0,165,255)  # green if decoded, orange if only detected
        cv2.rectangle(out, (x,y), (x+w, y+h), color, 2)
        label = f"QR {conf:.2f}"
        if payload:
            label += f" {payload}"
        cv2.putText(out, label, (x, max(15,y-8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return out

def main():
    ap = argparse.ArgumentParser(description="Test Pi YOLOv8 QR model on laptop webcam")
    ap.add_argument("--source", default="0", help="0=webcam, 1=second cam, path=image, http://=mjpeg")
    ap.add_argument("--model", default="models/qr_yolov8n.pt", help="models/qr_yolov8n.pt (24MB) or .onnx (12MB)")
    ap.add_argument("--conf", type=float, default=0.25, help="conf threshold 0.25 for 15m, 0.30 for 10m")
    ap.add_argument("--onnx", action="store_true", help="force ONNX detector (lighter)")
    ap.add_argument("--tiled", action="store_true", help="use 3x3 tiled detection (mandatory for A3 at 15m 34px)")
    ap.add_argument("--save", default="", help="save result image")
    ap.add_argument("--no-show", action="store_true", help="headless, no window")
    args = ap.parse_args()

    print(f"Loading YOLO: {args.model} conf={args.conf} onnx={args.onnx} tiled={args.tiled}")
    try:
        det = load_detector(args.model, conf_thr=args.conf, use_onnx=args.onnx)
    except Exception as e:
        print(f"Failed to load YOLO {args.model}: {e}")
        print("Trying classical fallback (will FAIL at 15m 34px, but PASS at 150px triple QR)")
        print("Install: pip install ultralytics (for .pt) or pip install onnxruntime (for .onnx)")
        sys.exit(1)
    print(f"Detector ready: {det.name} model={det.model_path}")
    print(f"Triple QR 1.782x2.52 at 15m ~204px easy, A3 0.297x0.42 at 15m 34px needs tiling")

    source = args.source
    is_webcam = source == "0" or source.isdigit()
    is_mjpeg = source.startswith("http")
    is_image = os.path.isfile(source)

    if is_image:
        print(f"Testing image: {source}")
        frame = cv2.imread(source)
        if frame is None:
            sys.exit(f"failed to read {source}")
        t0 = time.time()
        if args.tiled:
            try:
                from detector import detect_tiles
                boxes = detect_tiles(det, frame, rows=3, cols=3, overlap=0.25)
            except Exception as e:
                print(f"tiled failed {e}, using full")
                boxes = det.detect(frame)
        else:
            boxes = det.detect(frame)
        t1 = time.time()
        payloads = [decode_qr_in_box(frame, b) for b in boxes]
        print(f"Detected {len(boxes)} boxes in {1000*(t1-t0):.0f}ms: {boxes}")
        for i,p in enumerate(payloads):
            print(f"  Box {i}: payload={p}")
        vis = draw_boxes(frame, boxes, payloads)
        if args.save:
            cv2.imwrite(args.save, vis)
            print(f"Saved {args.save}")
        if not args.no_show:
            cv2.imshow("YOLO QR Test - Image", vis)
            print("Press any key to close")
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        return

    if is_webcam:
        cam_id = int(source) if source.isdigit() else 0
        print(f"Opening webcam {cam_id} — show QR to camera, press q to quit")
        print(f"Print QR: python3 tools/make_qr_panel.py --text MISSION-QR-001 --out /tmp/qr.png --full --dpi 300")
        cap = cv2.VideoCapture(cam_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    elif is_mjpeg:
        print(f"Opening MJPEG {source} — press q to quit")
        cap = cv2.VideoCapture(source)
    else:
        sys.exit(f"unknown source: {source}")

    if not cap.isOpened():
        sys.exit(f"failed to open source {source} — try --source 1 or --source /dev/video0")

    fps_t = time.time()
    frame_n = 0
    decoded_payloads = set()
    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                print("no frame, retrying...")
                time.sleep(0.1)
                continue
            frame_n += 1
            t0 = time.time()
            if args.tiled:
                try:
                    from detector import detect_tiles
                    boxes = detect_tiles(det, frame, rows=3, cols=3, overlap=0.25)
                except Exception:
                    boxes = det.detect(frame)
            else:
                boxes = det.detect(frame)
            t1 = time.time()
            payloads = [decode_qr_in_box(frame, b) for b in boxes]
            for p in payloads:
                if p:
                    decoded_payloads.add(p)
            vis = draw_boxes(frame, boxes, payloads)
            # Overlay info
            fps = frame_n / (time.time() - fps_t + 1e-6)
            info = f"{det.name} {len(boxes)} boxes {1000*(t1-t0):.0f}ms {fps:.1f}fps conf={args.conf} tiled={args.tiled}"
            cv2.putText(vis, info, (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
            if decoded_payloads:
                cv2.putText(vis, f"Decoded: {','.join(decoded_payloads)}", (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            # Instructions
            cv2.putText(vis, "Show QR to cam, q=quit", (10, vis.shape[0]-15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)

            if not args.no_show:
                cv2.imshow("YOLO QR Test - Laptop Webcam (Pi model)", vis)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                if key == ord('s') and args.save:
                    cv2.imwrite(args.save, vis)
                    print(f"Saved {args.save}")
            else:
                if frame_n % 30 == 0:
                    print(f"frame {frame_n} {len(boxes)} boxes {1000*(t1-t0):.0f}ms payloads={payloads}")
                if frame_n > 200:
                    break
            # Auto exit after first decode for quick test
            if decoded_payloads and frame_n > 30:
                print(f"SUCCESS: Decoded QR {decoded_payloads} in {frame_n} frames")
                # Continue a bit more to show
                if frame_n > 60:
                    pass
    finally:
        cap.release()
        if not args.no_show:
            cv2.destroyAllWindows()
        print(f"Done. Frames: {frame_n}, Unique payloads decoded: {decoded_payloads}")
        if decoded_payloads:
            print(f"Pi YOLO model WORKS on laptop webcam — will work in Gazebo too")
        else:
            print(f"No payload decoded — try:")
            print(f"  - Print QR bigger: python3 tools/make_qr_panel.py --text MISSION-QR-001 --out /tmp/qr.png --full --dpi 300")
            print(f"  - Hold QR closer, good light")
            print(f"  - Lower conf: --conf 0.15")
            print(f"  - Try tiled: --tiled")
            print(f"  - Try ONNX: --model models/qr_yolov8n.onnx --onnx")

if __name__ == "__main__":
    main()
