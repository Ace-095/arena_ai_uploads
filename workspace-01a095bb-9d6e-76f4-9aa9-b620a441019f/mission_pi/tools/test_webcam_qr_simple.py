#!/usr/bin/env python3
"""
Simple webcam QR test — no YOLO needed, just to prove QR scanning works on laptop
Then you can compare with YOLO model

Usage:
  python3 tools/test_webcam_qr_simple.py --source 0
  # Shows live webcam, detects QR with cv2, decodes payload, draws box

  # For YOLO test (Pi model):
  python3 tools/test_yolo_webcam_qr.py --source 0 --model models/qr_yolov8n.pt --conf 0.15
  python3 tools/test_yolo_webcam_qr.py --source 0 --model models/qr_yolov8n.onnx --onnx --conf 0.05 --tiled
"""
import argparse
import sys
import time

try:
    import cv2
except Exception as e:
    sys.exit(f"need opencv: {e}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0", help="0=webcam, 1=second cam")
    args = ap.parse_args()

    cam_id = int(args.source) if args.source.isdigit() else 0
    print(f"Opening webcam {cam_id} — show QR to camera, q=quit")
    print(f"Generate QR: python3 tools/make_qr_panel.py --text MISSION-QR-001 --out /tmp/qr.png --full --dpi 300")
    cap = cv2.VideoCapture(cam_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    if not cap.isOpened():
        sys.exit(f"failed to open {cam_id}")

    detector = cv2.QRCodeDetector()
    decoded_set = set()
    frame_n = 0
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.1)
                continue
            frame_n += 1
            data, bbox, _ = detector.detectAndDecode(frame)
            vis = frame.copy()
            if bbox is not None and data:
                # bbox is 4x2 array
                pts = bbox[0].astype(int)
                cv2.polylines(vis, [pts], True, (0,255,0), 2)
                cv2.putText(vis, f"QR: {data}", (pts[0][0], pts[0][1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
                decoded_set.add(data)
                print(f"Frame {frame_n}: DECODED '{data}'")
            # Show decoded history
            if decoded_set:
                cv2.putText(vis, f"Decoded: {','.join(decoded_set)}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.putText(vis, "Show QR, q=quit", (10, vis.shape[0]-15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)
            cv2.imshow("Webcam QR Test - Classical (no YOLO)", vis)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print(f"Done, decoded: {decoded_set}")

if __name__ == "__main__":
    main()
