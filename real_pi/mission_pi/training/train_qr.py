#!/usr/bin/env python3
"""Fine-tune YOLOv8n on the synthetic QR dataset, then export ONNX.

Runs on x86 (+GPU if you have one) — NOT on the Pi. Output best.pt goes
through training/compile_hef.md to become the .hef the HAT runs.

  python train_qr.py --data qr_dataset/qr.yaml --epochs 120 --imgsz 640
"""
import argparse
import os


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="qr_dataset/qr.yaml")
    ap.add_argument("--model", default="yolov8n.pt")
    ap.add_argument("--epochs", type=int, default=120)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=32)
    ap.add_argument("--project", default="runs")
    ap.add_argument("--name", default="qr_yolov8n")
    ap.add_argument("--device", default="0")
    args = ap.parse_args()
    try:
        from ultralytics import YOLO
    except Exception as e:
        raise SystemExit("need ultralytics (x86): pip install ultralytics (%r)" % (e,))
    model = YOLO(args.model)
    model.train(data=args.data, epochs=args.epochs, imgsz=args.imgsz,
                batch=args.batch, project=args.project, name=args.name,
                device=args.device, patience=25, cos_lr=True,
                hsv_h=0.0, hsv_s=0.3, hsv_v=0.5,  # lighting duty, keep hue
                fliplr=0.5, mosaic=0.6, mixup=0.0)
    best = os.path.join(args.project, args.name, "weights", "best.pt")
    print("best: %s" % best)
    YOLO(best).export(format="onnx", opset=13, simplify=True)  # opset 13: DFC-safe
    print("next: training/compile_hef.md (best.onnx -> qr_yolov8n.hef)")


if __name__ == "__main__":
    main()
