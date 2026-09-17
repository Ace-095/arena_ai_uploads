#!/usr/bin/env python3
"""
Test Pi YOLOv8 QR model on laptop webcam — with LIVE preset control in preview window
Same model that runs on Pi in Gazebo (qr_yolov8n.pt / .onnx)

Usage on Pop!_OS (your case):
  source ~/venv-ardupilot/bin/activate
  cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
  export QT_QPA_PLATFORM=xcb
  pip install ultralytics opencv-python-headless qrcode pillow pyzbar onnxruntime

  # Laptop webcam with live presets — press keys in window to tune dark
  python3 tools/test_yolo_webcam_qr.py --source 0 --model models/qr_yolov8n.pt --conf 0.15 --preset dark_qr_boost
  python3 tools/test_yolo_webcam_qr.py --source 0 --model models/qr_yolov8n.onnx --onnx --conf 0.10 --preset dark

  # Gazebo MJPEG (SITL+Gazebo running)
  python3 tools/test_yolo_webcam_qr.py --source http://127.0.0.1:8099/bottom.mjpg --model models/qr_yolov8n.pt --conf 0.15 --tiled --preset gazebo_dark

Keys in preview window:
  q = quit
  p / n = next/prev preset (cycle daylight/dark/night/dark_qr_boost/qr_boost_day/gazebo_dark)
  1-7 = direct preset: 1=daylight 2=dark 3=night 4=dark_qr_boost 5=qr_boost_day 6=qr_boost_night 7=gazebo_dark
  b/B = brightness -/+ 10 (V4L2 + SW)
  c/C = contrast -/+ 0.2
  s/S = saturation -/+ 0.2
  g/G = gain/ISO -/+ 5
  e/E = exposure -/+ 50
  h/H = sharpness -/+ 0.2
  r = reset to current preset
  d = toggle SW enhance on/off (see raw vs boosted)
  t = toggle tiled detection (for A3 34px at 15m)
  space = save frame
  l = lower conf 0.05, k = raise conf 0.05

What you will see:
  - Live window with green boxes around QR, conf score, payload decoded (e.g. MISSION-QR-001)
  - Current preset + V4L2 values + SW enhance overlay
  - FPS and inference time
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

# Presets — integrated workflow (same as qr_filter + camera_tune)
# Import from qr_filter if available, else fallback
try:
    from qr_filter import PRESETS as FILTER_PRESETS, list_presets
    PRESETS = {}
    for k, v in FILTER_PRESETS.items():
        PRESETS[k] = {
            "v4l2": v.get("v4l2", {}),
            "sw": v.get("sw", {}),
            "desc": v.get("description", ""),
            "detector": v.get("detector", {}),
            "pi": v.get("pi", {}),
        }
    PRESET_ORDER = list_presets()
except Exception:
    # Fallback if qr_filter not available
    PRESETS = {
        "daylight": {"v4l2": {"brightness": 0, "contrast": 45, "saturation": 63, "gain": 0, "exposure": 50, "sharpness": 100}, "sw": {"brightness": 1.0, "contrast": 1.0, "saturation": 1.0, "sharpness": 1.0}, "desc": "Daylight — normal auto, default", "detector": {"conf_thr": 0.35, "require_decode": False, "hide_fake": True}},
        "dark": {"v4l2": {"brightness": 64, "contrast": 75, "saturation": 70, "gain": 20, "exposure": 300, "sharpness": 100}, "sw": {"brightness": 1.3, "contrast": 1.5, "saturation": 1.2, "sharpness": 1.3}, "desc": "Pitch dark — high exposure 30ms, gain 20dB, bright 80 — for dark you saw", "detector": {"conf_thr": 0.30, "require_decode": True, "hide_fake": True}},
        "night": {"v4l2": {"brightness": 64, "contrast": 80, "saturation": 63, "gain": 40, "exposure": 400, "sharpness": 100}, "sw": {"brightness": 1.5, "contrast": 1.8, "saturation": 1.0, "sharpness": 1.5}, "desc": "Night max — 50ms exposure, 24dB gain, brightness max", "detector": {"conf_thr": 0.30, "require_decode": True, "hide_fake": True}},
        "dark_qr_boost": {"v4l2": {"brightness": 64, "contrast": 90, "saturation": 30, "gain": 20, "exposure": 300, "sharpness": 100}, "sw": {"brightness": 1.2, "contrast": 2.2, "saturation": 0.6, "sharpness": 2.5}, "desc": "Dark + QR boost — BEST for QR in pitch dark, QR pops vs ground", "detector": {"conf_thr": 0.30, "require_decode": True, "hide_fake": True}},
        "qr_boost_day": {"v4l2": {"brightness": 0, "contrast": 75, "saturation": 30, "gain": 0, "exposure": 50, "sharpness": 100}, "sw": {"brightness": 1.0, "contrast": 1.8, "saturation": 0.6, "sharpness": 2.0}, "desc": "Daylight QR boost — contrast high, sat low, sharp high", "detector": {"conf_thr": 0.35, "require_decode": True, "hide_fake": True}},
        "qr_boost_night": {"v4l2": {"brightness": 64, "contrast": 90, "saturation": 30, "gain": 30, "exposure": 350, "sharpness": 100}, "sw": {"brightness": 1.3, "contrast": 2.0, "saturation": 0.6, "sharpness": 2.5}, "desc": "Night QR boost — dark + QR boost, best for night QR", "detector": {"conf_thr": 0.30, "require_decode": True, "hide_fake": True}},
        "gazebo_dark": {"v4l2": {"brightness": 64, "contrast": 85, "saturation": 50, "gain": 20, "exposure": 300, "sharpness": 100}, "sw": {"brightness": 1.3, "contrast": 2.2, "saturation": 0.8, "sharpness": 2.5}, "desc": "Gazebo dark — for gz_cam_bridge --qr-boost --contrast 2.2 --brightness 1.3", "detector": {"conf_thr": 0.25, "require_decode": False, "hide_fake": True}},
    }
    PRESET_ORDER = list(PRESETS.keys())
V4L2_PROPS = {
    "brightness": cv2.CAP_PROP_BRIGHTNESS,
    "contrast": cv2.CAP_PROP_CONTRAST,
    "saturation": cv2.CAP_PROP_SATURATION,
    "hue": cv2.CAP_PROP_HUE,
    "gain": cv2.CAP_PROP_GAIN,
    "exposure": cv2.CAP_PROP_EXPOSURE,
    "sharpness": cv2.CAP_PROP_SHARPNESS,
}

def apply_v4l2_settings(cap, v4l2_dict, verbose=True):
    if cap is None or not cap.isOpened():
        return {}
    result = {}
    for k, v in v4l2_dict.items():
        prop = V4L2_PROPS.get(k)
        if prop is None:
            continue
        try:
            ok = cap.set(prop, float(v))
            actual = cap.get(prop)
            result[k] = actual
            if verbose:
                print(f"  V4L2 {k}={v} -> actual={actual} ok={ok}")
        except Exception as e:
            if verbose:
                print(f"  V4L2 {k} failed: {e}")
    return result

def enhance_frame_sw(frame, sw_settings):
    """Software enhance: brightness, contrast, saturation, sharpness — same as gz_cam_bridge QR boost"""
    if frame is None:
        return frame
    bri = float(sw_settings.get("brightness", 1.0))
    cont = float(sw_settings.get("contrast", 1.0))
    sat = float(sw_settings.get("saturation", 1.0))
    sharp = float(sw_settings.get("sharpness", 1.0))

    out = frame
    # Brightness + Contrast via convertScaleAbs: alpha=contrast, beta=(brightness-1)*80
    # brightness 1.0=0 beta, 1.3=+24, 0.8=-16 etc. For dark we want +30
    if bri != 1.0 or cont != 1.0:
        beta = (bri - 1.0) * 80.0
        out = cv2.convertScaleAbs(out, alpha=cont, beta=beta)

    # Saturation via HSV S channel scaling
    if sat != 1.0:
        try:
            hsv = cv2.cvtColor(out, cv2.COLOR_BGR2HSV)
            h, s, v = cv2.split(hsv)
            s = s.astype(np.float32) * sat
            s = np.clip(s, 0, 255).astype(np.uint8)
            hsv = cv2.merge((h, s, v))
            out = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        except Exception:
            pass

    # Sharpness via unsharp mask
    if sharp != 1.0 and sharp > 0:
        try:
            if sharp > 1.0:
                # amount = sharp-1, e.g. 2.5 => 1.5 strong
                blurred = cv2.GaussianBlur(out, (0, 0), 3)
                amount = sharp - 1.0
                out = cv2.addWeighted(out, 1.0 + amount, blurred, -amount, 0)
            else:
                # blur if <1
                k = int((1.0 - sharp) * 10) | 1
                if k < 3:
                    k = 3
                out = cv2.GaussianBlur(out, (k, k), 0)
        except Exception:
            pass
    return out

def load_detector(model_path, conf_thr=0.25, use_onnx=False):
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if not os.path.isabs(model_path):
        cand = os.path.join(base_dir, model_path)
        if os.path.isfile(cand):
            model_path = cand
    if not os.path.isfile(model_path):
        print(f"WARNING: model not found {model_path}, trying fallback")
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

def decode_qr_in_box(frame, box, try_enhance=True):
    """Try to decode QR payload from box crop — returns payload or None.
    Tries multiple methods: cv2 QR, pyzbar, enhanced gray, CLAHE"""
    x,y,w,h = box.x, box.y, box.w, box.h
    x0 = max(0, x-10)
    y0 = max(0, y-10)
    x1 = min(frame.shape[1], x+w+10)
    y1 = min(frame.shape[0], y+h+10)
    crop = frame[y0:y1, x0:x1]
    if crop.size == 0:
        return None

    # 1) cv2 QR on color crop
    try:
        detector = cv2.QRCodeDetector()
        data, bbox, _ = detector.detectAndDecode(crop)
        if data:
            return data
    except Exception:
        pass

    # 2) pyzbar on color
    try:
        from pyzbar.pyzbar import decode
        from PIL import Image
        pil = Image.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
        decoded = decode(pil)
        if decoded:
            return decoded[0].data.decode('utf-8', 'ignore')
    except Exception:
        pass

    if not try_enhance:
        return None

    # 3) Enhanced attempts for dark / low contrast — gray + CLAHE + threshold
    try:
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY) if len(crop.shape)==3 else crop
        # upscale small crops for better decode (15m triple QR ~200px, but A3 34px needs 3x)
        if min(gray.shape[:2]) < 100:
            gray = cv2.resize(gray, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)

        # CLAHE
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        gray_enh = clahe.apply(gray)

        detector = cv2.QRCodeDetector()
        for g in (gray_enh, gray, cv2.threshold(gray_enh, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)[1]):
            data, bbox, _ = detector.detectAndDecode(g)
            if data:
                return data

        # pyzbar on enhanced gray
        try:
            from pyzbar.pyzbar import decode
            from PIL import Image
            pil = Image.fromarray(gray_enh)
            decoded = decode(pil)
            if decoded:
                return decoded[0].data.decode('utf-8', 'ignore')
        except Exception:
            pass
    except Exception:
        pass

    return None

# Use qr_filter module if available (integrated workflow)
try:
    from qr_filter import is_fake_box as _is_fake_box, filter_boxes as _filter_boxes
    def is_fake_box(box, frame_shape=None):
        fake, _ = _is_fake_box(box, frame_shape)
        return fake
    def filter_boxes(boxes, payloads, frame_shape=None, require_decode=False, hide_fake=False, min_conf=0.0):
        return _filter_boxes(boxes, payloads, frame_shape, require_decode, hide_fake, min_conf)
except Exception:
    def is_fake_box(box, frame_shape=None):
        """Heuristic to filter obvious non-QR: extreme aspect, too small, too large"""
        x,y,w,h = box.x, box.y, box.w, box.h
        if w < 15 or h < 15:
            return True
        if frame_shape:
            fh, fw = frame_shape[:2]
            if w > fw*0.9 or h > fh*0.9:
                return True
        aspect = w / (h+1e-6)
        if aspect < 0.4 or aspect > 2.5:
            return True
        return False

    def filter_boxes(boxes, payloads, frame_shape=None, require_decode=False, hide_fake=False, min_conf=0.0):
        out_boxes = []
        out_payloads = []
        for b, p in zip(boxes, payloads):
            if b.conf < min_conf:
                continue
            if hide_fake and is_fake_box(b, frame_shape):
                continue
            if require_decode and p is None:
                continue
            if hide_fake and p is None and b.conf < 0.35:
                continue
            out_boxes.append(b)
            out_payloads.append(p)
        return out_boxes, out_payloads

def draw_boxes(frame, boxes, payloads, show_fake=True):
    out = frame.copy()
    for i,b in enumerate(boxes):
        x,y,w,h = b.x, b.y, b.w, b.h
        conf = b.conf
        payload = payloads[i] if i < len(payloads) else None
        if payload:
            color = (0,255,0)  # green = real decoded
            thick = 3
        else:
            if not show_fake:
                continue
            color = (0,165,255)  # orange = YOLO only, no decode — likely fake like grill
            thick = 2
        cv2.rectangle(out, (x,y), (x+w, y+h), color, thick)
        label = f"QR {conf:.2f}"
        if payload:
            label += f" {payload}"
        else:
            label += " (no decode)"
        cv2.putText(out, label, (x, max(15,y-8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return out

def main():
    ap = argparse.ArgumentParser(description="Test Pi YOLOv8 QR model on laptop webcam with live preset control")
    ap.add_argument("--source", default="0", help="0=webcam, 1=second cam, path=image, http://=mjpeg")
    ap.add_argument("--model", default="models/qr_yolov8n.pt", help="models/qr_yolov8n.pt (24MB) or .onnx (12MB)")
    ap.add_argument("--conf", type=float, default=0.35, help="conf threshold 0.35 default to kill fakes (was 0.15), 0.25 for 15m ground, 0.10 for debug")
    ap.add_argument("--onnx", action="store_true", help="force ONNX detector (lighter)")
    ap.add_argument("--tiled", action="store_true", help="use 3x3 tiled detection (mandatory for A3 at 15m 34px)")
    ap.add_argument("--save", default="", help="save result image")
    ap.add_argument("--no-show", action="store_true", help="headless, no window")
    # New: preset control
    ap.add_argument("--preset", default="daylight", choices=list(PRESETS.keys()), help="initial preset for dark control")
    ap.add_argument("--brightness", type=float, default=None, help="SW brightness 0.5..2.0")
    ap.add_argument("--contrast", type=float, default=None, help="SW contrast 0.5..3.0")
    ap.add_argument("--saturation", type=float, default=None, help="SW saturation 0.0..2.0")
    ap.add_argument("--sharpness", type=float, default=None, help="SW sharpness 0.0..3.0")
    ap.add_argument("--gain", type=float, default=None, help="V4L2 gain 0..100")
    ap.add_argument("--exposure", type=float, default=None, help="V4L2 exposure 1..500")
    ap.add_argument("--v4l2-brightness", type=float, default=None, help="V4L2 brightness -64..64")
    # Fake filtering — fix for window grill false positives you saw
    ap.add_argument("--require-decode", action="store_true", help="ONLY show boxes that actually decode to payload — kills ALL fakes (recommended for ground)")
    ap.add_argument("--decoded-only", action="store_true", help="alias for --require-decode")
    ap.add_argument("--hide-fake", action="store_true", help="hide low-conf non-decoded boxes + extreme aspect (your 0.16/0.24 grill)")
    ap.add_argument("--show-fake", action="store_true", help="show orange non-decoded boxes (debug, default now hidden for ground)")
    ap.add_argument("--min-size", type=int, default=15, help="min box size px to keep")
    args = ap.parse_args()

    print(f"Loading YOLO: {args.model} conf={args.conf} onnx={args.onnx} tiled={args.tiled} preset={args.preset}")
    try:
        det = load_detector(args.model, conf_thr=args.conf, use_onnx=args.onnx)
    except Exception as e:
        print(f"Failed to load YOLO {args.model}: {e}")
        print("Install: pip install ultralytics (for .pt) or pip install onnxruntime (for .onnx)")
        sys.exit(1)
    print(f"Detector ready: {det.name} model={det.model_path}")

    source = args.source
    is_webcam = source == "0" or source.isdigit()
    is_mjpeg = source.startswith("http")
    is_image = os.path.isfile(source)

    # Current preset state
    preset_idx = PRESET_ORDER.index(args.preset) if args.preset in PRESET_ORDER else 0
    current_preset_name = PRESET_ORDER[preset_idx]
    sw_settings = dict(PRESETS[current_preset_name]["sw"])
    v4l2_settings = dict(PRESETS[current_preset_name]["v4l2"])
    # Override from CLI
    if args.brightness is not None:
        sw_settings["brightness"] = args.brightness
    if args.contrast is not None:
        sw_settings["contrast"] = args.contrast
    if args.saturation is not None:
        sw_settings["saturation"] = args.saturation
    if args.sharpness is not None:
        sw_settings["sharpness"] = args.sharpness
    if args.gain is not None:
        v4l2_settings["gain"] = args.gain
    if args.exposure is not None:
        v4l2_settings["exposure"] = args.exposure
    if args.v4l2_brightness is not None:
        v4l2_settings["brightness"] = args.v4l2_brightness

    sw_enabled = True
    conf_thr = args.conf
    tiled = args.tiled
    # Fake filtering defaults — for ground you want no fakes
    require_decode = bool(args.require_decode or args.decoded_only)
    hide_fake = bool(args.hide_fake or require_decode)  # if require decode, also hide fake geometry
    # If user didn't specify show-fake, default to hide fakes for ground test (your image)
    # To debug, use --show-fake
    show_fake = bool(args.show_fake)
    if not require_decode and not args.hide_fake and not args.show_fake:
        # default for ground: hide fake low-conf orange boxes, but keep decoded green
        # User's image had 0.16/0.24 fake — these will be hidden by default now
        hide_fake = True
        show_fake = False
    if args.show_fake:
        show_fake = True
        hide_fake = False  # if explicitly show fake, don't hide

    print(f"Fake filter: require_decode={require_decode} hide_fake={hide_fake} show_fake={show_fake} conf_thr={conf_thr} — press f to toggle, v to toggle require-decode")

    if is_image:
        print(f"Testing image: {source} with preset {current_preset_name} sw={sw_settings} filter reqDecode={require_decode} hideFake={hide_fake}")
        frame = cv2.imread(source)
        if frame is None:
            sys.exit(f"failed to read {source}")
        if sw_enabled:
            frame = enhance_frame_sw(frame, sw_settings)
        t0 = time.time()
        if tiled:
            try:
                from detector import detect_tiles
                boxes = detect_tiles(det, frame, rows=3, cols=3, overlap=0.25)
            except Exception as e:
                print(f"tiled failed {e}, using full")
                boxes = det.detect(frame)
        else:
            boxes = det.detect(frame)
        t1 = time.time()
        payloads_raw = [decode_qr_in_box(frame, b) for b in boxes]
        boxes_f, payloads_f = filter_boxes(boxes, payloads_raw, frame_shape=frame.shape, require_decode=require_decode, hide_fake=hide_fake)
        print(f"Detected {len(boxes)} raw, {len(boxes_f)} after filter in {1000*(t1-t0):.0f}ms")
        for i,p in enumerate(payloads_raw):
            print(f"  Box {i} raw: {boxes[i]} payload={p} fake={is_fake_box(boxes[i], frame.shape)}")
        for i,p in enumerate(payloads_f):
            print(f"  Box {i} filt: {boxes_f[i]} payload={p}")
        boxes_draw = boxes_f if (hide_fake or require_decode) else boxes
        payloads_draw = payloads_f if (hide_fake or require_decode) else payloads_raw
        vis = draw_boxes(frame, boxes_draw, payloads_draw, show_fake=show_fake or not (hide_fake or require_decode))
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
        print(f"Opening webcam {cam_id} — preset {current_preset_name} — show QR to camera")
        print(f"Keys: q=quit p/n=next/prev preset 1-7=preset b/B bright c/C contrast s/S sat g/G gain e/E exp h/H sharp r=reset d=toggle enhance t=tiled space=save")
        cap = cv2.VideoCapture(cam_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    elif is_mjpeg:
        print(f"Opening MJPEG {source} — preset {current_preset_name} — press q to quit")
        cap = cv2.VideoCapture(source)
    else:
        sys.exit(f"unknown source: {source}")

    if not cap.isOpened():
        sys.exit(f"failed to open source {source} — try --source 1 or --source /dev/video0")

    # Apply initial V4L2 preset if webcam
    if is_webcam:
        print(f"Applying V4L2 preset {current_preset_name}: {v4l2_settings}")
        apply_v4l2_settings(cap, v4l2_settings)

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
            raw_frame = frame.copy()
            # Software enhance
            if sw_enabled:
                enhanced = enhance_frame_sw(frame, sw_settings)
            else:
                enhanced = frame

            t0 = time.time()
            if tiled:
                try:
                    from detector import detect_tiles
                    boxes = detect_tiles(det, enhanced, rows=3, cols=3, overlap=0.25)
                except Exception:
                    boxes = det.detect(enhanced)
            else:
                boxes = det.detect(enhanced)
            t1 = time.time()
            # Decode all
            payloads_raw = [decode_qr_in_box(enhanced, b) for b in boxes]
            for p in payloads_raw:
                if p:
                    decoded_payloads.add(p)

            # Filter fakes — this kills your window grill 0.16/0.24
            boxes_f, payloads_f = filter_boxes(boxes, payloads_raw, frame_shape=enhanced.shape,
                                               require_decode=require_decode, hide_fake=hide_fake, min_conf=0.0)
            # For drawing, use filtered if hide_fake/require_decode, else show all but with colors
            if hide_fake or require_decode:
                boxes_draw = boxes_f
                payloads_draw = payloads_f
            else:
                boxes_draw = boxes
                payloads_draw = payloads_raw

            vis = draw_boxes(enhanced, boxes_draw, payloads_draw, show_fake=show_fake or not (hide_fake or require_decode))

            # Overlay info — preset + controls + filter status
            fps = frame_n / (time.time() - fps_t + 1e-6)
            raw_cnt = len(boxes)
            filt_cnt = len(boxes_f)
            info1 = f"{det.name} raw:{raw_cnt} filt:{filt_cnt} {1000*(t1-t0):.0f}ms {fps:.1f}fps conf={conf_thr:.2f} tiled={tiled} sw={'ON' if sw_enabled else 'OFF'} filter: reqDecode={require_decode} hideFake={hide_fake}"
            cv2.putText(vis, info1, (10,25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)

            preset_info = PRESETS[current_preset_name]
            info2 = f"Preset [{preset_idx+1}/{len(PRESET_ORDER)}] {current_preset_name}: {preset_info['desc']}"
            cv2.putText(vis, info2, (10,50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1)

            # Show V4L2 + SW values
            v4l2_str = f"V4L2 b={v4l2_settings.get('brightness',0):.0f} c={v4l2_settings.get('contrast',0):.0f} sat={v4l2_settings.get('saturation',0):.0f} g={v4l2_settings.get('gain',0):.0f} e={v4l2_settings.get('exposure',0):.0f} sh={v4l2_settings.get('sharpness',0):.0f}"
            sw_str = f"SW bri={sw_settings.get('brightness',1):.2f} cont={sw_settings.get('contrast',1):.2f} sat={sw_settings.get('saturation',1):.2f} sharp={sw_settings.get('sharpness',1):.2f}"
            cv2.putText(vis, v4l2_str, (10,70), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200,200,255), 1)
            cv2.putText(vis, sw_str, (10,90), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200,255,200), 1)

            if decoded_payloads:
                cv2.putText(vis, f"Decoded: {','.join(decoded_payloads)}", (10,115), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            else:
                if hide_fake or require_decode:
                    cv2.putText(vis, f"No QR decoded — fakes hidden ({raw_cnt} raw filtered)", (10,115), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,165,255), 1)

            # Help line
            help_line = "q=quit p/n=preset 1-7=preset f=toggle hideFake v=reqDecode b/B c/C s/S g/G e/E h/H r=reset d=enh t=tiled l/k=conf"
            cv2.putText(vis, help_line, (10, vis.shape[0]-15), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200,200,200), 1)

            if not args.no_show:
                cv2.imshow("YOLO QR Test - Laptop Webcam (Pi model) + PRESET CONTROL", vis)
                key = cv2.waitKey(1) & 0xFF
                if key == 255:
                    continue
                if key == ord('q') or key == 27:
                    break
                elif key == ord('p') or key == ord('n'):  # next preset
                    preset_idx = (preset_idx + 1) % len(PRESET_ORDER)
                    current_preset_name = PRESET_ORDER[preset_idx]
                    sw_settings = dict(PRESETS[current_preset_name]["sw"])
                    v4l2_settings = dict(PRESETS[current_preset_name]["v4l2"])
                    print(f">>> Preset -> {current_preset_name}: {PRESETS[current_preset_name]['desc']}")
                    if is_webcam:
                        apply_v4l2_settings(cap, v4l2_settings)
                elif key == ord('o'):  # prev
                    preset_idx = (preset_idx - 1) % len(PRESET_ORDER)
                    current_preset_name = PRESET_ORDER[preset_idx]
                    sw_settings = dict(PRESETS[current_preset_name]["sw"])
                    v4l2_settings = dict(PRESETS[current_preset_name]["v4l2"])
                    print(f">>> Preset -> {current_preset_name}")
                    if is_webcam:
                        apply_v4l2_settings(cap, v4l2_settings)
                elif ord('1') <= key <= ord('7'):
                    idx = key - ord('1')
                    if idx < len(PRESET_ORDER):
                        preset_idx = idx
                        current_preset_name = PRESET_ORDER[preset_idx]
                        sw_settings = dict(PRESETS[current_preset_name]["sw"])
                        v4l2_settings = dict(PRESETS[current_preset_name]["v4l2"])
                        print(f">>> Preset {idx+1} -> {current_preset_name}")
                        if is_webcam:
                            apply_v4l2_settings(cap, v4l2_settings)
                elif key == ord('b'):
                    sw_settings["brightness"] = max(0.2, sw_settings["brightness"] - 0.1)
                    v4l2_settings["brightness"] = max(-64, v4l2_settings.get("brightness",0)-5)
                    if is_webcam:
                        apply_v4l2_settings(cap, {"brightness": v4l2_settings["brightness"]}, verbose=False)
                    print(f"brightness SW {sw_settings['brightness']:.2f} V4L2 {v4l2_settings['brightness']}")
                elif key == ord('B'):
                    sw_settings["brightness"] = min(3.0, sw_settings["brightness"] + 0.1)
                    v4l2_settings["brightness"] = min(64, v4l2_settings.get("brightness",0)+5)
                    if is_webcam:
                        apply_v4l2_settings(cap, {"brightness": v4l2_settings["brightness"]}, verbose=False)
                    print(f"brightness SW {sw_settings['brightness']:.2f} V4L2 {v4l2_settings['brightness']}")
                elif key == ord('c'):
                    sw_settings["contrast"] = max(0.2, sw_settings["contrast"] - 0.2)
                    v4l2_settings["contrast"] = max(0, v4l2_settings.get("contrast",45)-5)
                    if is_webcam:
                        apply_v4l2_settings(cap, {"contrast": v4l2_settings["contrast"]}, verbose=False)
                    print(f"contrast SW {sw_settings['contrast']:.2f} V4L2 {v4l2_settings['contrast']}")
                elif key == ord('C'):
                    sw_settings["contrast"] = min(4.0, sw_settings["contrast"] + 0.2)
                    v4l2_settings["contrast"] = min(100, v4l2_settings.get("contrast",45)+5)
                    if is_webcam:
                        apply_v4l2_settings(cap, {"contrast": v4l2_settings["contrast"]}, verbose=False)
                    print(f"contrast SW {sw_settings['contrast']:.2f} V4L2 {v4l2_settings['contrast']}")
                elif key == ord('s'):
                    sw_settings["saturation"] = max(0.0, sw_settings["saturation"] - 0.2)
                    v4l2_settings["saturation"] = max(0, v4l2_settings.get("saturation",63)-5)
                    if is_webcam:
                        apply_v4l2_settings(cap, {"saturation": v4l2_settings["saturation"]}, verbose=False)
                    print(f"saturation SW {sw_settings['saturation']:.2f} V4L2 {v4l2_settings['saturation']}")
                elif key == ord('S'):
                    sw_settings["saturation"] = min(3.0, sw_settings["saturation"] + 0.2)
                    v4l2_settings["saturation"] = min(100, v4l2_settings.get("saturation",63)+5)
                    if is_webcam:
                        apply_v4l2_settings(cap, {"saturation": v4l2_settings["saturation"]}, verbose=False)
                    print(f"saturation SW {sw_settings['saturation']:.2f} V4L2 {v4l2_settings['saturation']}")
                elif key == ord('g'):
                    sw_settings["brightness"] = max(0.2, sw_settings["brightness"] - 0.05)  # gain affects brightness too
                    v4l2_settings["gain"] = max(0, v4l2_settings.get("gain",0)-5)
                    if is_webcam:
                        apply_v4l2_settings(cap, {"gain": v4l2_settings["gain"]}, verbose=False)
                    print(f"gain V4L2 {v4l2_settings['gain']}")
                elif key == ord('G'):
                    v4l2_settings["gain"] = min(100, v4l2_settings.get("gain",0)+5)
                    if is_webcam:
                        apply_v4l2_settings(cap, {"gain": v4l2_settings["gain"]}, verbose=False)
                    print(f"gain V4L2 {v4l2_settings['gain']}")
                elif key == ord('e'):
                    v4l2_settings["exposure"] = max(1, v4l2_settings.get("exposure",50)-10)
                    if is_webcam:
                        apply_v4l2_settings(cap, {"exposure": v4l2_settings["exposure"]}, verbose=False)
                    print(f"exposure V4L2 {v4l2_settings['exposure']}")
                elif key == ord('E'):
                    v4l2_settings["exposure"] = min(500, v4l2_settings.get("exposure",50)+10)
                    if is_webcam:
                        apply_v4l2_settings(cap, {"exposure": v4l2_settings["exposure"]}, verbose=False)
                    print(f"exposure V4L2 {v4l2_settings['exposure']}")
                elif key == ord('h'):
                    sw_settings["sharpness"] = max(0.0, sw_settings["sharpness"] - 0.2)
                    print(f"sharpness SW {sw_settings['sharpness']:.2f}")
                elif key == ord('H'):
                    sw_settings["sharpness"] = min(4.0, sw_settings["sharpness"] + 0.2)
                    print(f"sharpness SW {sw_settings['sharpness']:.2f}")
                elif key == ord('r'):
                    # reset to current preset
                    sw_settings = dict(PRESETS[current_preset_name]["sw"])
                    v4l2_settings = dict(PRESETS[current_preset_name]["v4l2"])
                    if is_webcam:
                        apply_v4l2_settings(cap, v4l2_settings)
                    print(f"reset to {current_preset_name}")
                elif key == ord('d'):
                    sw_enabled = not sw_enabled
                    print(f"SW enhance {'ON' if sw_enabled else 'OFF'}")
                elif key == ord('t'):
                    tiled = not tiled
                    print(f"tiled {'ON' if tiled else 'OFF'}")
                elif key == ord('l'):
                    conf_thr = max(0.01, conf_thr - 0.05)
                    try:
                        det.conf_thr = conf_thr
                    except:
                        pass
                    print(f"conf {conf_thr:.2f}")
                elif key == ord('k'):
                    conf_thr = min(0.9, conf_thr + 0.05)
                    try:
                        det.conf_thr = conf_thr
                    except:
                        pass
                    print(f"conf {conf_thr:.2f}")
                elif key == ord('f'):
                    hide_fake = not hide_fake
                    if hide_fake:
                        show_fake = False
                    print(f">>> hide_fake={hide_fake} show_fake={show_fake} — fakes like grill 0.16/0.24 will {'hide' if hide_fake else 'show'}")
                elif key == ord('v'):
                    require_decode = not require_decode
                    if require_decode:
                        hide_fake = True
                        show_fake = False
                    print(f">>> require_decode={require_decode} — ONLY decoded QR shown, ALL fakes killed")
                elif key == ord(' ') or key == ord('s'):
                    fname = f"/tmp/qr_test_{int(time.time())}_{current_preset_name}.jpg"
                    if args.save:
                        fname = args.save
                    cv2.imwrite(fname, vis)
                    cv2.imwrite(fname.replace(".jpg","_raw.jpg"), raw_frame)
                    print(f"Saved {fname} and raw")
            else:
                if frame_n % 30 == 0:
                    print(f"frame {frame_n} raw={len(boxes)} filt={len(boxes_f)} {1000*(t1-t0):.0f}ms preset={current_preset_name} decoded={decoded_payloads}")
    finally:
        cap.release()
        if not args.no_show:
            cv2.destroyAllWindows()
        print(f"Done. Frames: {frame_n}, Unique payloads decoded: {decoded_payloads}")
        print(f"Final preset {current_preset_name} V4L2 {v4l2_settings} SW {sw_settings}")
        if decoded_payloads:
            print(f"Pi YOLO model WORKS on laptop webcam — will work in Gazebo too")
        else:
            print(f"No payload decoded — try:")
            print(f"  - Press p to cycle presets, 4=dark_qr_boost best for dark")
            print(f"  - Hold QR closer, good light, try --preset dark_qr_boost")
            print(f"  - Lower conf: press l or --conf 0.10")
            print(f"  - Try tiled: press t or --tiled")

if __name__ == "__main__":
    main()
