#!/usr/bin/env python3
"""
Camera tuning tool — easy control of brightness, ISO, exposure, contrast, etc.
For pitch dark conditions, use dark presets + LIVE preview with preset selection

Usage on Pi (or laptop with Pi reachable):
  # List current controls
  python3 tools/camera_tune.py --pi http://192.168.29.221:8000 --list

  # Dark preset — pitch dark last time
  python3 tools/camera_tune.py --pi http://192.168.29.221:8000 --preset dark
  python3 tools/camera_tune.py --pi http://192.168.29.221:8000 --preset night
  python3 tools/camera_tune.py --pi http://192.168.29.221:8000 --preset dark_qr_boost

  # Manual adjust
  python3 tools/camera_tune.py --pi http://192.168.29.221:8000 --cam cam2 --brightness 80 --gain 20 --exposure 30000
  python3 tools/camera_tune.py --pi http://192.168.29.221:8000 --cam cam1 --brightness 60 --contrast 1.5 --saturation 1.2

  # For Gazebo bridge (SITL)
  python3 tools/camera_tune.py --pi http://127.0.0.1:8099 --list
  # Gazebo bridge tuning via its args:
  # python3 tools/gz_cam_bridge.py --qr-boost --contrast 2.2 --brightness 1.3 --saturation 0.8 --sharpness 2.5

  # Local V4L2 webcam (laptop) — with LIVE preview and preset keys!
  python3 tools/camera_tune.py --v4l2 0 --list
  python3 tools/camera_tune.py --v4l2 0 --preview --preset dark_qr_boost
  python3 tools/camera_tune.py --v4l2 0 --preview  # then press keys 1-7 to switch preset live
  # Keys in preview: q=quit p=next preset 1-7=preset b/B brightness c/C contrast s/S sat g/G gain e/E exposure h/H sharp r=reset

Presets for dark:
  dark: brightness 80, gain 20dB, exposure 30000us, contrast 1.5, saturation 1.2 — for pitch dark
  night: brightness 100, gain 24dB, exposure 50000us, contrast 1.8, saturation 1.0 — for night
  dark_qr_boost: dark + qr_boost + ground_suppress — best for QR in dark
  daylight: brightness 0, gain 6dB, exposure 8333us, contrast 1.0 — normal
  qr_boost_day: daylight + qr_boost — QR pop in day
  qr_boost_night: dark + qr_boost — QR pop at night
"""

import argparse
import json
import sys
import os
import time

try:
    import requests
    _HAVE_REQUESTS = True
except:
    requests = None
    _HAVE_REQUESTS = False

# Presets — integrated workflow (same as qr_filter)
try:
    from qr_filter import PRESETS as FILTER_PRESETS, list_presets
    PRESETS = FILTER_PRESETS
    PRESET_ORDER = list_presets()
except Exception:
    PRESETS = {
        "daylight": {
            "exposure_us": 8333, "gain_db": 6.0, "brightness": 0.0, "contrast": 1.0, "saturation": 1.0, "sharpness": 1.0, "adaptive": True, "af_mode": "continuous", "qr_boost_enabled": False, "qr_software_enhance": True, "qr_enhance_mode": "none", "description": "Normal daylight — auto exposure, default", "v4l2": {"brightness": 0, "contrast": 45, "saturation": 63, "gain": 0, "exposure": 50, "sharpness": 100}, "sw": {"brightness": 1.0, "contrast": 1.0, "saturation": 1.0, "sharpness": 1.0},
        },
        "dark": {
            "exposure_us": 30000, "gain_db": 20.0, "brightness": 80.0, "contrast": 1.5, "saturation": 1.2, "sharpness": 1.3, "adaptive": False, "af_mode": "continuous", "qr_boost_enabled": False, "qr_software_enhance": True, "qr_enhance_mode": "none", "description": "Pitch dark — high exposure 30ms, high gain 20dB, brightness 80 — for dark you saw", "v4l2": {"brightness": 64, "contrast": 75, "saturation": 70, "gain": 20, "exposure": 300, "sharpness": 100}, "sw": {"brightness": 1.3, "contrast": 1.5, "saturation": 1.2, "sharpness": 1.3},
        },
        "night": {
            "exposure_us": 50000, "gain_db": 24.0, "brightness": 100.0, "contrast": 1.8, "saturation": 1.0, "sharpness": 1.5, "adaptive": False, "af_mode": "continuous", "qr_boost_enabled": False, "qr_software_enhance": True, "qr_enhance_mode": "none", "description": "Night — max exposure 50ms, max gain 24dB, brightness 100", "v4l2": {"brightness": 64, "contrast": 80, "saturation": 63, "gain": 40, "exposure": 400, "sharpness": 100}, "sw": {"brightness": 1.5, "contrast": 1.8, "saturation": 1.0, "sharpness": 1.5},
        },
        "dark_qr_boost": {
            "exposure_us": 30000, "gain_db": 20.0, "brightness": 80.0, "contrast": 1.8, "saturation": 0.6, "sharpness": 2.0, "adaptive": False, "af_mode": "continuous", "qr_boost_enabled": True, "qr_boost_mode": "bottom_qr_boost", "qr_software_enhance": True, "qr_enhance_mode": "ground_suppress", "description": "Dark + QR boost — best for QR in pitch dark, QR pops vs ground", "v4l2": {"brightness": 64, "contrast": 90, "saturation": 30, "gain": 20, "exposure": 300, "sharpness": 100}, "sw": {"brightness": 1.2, "contrast": 2.2, "saturation": 0.6, "sharpness": 2.5},
        },
        "qr_boost_day": {
            "exposure_us": 8333, "gain_db": 6.0, "brightness": 0.0, "contrast": 1.8, "saturation": 0.6, "sharpness": 2.0, "adaptive": True, "af_mode": "continuous", "qr_boost_enabled": True, "qr_boost_mode": "bottom_qr_boost", "qr_software_enhance": True, "qr_enhance_mode": "ground_suppress", "description": "Daylight QR boost — contrast high, sat low, sharp high, QR pops", "v4l2": {"brightness": 0, "contrast": 75, "saturation": 30, "gain": 0, "exposure": 50, "sharpness": 100}, "sw": {"brightness": 1.0, "contrast": 1.8, "saturation": 0.6, "sharpness": 2.0},
        },
        "qr_boost_night": {
            "exposure_us": 30000, "gain_db": 20.0, "brightness": 80.0, "contrast": 1.8, "saturation": 0.6, "sharpness": 2.0, "adaptive": False, "af_mode": "continuous", "qr_boost_enabled": True, "qr_boost_mode": "bottom_qr_boost", "qr_software_enhance": True, "qr_enhance_mode": "ground_suppress", "description": "Night QR boost — dark settings + QR boost, best for night QR", "v4l2": {"brightness": 64, "contrast": 90, "saturation": 30, "gain": 30, "exposure": 350, "sharpness": 100}, "sw": {"brightness": 1.3, "contrast": 2.0, "saturation": 0.6, "sharpness": 2.5},
        },
        "gazebo_dark": {
            "contrast": 2.2, "saturation": 0.8, "sharpness": 2.5, "brightness": 1.3, "qr_boost": True, "enhance_mode": "ground_suppress", "description": "Gazebo dark — for gz_cam_bridge.py --qr-boost --contrast 2.2 --brightness 1.3 --saturation 0.8 --sharpness 2.5", "v4l2": {"brightness": 64, "contrast": 85, "saturation": 50, "gain": 20, "exposure": 300, "sharpness": 100}, "sw": {"brightness": 1.3, "contrast": 2.2, "saturation": 0.8, "sharpness": 2.5},
        },
        "gazebo_bright": {
            "contrast": 1.0, "saturation": 1.0, "sharpness": 1.0, "brightness": 1.0, "qr_boost": False, "enhance_mode": "none", "description": "Gazebo normal", "v4l2": {"brightness": 0, "contrast": 45, "saturation": 63, "gain": 0, "exposure": 50, "sharpness": 100}, "sw": {"brightness": 1.0, "contrast": 1.0, "saturation": 1.0, "sharpness": 1.0},
        }
    }
    PRESET_ORDER = list(PRESETS.keys())

def pi_request(pi_url, method="GET", path="", json_data=None):
    if not _HAVE_REQUESTS:
        print("need requests: pip install requests")
        sys.exit(1)
    url = pi_url.rstrip("/") + path
    try:
        if method == "GET":
            r = requests.get(url, timeout=5)
        else:
            r = requests.post(url, json=json_data, timeout=5)
        r.raise_for_status()
        return r.json() if "application/json" in r.headers.get("Content-Type","") else r.text
    except Exception as e:
        print(f"Pi request failed {url}: {e}")
        return None

def list_pi(pi_url):
    print(f"Pi {pi_url} — listing cameras")
    data = pi_request(pi_url, "GET", "/api/cameras")
    if data:
        print(json.dumps(data, indent=2))
        for cam, st in data.items():
            print(f"\n{cam}: {st.get('model')} running={st.get('running')} frames={st.get('frames')}")
            print(f"  tuning: {st.get('tuning')}")
            print(f"  qr_boost: {st.get('qr_boost')}")
            print(f"  controls: {st.get('controls')}")
        return
    print(f"Failed /api/cameras, trying /health")
    data = pi_request(pi_url, "GET", "/health")
    if data:
        print(json.dumps(data, indent=2))
        if isinstance(data, dict) and ("front" in data or "bottom" in data):
            print(f"\nGazebo bridge health:")
            print(json.dumps(data, indent=2))
    else:
        print("No Pi or bridge reachable — for laptop webcam use --v4l2 0 --preview")

def tune_pi(pi_url, cam, preset_name=None, manual=None):
    if preset_name:
        if preset_name not in PRESETS:
            print(f"Unknown preset {preset_name}, available: {list(PRESETS.keys())}")
            sys.exit(1)
        preset = PRESETS[preset_name]
        desc = preset.get("description", "")
        print(f"Applying preset {preset_name}: {desc}")
        # Build payload from pi tuning + detector if present
        pi_part = preset.get("pi", {})
        # If preset is old flat style, use whole preset except v4l2/sw/detector
        if not pi_part and "v4l2" not in preset:
            pi_part = {k:v for k,v in preset.items() if k not in ("description","v4l2","sw","detector","presets")}
        # Merge detector conf if present for logging
        det_part = preset.get("detector", {})
        print(f"Pi tuning: {json.dumps(pi_part, indent=2)}")
        if det_part:
            print(f"Detector filter: {json.dumps(det_part, indent=2)}")
        payload = {"cam": cam}
        payload.update(pi_part)
        # Also try new /api/presets endpoint first (integrated workflow)
        result = pi_request(pi_url, "POST", f"/api/presets/{preset_name}", json_data={"cam": cam, "all": False})
        if result and result.get("ok"):
            print(f"Result via /api/presets: {json.dumps(result, indent=2)}")
        else:
            # Fallback to old /api/camera/controls
            result = pi_request(pi_url, "POST", "/api/camera/controls", json_data=payload)
            if result:
                print(f"Result via /api/camera/controls: {json.dumps(result, indent=2)}")
            else:
                print("Failed — Pi not reachable (expected if testing laptop webcam only)")
        # Also set fake filter if present
        if det_part:
            ff_res = pi_request(pi_url, "POST", "/api/detector/fake_filter", json_data=det_part)
            if ff_res:
                print(f"Fake filter set: {json.dumps(ff_res, indent=2)}")
    elif manual:
        print(f"Tuning {cam} manual: {manual}")
        payload = {"cam": cam}
        payload.update(manual)
        result = pi_request(pi_url, "POST", "/api/camera/controls", json_data=payload)
        if result:
            print(f"Result: {json.dumps(result, indent=2)}")

def tune_v4l2(device, list_only=False, manual=None, preset_name=None):
    try:
        import cv2
    except Exception as e:
        sys.exit(f"need opencv: {e}")
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        sys.exit(f"failed to open V4L2 {device}")
    props = {
        "brightness": cv2.CAP_PROP_BRIGHTNESS,
        "contrast": cv2.CAP_PROP_CONTRAST,
        "saturation": cv2.CAP_PROP_SATURATION,
        "hue": cv2.CAP_PROP_HUE,
        "gain": cv2.CAP_PROP_GAIN,
        "exposure": cv2.CAP_PROP_EXPOSURE,
        "sharpness": cv2.CAP_PROP_SHARPNESS,
    }
    print(f"V4L2 device {device} current controls:")
    for name, prop in props.items():
        try:
            v = cap.get(prop)
            print(f"  {name}: {v}")
        except:
            pass
    if list_only:
        cap.release()
        return
    if preset_name:
        preset = PRESETS.get(preset_name, {})
        v4l2_map = preset.get("v4l2", {})
        if not v4l2_map:
            # fallback mapping from old preset
            v4l2_map = {}
            if "brightness" in preset:
                v4l2_map["brightness"] = preset["brightness"]
            if "contrast" in preset:
                v4l2_map["contrast"] = preset["contrast"] * 50
            if "gain_db" in preset:
                v4l2_map["gain"] = preset["gain_db"]
            if "exposure_us" in preset:
                v4l2_map["exposure"] = preset["exposure_us"] / 100
        print(f"Applying preset {preset_name} V4L2: {v4l2_map}")
        for k,v in v4l2_map.items():
            prop = props.get(k)
            if prop is None:
                continue
            ok = cap.set(prop, float(v))
            actual = cap.get(prop)
            print(f"  {k}={v} set_ok={ok} actual={actual}")
    if manual:
        print(f"Setting manual: {manual}")
        for k,v in manual.items():
            prop = props.get(k)
            if prop is None:
                print(f"Unknown control {k}, available: {list(props.keys())}")
                continue
            ok = cap.set(prop, float(v))
            actual = cap.get(prop)
            print(f"  {k}={v} set_ok={ok} actual={actual}")
    cap.release()

def v4l2_preview(device, preset_name="daylight"):
    try:
        import cv2
        import numpy as np
    except Exception as e:
        sys.exit(f"need opencv: {e} pip install opencv-python-headless")

    preset_idx = PRESET_ORDER.index(preset_name) if preset_name in PRESET_ORDER else 0
    current_name = PRESET_ORDER[preset_idx]
    preset = PRESETS[current_name]
    v4l2_settings = dict(preset.get("v4l2", {}))
    sw_settings = dict(preset.get("sw", {}))

    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        sys.exit(f"failed to open V4L2 {device}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    props = {
        "brightness": cv2.CAP_PROP_BRIGHTNESS,
        "contrast": cv2.CAP_PROP_CONTRAST,
        "saturation": cv2.CAP_PROP_SATURATION,
        "gain": cv2.CAP_PROP_GAIN,
        "exposure": cv2.CAP_PROP_EXPOSURE,
        "sharpness": cv2.CAP_PROP_SHARPNESS,
    }

    def apply_v4l2():
        for k,v in v4l2_settings.items():
            prop = props.get(k)
            if prop:
                try:
                    cap.set(prop, float(v))
                except:
                    pass

    apply_v4l2()
    print(f"V4L2 Preview device {device} — preset {current_name}")
    print(f"Keys: q=quit p=next preset 1-7=preset b/B c/C s/S g/G e/E h/H r=reset space=save")
    print(f"  Current V4L2: {v4l2_settings} SW: {sw_settings}")

    def enhance(frame, sw):
        out = frame
        bri = sw.get("brightness",1.0)
        cont = sw.get("contrast",1.0)
        sat = sw.get("saturation",1.0)
        sharp = sw.get("sharpness",1.0)
        if bri!=1.0 or cont!=1.0:
            beta = (bri-1.0)*80
            out = cv2.convertScaleAbs(out, alpha=cont, beta=beta)
        if sat!=1.0:
            try:
                hsv = cv2.cvtColor(out, cv2.COLOR_BGR2HSV)
                h,s,v = cv2.split(hsv)
                s = np.clip(s.astype(np.float32)*sat,0,255).astype(np.uint8)
                out = cv2.cvtColor(cv2.merge((h,s,v)), cv2.COLOR_HSV2BGR)
            except:
                pass
        if sharp!=1.0 and sharp>1.0:
            try:
                blurred = cv2.GaussianBlur(out,(0,0),3)
                out = cv2.addWeighted(out, 1.0+(sharp-1.0), blurred, -(sharp-1.0), 0)
            except:
                pass
        return out

    sw_enabled = True
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue
            disp = enhance(frame, sw_settings) if sw_enabled else frame

            # overlay
            cv2.putText(disp, f"Preset [{preset_idx+1}/{len(PRESET_ORDER)}] {current_name}: {PRESETS[current_name]['description']}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,255,255), 2)
            cv2.putText(disp, f"V4L2 b={v4l2_settings.get('brightness',0):.0f} c={v4l2_settings.get('contrast',0):.0f} sat={v4l2_settings.get('saturation',0):.0f} g={v4l2_settings.get('gain',0):.0f} e={v4l2_settings.get('exposure',0):.0f} sh={v4l2_settings.get('sharpness',0):.0f} | SW bri={sw_settings.get('brightness',1):.2f} cont={sw_settings.get('contrast',1):.2f} sat={sw_settings.get('saturation',1):.2f} sharp={sw_settings.get('sharpness',1):.2f} {'ON' if sw_enabled else 'OFF'}", (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1)
            cv2.putText(disp, f"Keys: q=quit p=next 1-7=preset b/B bright c/C contrast s/S sat g/G gain e/E exp h/H sharp r=reset d=toggle SW space=save", (10, disp.shape[0]-15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200,200,200), 1)

            cv2.imshow(f"Camera Tune Preview - V4L2 {device} - PRESET CONTROL (press p or 1-7)", disp)
            key = cv2.waitKey(1) & 0xFF
            if key==255:
                continue
            if key==ord('q') or key==27:
                break
            elif key==ord('p') or key==ord('n'):
                preset_idx = (preset_idx+1)%len(PRESET_ORDER)
                current_name = PRESET_ORDER[preset_idx]
                v4l2_settings = dict(PRESETS[current_name].get("v4l2",{}))
                sw_settings = dict(PRESETS[current_name].get("sw",{}))
                apply_v4l2()
                print(f">>> Preset -> {current_name}")
            elif key==ord('o'):
                preset_idx = (preset_idx-1)%len(PRESET_ORDER)
                current_name = PRESET_ORDER[preset_idx]
                v4l2_settings = dict(PRESETS[current_name].get("v4l2",{}))
                sw_settings = dict(PRESETS[current_name].get("sw",{}))
                apply_v4l2()
                print(f">>> Preset -> {current_name}")
            elif ord('1') <= key <= ord('7'):
                idx = key-ord('1')
                if idx < len(PRESET_ORDER):
                    preset_idx = idx
                    current_name = PRESET_ORDER[preset_idx]
                    v4l2_settings = dict(PRESETS[current_name].get("v4l2",{}))
                    sw_settings = dict(PRESETS[current_name].get("sw",{}))
                    apply_v4l2()
                    print(f">>> Preset {idx+1} -> {current_name}")
            elif key==ord('b'):
                v4l2_settings["brightness"] = max(-64, v4l2_settings.get("brightness",0)-5)
                sw_settings["brightness"] = max(0.2, sw_settings.get("brightness",1)-0.1)
                apply_v4l2()
                print(f"brightness V4L2 {v4l2_settings['brightness']} SW {sw_settings['brightness']:.2f}")
            elif key==ord('B'):
                v4l2_settings["brightness"] = min(64, v4l2_settings.get("brightness",0)+5)
                sw_settings["brightness"] = min(3.0, sw_settings.get("brightness",1)+0.1)
                apply_v4l2()
                print(f"brightness V4L2 {v4l2_settings['brightness']} SW {sw_settings['brightness']:.2f}")
            elif key==ord('c'):
                v4l2_settings["contrast"] = max(0, v4l2_settings.get("contrast",45)-5)
                sw_settings["contrast"] = max(0.2, sw_settings.get("contrast",1)-0.2)
                apply_v4l2()
            elif key==ord('C'):
                v4l2_settings["contrast"] = min(100, v4l2_settings.get("contrast",45)+5)
                sw_settings["contrast"] = min(4.0, sw_settings.get("contrast",1)+0.2)
                apply_v4l2()
            elif key==ord('s'):
                v4l2_settings["saturation"] = max(0, v4l2_settings.get("saturation",63)-5)
                sw_settings["saturation"] = max(0.0, sw_settings.get("saturation",1)-0.2)
                apply_v4l2()
            elif key==ord('S'):
                v4l2_settings["saturation"] = min(100, v4l2_settings.get("saturation",63)+5)
                sw_settings["saturation"] = min(3.0, sw_settings.get("saturation",1)+0.2)
                apply_v4l2()
            elif key==ord('g'):
                v4l2_settings["gain"] = max(0, v4l2_settings.get("gain",0)-5)
                apply_v4l2()
            elif key==ord('G'):
                v4l2_settings["gain"] = min(100, v4l2_settings.get("gain",0)+5)
                apply_v4l2()
            elif key==ord('e'):
                v4l2_settings["exposure"] = max(1, v4l2_settings.get("exposure",50)-10)
                apply_v4l2()
            elif key==ord('E'):
                v4l2_settings["exposure"] = min(500, v4l2_settings.get("exposure",50)+10)
                apply_v4l2()
            elif key==ord('h'):
                sw_settings["sharpness"] = max(0.0, sw_settings.get("sharpness",1)-0.2)
            elif key==ord('H'):
                sw_settings["sharpness"] = min(4.0, sw_settings.get("sharpness",1)+0.2)
            elif key==ord('r'):
                v4l2_settings = dict(PRESETS[current_name].get("v4l2",{}))
                sw_settings = dict(PRESETS[current_name].get("sw",{}))
                apply_v4l2()
                print(f"reset to {current_name}")
            elif key==ord('d'):
                sw_enabled = not sw_enabled
                print(f"SW {'ON' if sw_enabled else 'OFF'}")
            elif key==ord(' ') or key==ord('w'):
                fname = f"/tmp/cam_tune_{current_name}_{int(time.time())}.jpg"
                cv2.imwrite(fname, disp)
                print(f"Saved {fname}")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print(f"Final: preset {current_name} V4L2 {v4l2_settings} SW {sw_settings}")
        print(f"To reuse: python3 tools/camera_tune.py --v4l2 {device} --preset {current_name}")
        print(f"Or manual: --v4l2 {device} --brightness {v4l2_settings.get('brightness')} --contrast {v4l2_settings.get('contrast')} --gain {v4l2_settings.get('gain')} --exposure {v4l2_settings.get('exposure')}")

def main():
    ap = argparse.ArgumentParser(description="Camera tuning — brightness, ISO, exposure, etc. for dark + LIVE preview")
    ap.add_argument("--pi", default="", help="Pi URL http://192.168.29.221:8000 or http://127.0.0.1:8000 or bridge http://127.0.0.1:8099")
    ap.add_argument("--cam", default="cam2", help="cam1 front or cam2 bottom")
    ap.add_argument("--list", action="store_true", help="list current controls")
    ap.add_argument("--preset", default="", help=f"preset: {', '.join(PRESETS.keys())}")
    ap.add_argument("--brightness", type=float, default=None, help="brightness -100..100 (Pi) or 0..255 (V4L2) or SW 0.5..2.0")
    ap.add_argument("--contrast", type=float, default=None, help="contrast 0..2 (Pi) or SW")
    ap.add_argument("--saturation", type=float, default=None, help="saturation 0..2")
    ap.add_argument("--sharpness", type=float, default=None, help="sharpness 0..2")
    ap.add_argument("--gain", type=float, default=None, help="gain dB 0..24 or V4L2 0..100")
    ap.add_argument("--exposure", type=float, default=None, help="exposure us 100..50000 or V4L2 1..500")
    ap.add_argument("--v4l2", type=int, default=None, help="V4L2 device index 0,1 for laptop webcam")
    ap.add_argument("--preview", action="store_true", help="LIVE preview window with preset selection (press p or 1-7) — for laptop webcam")
    args = ap.parse_args()

    manual = {}
    if args.brightness is not None:
        manual["brightness"] = args.brightness
    if args.contrast is not None:
        manual["contrast"] = args.contrast
    if args.saturation is not None:
        manual["saturation"] = args.saturation
    if args.sharpness is not None:
        manual["sharpness"] = args.sharpness
    if args.gain is not None:
        manual["gain_db"] = args.gain
        manual["gain"] = args.gain
    if args.exposure is not None:
        manual["exposure_us"] = args.exposure
        manual["exposure"] = args.exposure

    if args.v4l2 is not None:
        if args.preview:
            preset = args.preset if args.preset else "daylight"
            v4l2_preview(args.v4l2, preset_name=preset)
            return
        tune_v4l2(args.v4l2, list_only=args.list, manual=manual if not args.preset else None, preset_name=args.preset if args.preset else None)
        return

    if not args.pi:
        if args.preview:
            print("Preview needs --v4l2 0")
            sys.exit(1)
        print("Need --pi URL or --v4l2 device")
        print(f"Presets: {list(PRESETS.keys())}")
        for k,v in PRESETS.items():
            print(f"  {k}: {v['description']}")
        print("\nFor laptop webcam LIVE preview with preset selection:")
        print("  python3 tools/camera_tune.py --v4l2 0 --preview --preset dark_qr_boost")
        print("  Keys: q=quit p=next preset 1-7=direct b/B c/C s/S g/G e/E h/H r=reset")
        sys.exit(1)

    if args.list:
        list_pi(args.pi)
    elif args.preset:
        tune_pi(args.pi, args.cam, preset_name=args.preset)
    elif manual:
        tune_pi(args.pi, args.cam, manual=manual)
    else:
        print("Need --list or --preset or manual --brightness etc.")
        print(f"Available presets: {list(PRESETS.keys())}")

if __name__ == "__main__":
    main()
