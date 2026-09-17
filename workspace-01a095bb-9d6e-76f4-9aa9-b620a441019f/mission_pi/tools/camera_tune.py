#!/usr/bin/env python3
"""
Camera tuning tool — easy control of brightness, ISO, exposure, contrast, etc.
For pitch dark conditions, use dark presets

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
  python3 tools/camera_tune.py --pi http://127.0.0.1:8099 --list  # bridge health
  # Gazebo bridge tuning via its args:
  # python3 tools/gz_cam_bridge.py --qr-boost --contrast 2.2 --brightness 1.3 --saturation 0.8 --sharpness 2.5

  # Local V4L2 webcam (laptop)
  python3 tools/camera_tune.py --v4l2 0 --list
  python3 tools/camera_tune.py --v4l2 0 --brightness 150 --contrast 40 --gain 20 --exposure 200

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

try:
    import requests
    _HAVE_REQUESTS = True
except:
    requests = None
    _HAVE_REQUESTS = False

# Presets for easy control — especially for dark
PRESETS = {
    "daylight": {
        "exposure_us": 8333,
        "gain_db": 6.0,
        "brightness": 0.0,
        "contrast": 1.0,
        "saturation": 1.0,
        "sharpness": 1.0,
        "adaptive": True,
        "af_mode": "continuous",
        "qr_boost_enabled": False,
        "qr_software_enhance": True,
        "qr_enhance_mode": "none",
        "description": "Normal daylight — auto exposure, default"
    },
    "dark": {
        "exposure_us": 30000,
        "gain_db": 20.0,
        "brightness": 80.0,
        "contrast": 1.5,
        "saturation": 1.2,
        "sharpness": 1.3,
        "adaptive": False,
        "af_mode": "continuous",
        "qr_boost_enabled": False,
        "qr_software_enhance": True,
        "qr_enhance_mode": "none",
        "description": "Pitch dark — high exposure 30ms, high gain 20dB, brightness 80 — for dark you saw"
    },
    "night": {
        "exposure_us": 50000,
        "gain_db": 24.0,
        "brightness": 100.0,
        "contrast": 1.8,
        "saturation": 1.0,
        "sharpness": 1.5,
        "adaptive": False,
        "af_mode": "continuous",
        "qr_boost_enabled": False,
        "qr_software_enhance": True,
        "qr_enhance_mode": "none",
        "description": "Night — max exposure 50ms, max gain 24dB, brightness 100"
    },
    "dark_qr_boost": {
        "exposure_us": 30000,
        "gain_db": 20.0,
        "brightness": 80.0,
        "contrast": 1.8,
        "saturation": 0.6,
        "sharpness": 2.0,
        "adaptive": False,
        "af_mode": "continuous",
        "qr_boost_enabled": True,
        "qr_boost_mode": "bottom_qr_boost",
        "qr_software_enhance": True,
        "qr_enhance_mode": "ground_suppress",
        "description": "Dark + QR boost — best for QR in pitch dark, QR pops vs ground"
    },
    "qr_boost_day": {
        "exposure_us": 8333,
        "gain_db": 6.0,
        "brightness": 0.0,
        "contrast": 1.8,
        "saturation": 0.6,
        "sharpness": 2.0,
        "adaptive": True,
        "af_mode": "continuous",
        "qr_boost_enabled": True,
        "qr_boost_mode": "bottom_qr_boost",
        "qr_software_enhance": True,
        "qr_enhance_mode": "ground_suppress",
        "description": "Daylight QR boost — contrast high, sat low, sharp high, QR pops"
    },
    "qr_boost_night": {
        "exposure_us": 30000,
        "gain_db": 20.0,
        "brightness": 80.0,
        "contrast": 1.8,
        "saturation": 0.6,
        "sharpness": 2.0,
        "adaptive": False,
        "af_mode": "continuous",
        "qr_boost_enabled": True,
        "qr_boost_mode": "bottom_qr_boost",
        "qr_software_enhance": True,
        "qr_enhance_mode": "ground_suppress",
        "description": "Night QR boost — dark settings + QR boost, best for night QR"
    },
    "gazebo_dark": {
        "contrast": 2.2,
        "saturation": 0.8,
        "sharpness": 2.5,
        "brightness": 1.3,
        "qr_boost": True,
        "enhance_mode": "ground_suppress",
        "description": "Gazebo dark — for gz_cam_bridge.py --qr-boost --contrast 2.2 --brightness 1.3 --saturation 0.8 --sharpness 2.5"
    },
    "gazebo_bright": {
        "contrast": 1.0,
        "saturation": 1.0,
        "sharpness": 1.0,
        "brightness": 1.0,
        "qr_boost": False,
        "enhance_mode": "none",
        "description": "Gazebo normal"
    }
}

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
        sys.exit(1)

def list_pi(pi_url):
    print(f"Pi {pi_url} — listing cameras")
    try:
        data = pi_request(pi_url, "GET", "/api/cameras")
        print(json.dumps(data, indent=2))
        for cam, st in data.items():
            print(f"\n{cam}: {st.get('model')} running={st.get('running')} frames={st.get('frames')}")
            print(f"  tuning: {st.get('tuning')}")
            print(f"  qr_boost: {st.get('qr_boost')}")
            print(f"  controls: {st.get('controls')}")
    except Exception as e:
        print(f"Failed /api/cameras, trying /health: {e}")
        data = pi_request(pi_url, "GET", "/health")
        print(json.dumps(data, indent=2))

    # Also try bridge health for Gazebo
    try:
        data = pi_request(pi_url, "GET", "/health")
        if "front" in data or "bottom" in data:
            print(f"\nGazebo bridge health:")
            print(json.dumps(data, indent=2))
    except:
        pass

def tune_pi(pi_url, cam, preset_name=None, manual=None):
    if preset_name:
        if preset_name not in PRESETS:
            print(f"Unknown preset {preset_name}, available: {list(PRESETS.keys())}")
            sys.exit(1)
        preset = PRESETS[preset_name]
        print(f"Applying preset {preset_name}: {preset['description']}")
        print(f"Values: {json.dumps(preset, indent=2)}")
        # For Pi, use /api/camera/controls
        payload = {"cam": cam}
        payload.update({k:v for k,v in preset.items() if k != "description"})
        # Map to contract
        # The server expects exposure_us, gain_db, etc.
        result = pi_request(pi_url, "POST", "/api/camera/controls", json_data=payload)
        print(f"Result: {json.dumps(result, indent=2)}")
    elif manual:
        print(f"Tuning {cam} manual: {manual}")
        payload = {"cam": cam}
        payload.update(manual)
        result = pi_request(pi_url, "POST", "/api/camera/controls", json_data=payload)
        print(f"Result: {json.dumps(result, indent=2)}")

def tune_v4l2(device, list_only=False, manual=None):
    try:
        import cv2
    except Exception as e:
        sys.exit(f"need opencv: {e}")
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        sys.exit(f"failed to open V4L2 {device}")
    # List controls
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

def main():
    ap = argparse.ArgumentParser(description="Camera tuning — brightness, ISO, exposure, etc. for dark")
    ap.add_argument("--pi", default="", help="Pi URL http://192.168.29.221:8000 or http://127.0.0.1:8000 or bridge http://127.0.0.1:8099")
    ap.add_argument("--cam", default="cam2", help="cam1 front or cam2 bottom")
    ap.add_argument("--list", action="store_true", help="list current controls")
    ap.add_argument("--preset", default="", help=f"preset: {', '.join(PRESETS.keys())}")
    ap.add_argument("--brightness", type=float, default=None, help="brightness -100..100 (DShow) or 0..100 (Pi) or 0..255 (V4L2)")
    ap.add_argument("--contrast", type=float, default=None, help="contrast 0..2")
    ap.add_argument("--saturation", type=float, default=None, help="saturation 0..2")
    ap.add_argument("--sharpness", type=float, default=None, help="sharpness 0..2")
    ap.add_argument("--gain", type=float, default=None, help="gain dB 0..24 or V4L2 0..100")
    ap.add_argument("--exposure", type=float, default=None, help="exposure us 100..50000")
    ap.add_argument("--v4l2", type=int, default=None, help="V4L2 device index 0,1 for laptop webcam")
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
        tune_v4l2(args.v4l2, list_only=args.list, manual=manual if not args.preset else None)
        if args.preset:
            print(f"Preset {args.preset} for V4L2: use manual values from preset")
            preset = PRESETS.get(args.preset, {})
            # Map preset to V4L2
            v4l2_map = {}
            if "brightness" in preset:
                v4l2_map["brightness"] = preset["brightness"]
            if "contrast" in preset:
                v4l2_map["contrast"] = preset["contrast"] * 50  # crude
            if "gain_db" in preset:
                v4l2_map["gain"] = preset["gain_db"]
            if "exposure_us" in preset:
                v4l2_map["exposure"] = preset["exposure_us"] / 100
            print(f"V4L2 mapped: {v4l2_map}")
            tune_v4l2(args.v4l2, manual=v4l2_map)
        return

    if not args.pi:
        print("Need --pi URL or --v4l2 device")
        print(f"Presets: {list(PRESETS.keys())}")
        for k,v in PRESETS.items():
            print(f"  {k}: {v['description']}")
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
