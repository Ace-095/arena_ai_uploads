#!/usr/bin/env python3
"""real_pi camera auto-detect inspector.

Shows EXACTLY what this machine sees and how config.yaml would assign the
roles — the 30-second answer to "why is cam2 red?":

    python3 tools/list_cams.py                  # system + default config
    python3 tools/list_cams.py --config config.yaml

Exit 0 when every configured role gets a camera, 1 otherwise.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                os.pardir))


def parse_boot_config(text):
    """Pull the camera-relevant lines out of /boot/firmware/config.txt.

    Pure so it can be tested without a Pi: the field failures this explains
    ("csi: none found") are almost always `camera_auto_detect` still being 1,
    or no `dtoverlay=...imx477...` line at all — a third-party CSI module is
    invisible to libcamera until the overlay is declared by hand.
    """
    out = {"camera_auto_detect": None, "overlays": []}
    for raw in str(text or "").splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        key, _, val = line.partition("=")
        key, val = key.strip().lower(), val.strip()
        if key == "camera_auto_detect":
            out["camera_auto_detect"] = val
        elif key == "dtoverlay" and any(s in val.lower() for s in
                                        ("imx477", "imx708", "imx219",
                                         "ov5647", "vcm")):
            out["overlays"].append(val)
    return out


DTBO_PATHS = ("/boot/firmware/overlays/imx477.dtbo",
              "/boot/overlays/imx477.dtbo")


def dtbo_supports_vcm(path):
    """Does the INSTALLED imx477.dtbo declare a `vcm` parameter?

    The stock Raspberry Pi imx477 overlay does not (its params are rotation,
    orientation, media-controller, cam0, always-on, link-frequency, sync-*),
    so `dtoverlay=imx477,vcm` cannot apply with it — the boot log then shows
    "Failed to apply overlay" and the camera never appears. Arducam's package
    ships a patched dtbo that does have vcm. Parameter names are plain strings
    inside the dtb, so scanning the file is enough.

    Returns True / False, or None when the file cannot be read.
    """
    try:
        with open(path, "rb") as f:
            blob = f.read()
    except Exception:
        return None
    return b"vcm" in blob


def boot_verdict(cfg, dtbo_path=None):
    """Given parse_boot_config() output: (ok, [lines]) — what is still
    missing for a CSI camera to show up in libcamera.

    This is the check that would have answered "csi: none found" in one step:
    a third-party IMX477 module on a Pi 5 needs BOTH camera_auto_detect=0 and
    an explicit dtoverlay line, and ,vcm only applies if the Arducam dtbo (not
    the stock one) is installed.
    """
    lines = []
    ok = True
    cad = (cfg or {}).get("camera_auto_detect")
    overlays = [str(o).lower() for o in (cfg or {}).get("overlays") or []]

    if cad == "0":
        lines.append("OK   camera_auto_detect=0 (third-party modules need manual overlays)")
    else:
        ok = False
        lines.append("BAD  camera_auto_detect=%s — set it to 0; Pi 5 does not "
                     "auto-detect third-party CSI modules" % (cad or "unset (=1)"))

    if not overlays:
        ok = False
        lines.append("BAD  no camera dtoverlay declared — add under [all]:  "
                     "dtoverlay=imx477,vcm        (or ,vcm,cam0 for the CAM0 port)")
        return ok, lines

    for o in overlays:
        lines.append("OK   dtoverlay=%s" % o)

    if not any("imx477" in o for o in overlays):
        lines.append("note no imx477 overlay: an IMX477 module (?UC-517) will "
                     "stay invisible")

    wants_vcm = any("imx477" in o and "vcm" in o for o in overlays)
    if wants_vcm:
        path = dtbo_path or next((p for p in DTBO_PATHS if os.path.isfile(p)),
                                None)
        if path is None:
            ok = False
            lines.append("BAD  overlays/imx477.dtbo not found — install "
                         "Arducam's imx477_rpi_dtoverlay package first")
        else:
            has = dtbo_supports_vcm(path)
            if has is False:
                ok = False
                lines.append("BAD  the installed imx477.dtbo is the STOCK "
                             "overlay (no vcm) — ',vcm' cannot apply, so the "
                             "camera will not appear. Install Arducam's "
                             "package and re-check with: "
                             "strings %s | grep vcm" % path)
                lines.append("     (the stock dtbo otherwise works, but the "
                             "focus motor is then dead)")
            elif has is None:
                if not os.path.isfile(path):
                    ok = False
                    lines.append("BAD  %s not found — install Arducam's "
                                 "imx477_rpi_dtoverlay package first" % path)
                else:
                    lines.append("note could not read %s — check permissions"
                                 % path)
            else:
                lines.append("OK   installed imx477.dtbo supports vcm "
                             "(Arducam's overlay; focus motor can work)")

    if not any("imx708" in o for o in overlays):
        lines.append("note with auto-detect off, any OTHER CSI camera (e.g. "
                     "Pi Cam 3) also needs its own line: dtoverlay=imx708")
    return ok, lines


def print_boot_config():
    """Report the boot config + the two files the stack needs. Returns the
    parsed dict (empty when no config.txt exists, e.g. not a Pi)."""
    path = next((p for p in ("/boot/firmware/config.txt", "/boot/config.txt")
                 if os.path.isfile(p)), None)
    print("== boot config ==")
    if path is None:
        print("  no /boot/firmware/config.txt (not a Pi?) — skipping")
        return {}
    try:
        with open(path) as f:
            cfg = parse_boot_config(f.read())
    except Exception as e:
        print("  %s unreadable: %r" % (path, e))
        return {}
    print("  file: %s" % path)
    cad = cfg["camera_auto_detect"]
    print("  camera_auto_detect: %s%s" % (
        "unset (=1 default)" if cad is None else cad,
        "" if cad == "0" else
        "   <-- must be 0: auto-detect does NOT notice third-party CSI modules"
        " and hides the overlay you add below"))
    if cfg["overlays"]:
        for o in cfg["overlays"]:
            print("  dtoverlay=%s" % o)
        if not any("imx477" in o.lower() for o in cfg["overlays"]):
            print("  note: no imx477 overlay listed — an IMX477 module needs one")
    else:
        print("  dtoverlay=...       : NONE for any camera")
        print("      under [all]:  dtoverlay=imx477,vcm        (UC-517 / IMX477 + focus)")
        print("                    dtoverlay=imx708            (Pi Cam 3)")
        print("      add ,cam0 for a camera on the CAM0 connector")
    try:
        _ok, vlines = boot_verdict(cfg)
        print("  verdict:")
        for ln in vlines:
            print("    %s" % ln)
    except Exception as e:                       # never break the report
        print("  verdict: could not evaluate (%r)" % e)
    return cfg


def print_file_checks():
    """The two files whose ABSENCE silently degrades the stack. Checked
    independently of the boot config so they are reported on any machine
    (including a dev box with no /boot/firmware/config.txt at all)."""
    print("== files ==")
    seen = False
    for base in ("/usr/share/libcamera/ipa/rpi/pisp",
                 "/usr/share/libcamera/ipa/rpi/vc4"):
        if os.path.isdir(base):
            seen = True
            p = os.path.join(base, "imx477_af.json")
            print("  %s   %s" % (os.path.join(os.path.basename(base),
                                              "imx477_af.json"),
                                 "OK" if os.path.isfile(p) else
                                 "MISSING (UC-517 autofocus needs it)"))
    if not seen:
        print("  libcamera AF tuning dir: not present on this machine")
    hef = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       os.pardir, "models", "qr_yolov8n.hef")
    print("  models/qr_yolov8n.hef   %s" % (
        "OK" if os.path.isfile(hef) else
        "MISSING — Hailo runs need it; detector.kind auto uses ONNX on CPU"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.yaml")
    args = ap.parse_args()

    cfg = {}
    if os.path.isfile(args.config):
        try:
            import yaml
            with open(args.config) as f:
                cfg = yaml.safe_load(f) or {}
        except Exception as e:
            print("config %s unreadable: %r" % (args.config, e))
    else:
        print("config %s not found — showing raw hardware only" % args.config)

    boot_cfg = print_boot_config()
    boot_ok = False
    try:
        boot_ok = bool(boot_verdict(boot_cfg)[0]) if boot_cfg else False
    except Exception:
        boot_ok = False
    print_file_checks()

    print("== hardware ==")
    from cameras import (list_cameras, list_video_devices,
                         v4l2_capture_devices, _HAVE_PICAM)
    print("[csi] picamera2: %s" % ("OK" if _HAVE_PICAM else
                                    "MISSING (apt install python3-picamera2)"))
    csi = list_cameras()
    if csi:
        for i, model, info in csi:
            print("  csi idx %d: %s  (sensor=%s)" % (
                i, model, info.get("Sensor", "?")))
    else:
        print("  csi: none found")
    usb = v4l2_capture_devices()
    print("[usb] V4L2 capture nodes:")
    if usb:
        for d in usb:
            tag = "CAPTURE" if d["is_capture"] else "skip"
            print("  %-12s %s  vid:pid=%s  %s" % (
                d["node"], tag, d["vidpid"], d["usb_desc"] or d["name"]))
    else:
        print("  none (all /dev/video* nodes: %s)"
              % (list_video_devices() or "no /dev/video* at all"))

    print("== assignment (per %s) ==" % args.config)
    try:
        from cameras import CameraRig
        rig = CameraRig(cfg)
        assigned = rig.detect()
        if not assigned:
            print("  NO CAMERAS ASSIGNED — check hardware + config")
            return 1
        for name, c in sorted(assigned.items()):
            print("  %s (%s): %s  device=%s  size=%s" % (
                name, c.facing, c.model, c.device_desc or c.index,
                c.size))
        single = (cfg.get("cameras", {}) or {}).get("single_cam_role", "bottom")
        if len(assigned) == 1:
            print("  note: single camera -> treated as %r (cameras."
                  "single_cam_role)" % single)
        missing = {"front", "bottom"} - \
            {c.facing for c in assigned.values()}
        if missing and len(assigned) < 2 and boot_ok:
            # The boot config is right and libcamera STILL sees no camera —
            # only a physical cause is left. Say which, in order.
            print("  boot config is OK, so this is physical: reseat the FPC "
                  "cable (contacts the right way), try the other CAM "
                  "connector (then update the ,cam0 suffix), and check "
                  "`dmesg | grep -i imx477` after a reboot")
        if missing:
            print("  MISSING ROLE(S): %s" % ", ".join(sorted(missing)))
            print("  fix: plug the camera, or set cameras.%s.kind: usb "
                  "with device: /dev/videoN, or loosen match_name"
                  % sorted(missing)[0])
            return 1
        return 0
    except Exception as e:
        print("  assignment failed: %r" % e)
        return 1


if __name__ == "__main__":
    sys.exit(main())
