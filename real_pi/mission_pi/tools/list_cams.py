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
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                os.pardir))


def parse_boot_config(text):
    """Pull the camera-relevant lines out of /boot/firmware/config.txt.

    Pure so it can be tested without a Pi: the field failures this explains
    ("csi: none found") are almost always `camera_auto_detect` still being 1,
    or no `dtoverlay=...imx477...` line at all — a third-party CSI module is
    invisible to libcamera until the overlay is declared by hand.

    Section headers are tracked: a line under `[pi5]` only applies to a Pi 5,
    and a MALFORMED header (e.g. `[all],,`) is not recognised, so everything
    under it is ignored by the boot loader — both look like "the overlay is
    there" in a naive grep.
    """
    out = {"camera_auto_detect": None, "overlays": [], "overlays_detail": [],
           "sections": [], "malformed_sections": []}
    section = None
    for raw in str(text or "").splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        if line.startswith("["):
            end = line.find("]")
            name = line[1:end].strip().lower() if end > 0 else ""
            rest = line[end + 1:].strip() if end > 0 else ""
            if not name or rest or not re.match(r"^[a-z0-9_=.+-]+$", name):
                out["malformed_sections"].append(line)
            elif name not in out["sections"]:
                out["sections"].append(name)
            section = name or None
            continue
        key, _, val = line.partition("=")
        key, val = key.strip().lower(), val.strip()
        if key == "camera_auto_detect":
            out["camera_auto_detect"] = val
        elif key == "dtoverlay" and any(s in val.lower() for s in
                                        ("imx477", "imx708", "imx219",
                                         "ov5647", "vcm")):
            out["overlays"].append(val)
            out["overlays_detail"].append((val, section))
    return out


def board_section(model=None):
    """This board's config.txt section name ("pi5", "cm4", …) or None."""
    if model is None:
        for p in ("/proc/device-tree/model", "/sys/firmware/devicetree/base/model"):
            try:
                with open(p, "rb") as f:
                    model = f.read().decode("utf-8", "ignore").strip("\x00")
                break
            except Exception:
                continue
    m = str(model or "").lower()
    for needle, section in (("compute module 5", "cm5"), ("compute module 4", "cm4"),
                            ("pi 500", "pi500"), ("pi 400", "pi400"),
                            ("pi 5", "pi5"), ("pi 4", "pi4"), ("pi 3", "pi3"),
                            ("pi 2", "pi2"), ("zero 2", "zero2w"),
                            ("zero", "zero")):
        if needle in m:
            return section
    return None


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


def parse_dtoverlay_params(text):
    """Parameter names out of `dtoverlay -h <name>` output. Pure/testable.

    The utility prints the first param on the "Params:" line and the rest on
    indented continuation lines, each beginning with the param name. This is
    the authoritative check for "is Arducam's dtbo installed?" — plan
    `strings imx477.dtbo | grep -i vcm` is NOT: GNU strings only prints runs
    of 4+ characters by default, so the 3-character name "vcm" never appears
    even when it is there.
    """
    params = set()
    in_params = False
    for line in str(text or "").splitlines():
        s = line.rstrip()
        if s.strip().startswith("Params:"):
            in_params = True
            s = s.split("Params:", 1)[1]
        elif not in_params:
            continue
        s = s.strip()
        if not s or s.startswith(("Usage:", "Name:", "Info:")):
            continue
        params.add(s.split()[0].split("=")[0])
    return params


def dtoverlay_h_params(name):
    """Params of the INSTALLED overlay `name` via `dtoverlay -h` — a set, or
    None when the tool is unavailable or the overlay is unknown."""
    import shutil
    import subprocess
    exe = shutil.which("dtoverlay")
    if not exe:
        return None
    try:
        r = subprocess.run([exe, "-h", name], capture_output=True,
                           text=True, timeout=10)
    except Exception:
        return None
    out = (r.stdout or "") + (r.stderr or "")
    if "Params:" not in out:
        return None
    return parse_dtoverlay_params(out)


def boot_verdict(cfg, dtbo_path=None, board=None):
    """Given parse_boot_config() output: (ok, [lines]) — what is still
    missing for a CSI camera to show up in libcamera.

    This is the check that would have answered "csi: none found" in one step:
    a third-party IMX477 module on a Pi 5 needs BOTH camera_auto_detect=0 and
    an explicit dtoverlay line, in a section that APPLIES, and ,vcm only
    applies if the Arducam dtbo (not the stock one) is installed.
    """
    lines = []
    ok = True
    cfg = cfg or {}
    cad = cfg.get("camera_auto_detect")
    overlays = [str(o).lower() for o in cfg.get("overlays") or []]
    detail = cfg.get("overlays_detail") or [(o, None) for o in overlays]

    if cad == "0":
        lines.append("OK   camera_auto_detect=0 (third-party modules need manual overlays)")
    else:
        ok = False
        lines.append("BAD  camera_auto_detect=%s — set it to 0; Pi 5 does not "
                     "auto-detect third-party CSI modules" % (cad or "unset (=1)"))

    if cfg.get("malformed_sections"):
        ok = False
        for h in cfg["malformed_sections"]:
            lines.append("BAD  malformed section header %r — the boot loader "
                         "does not recognise it, so every line under it is "
                         "IGNORED. Fix it (usually to [all]) and move the "
                         "camera lines below something that applies." % h)

    if not overlays:
        ok = False
        lines.append("BAD  no camera dtoverlay declared — add under [all]:  "
                     "dtoverlay=imx477,vcm        (or ,vcm,cam0 for the CAM0 port)")
        return ok, lines

    # a line only takes effect if its section applies to THIS board
    this = board if board is not None else board_section()
    for value, section in detail:
        if section in (None, "all"):
            lines.append("OK   dtoverlay=%s%s" % (
                value, "" if section is None else "  ([all])"))
        elif this is not None and section == this:
            lines.append("OK   dtoverlay=%s  ([%s] — this board)" % (value, section))
        elif this is None:
            lines.append("note dtoverlay=%s sits under [%s] — make sure this "
                         "machine matches that section" % (value, section))
        else:
            ok = False
            lines.append("BAD  dtoverlay=%s sits under [%s] but this board is "
                         "[%s] — the line is skipped, so the camera stays "
                         "invisible. Move it to [all]." % (value, section, this))

    if not any("imx477" in o for o in overlays):
        lines.append("note no imx477 overlay: an IMX477 module (?UC-517) will "
                     "stay invisible")

    wants_vcm = any("imx477" in o and "vcm" in o for o in overlays)
    if wants_vcm:
        path = dtbo_path or next((p for p in DTBO_PATHS if os.path.isfile(p)),
                                None)
        # `dtoverlay -h` lists the parameters the boot loader will accept;
        # the byte scan says whether the FILE is Arducam's at all. They can
        # disagree (Arducam's newer overlays wire the lens unconditionally and
        # do not advertise a parameter), so report what each one actually
        # shows instead of collapsing them into one claim.
        names = dtoverlay_h_params("imx477")
        listed = None if names is None else ("vcm" in names)
        in_file = dtbo_supports_vcm(path) if path else None
        if listed is True:
            lines.append("OK   `dtoverlay -h imx477` offers a vcm parameter — "
                         "Arducam's overlay is installed and the focus motor "
                         "can be driven")
        elif in_file is True:
            lines.append("OK*  the installed imx477 overlay is not the stock "
                         "one (its file references vcm) but it does NOT "
                         "advertise a 'vcm' parameter — likely a newer "
                         "Arducam build that wires the lens unconditionally, "
                         "in which case ',vcm' in config.txt is harmless but "
                         "unnecessary. VERIFY the lens actually moves: see the "
                         "focus check in docs/CAMERA_AND_PIXHAWK_SETUP.md §1.3")
        elif in_file is False:
            ok = False
            lines.append("BAD  the installed overlay is the STOCK Raspberry Pi "
                         "imx477 one (no vcm at all) — the camera may stream "
                         "but the focus motor cannot be driven. Install "
                         "Arducam's imx477_rpi_dtoverlay package, then check: "
                         "dtoverlay -h imx477 | grep -i vcm")
        elif path is None or not os.path.isfile(path):
            ok = False
            lines.append("BAD  overlays/imx477.dtbo not found — install "
                         "Arducam's imx477_rpi_dtoverlay package first")
        else:
            lines.append("note could not read %s — check permissions" % path)

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
