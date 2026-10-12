#!/usr/bin/env python3
"""Field report: everything this machine can tell us about the real rig.

WHY THIS EXISTS
---------------
The machine that writes this code cannot reach a private LAN address, so the
hardware is only ever measurable from the machine itself. This tool collects
the answers in ONE run and writes ONE file, so whoever is at the aircraft
pastes a short summary back instead of transcribing ten commands.

READ-ONLY GUARANTEE - this tool observes and never actuates:
  * no config file is written (--config is read only)
  * no camera control is set (no VCM/focus position, no exposure change)
  * no MAVLink is sent unless you pass --with-fc, and even then it only
    connects, reads and disconnects (fc_probe.py --fence / --fence-check)
  * no model is converted, no HEF compiled, nothing pip-installed
It DOES open the camera and capture frames (timing a real capture is the whole
point), so STOP mission_pi FIRST: two processes cannot share a CSI camera, and
this would otherwise look like a camera fault.

USE ON THE PI
-------------
  cd ~/arena_ai_uploads/real_pi/mission_pi
  python3 tools/pi_field_report.py                        # read-only survey
  python3 tools/pi_field_report.py --with-fc               # + read the Pixhawk
  python3 tools/pi_field_report.py --dist-m 2.0 --panel-mm 297   # + MEASURE FOV
  python3 tools/pi_field_report.py --with-tests            # + run the suite

Then paste back the SUMMARY block it prints. The FOV step needs a tape measure
and a plain sheet: white A3 flat on a DARKER floor, directly under the lens,
centred in the picture, and the measured lens-to-sheet distance. That is the
one number every range calculation in this repo depends on and nobody has ever
measured - config.yaml still says VENDOR-QUOTED.
"""
import argparse
import json
import math
import os
import platform
import re
import signal
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.path.dirname(HERE)
sys.path.insert(0, PI)

#: ASCII only: this prints over ssh into terminals of unknown encoding.
OK = "[ok]"
NO = "[--]"
WARN = "[!!]"


def _child(code, timeout=120, cwd=None):
    """Run python in a CHILD process; a native crash must not take this one down.

    try/except catches exceptions. It does not catch SIGNALS, and a native
    library can die by signal. Measured on the real Pi: HailoYolo8Detector()
    died with SIGBUS inside the detector section, which killed the entire
    survey and wrote NO report at all - the worst possible outcome for a tool
    whose only job is to bring information back from a machine nobody can log
    into. Everything that loads a native accelerator therefore runs out here,
    where a crash is a result instead of an ending.

    Returns dict(rc, signal, stdout, stderr, timed_out).
    """
    prelude = "import sys; sys.path[:0] = [%r, %r]\n" % (PI, HERE)
    try:
        # -u is load-bearing. A child's stdout is a PIPE, so CPython block-buffers
        # it (8 KiB) and every print before a native crash is thrown away with the
        # process. Without this, a step that dies by SIGSEGV after printing five
        # progress markers is indistinguishable from one that died on its first
        # line - and "how far did it get" is the entire question.
        pr = subprocess.run([sys.executable, "-u", "-c", prelude + code],
                            cwd=cwd or PI, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, timeout=timeout)
        rc, sig = pr.returncode, None
        if rc is not None and rc < 0:
            try:
                sig = signal.Signals(-rc).name
            except Exception:
                sig = "signal %d" % -rc
        return {"rc": rc, "signal": sig,
                "stdout": pr.stdout.decode("utf-8", "replace"),
                "stderr": pr.stderr.decode("utf-8", "replace"),
                "timed_out": False}
    except subprocess.TimeoutExpired:
        return {"rc": None, "signal": None, "stdout": "", "stderr": "",
                "timed_out": True}
    except Exception as e:
        return {"rc": None, "signal": None, "stdout": "",
                "stderr": "%s: %s" % (type(e).__name__, e), "timed_out": False}


def _hailort_errors(text, limit=6):
    """The lines that actually name the cause. HailoRT writes them to stderr
    in the form `[HailoRT] [error] CHECK failed - ...`, and they are the whole
    diagnosis: page size, driver version, firmware version."""
    out = []
    for line in (text or "").splitlines():
        low = line.lower()
        if "[hailort]" in low or "error" in low or "failed" in low:
            line = line.strip()
            if line and line not in out:
                out.append(line)
        if len(out) >= limit:
            break
    return out


def _run(cmd, timeout=120, cwd=None):
    """(rc, stdout+stderr) - never raises."""
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout,
                           cwd=cwd or PI)
        return p.returncode, (p.stdout or "") + (p.stderr or "")
    except Exception as e:
        return -1, "could not run %s: %r" % (" ".join(cmd), e)


def _read(path):
    try:
        with open(path, "r", errors="replace") as fh:
            return fh.read().strip()
    except Exception:
        return None


def _cfg(path):
    try:
        import yaml
        with open(path) as fh:
            return yaml.safe_load(fh) or {}
    except Exception:
        return {}


class Frames(object):
    """Frames captured ONCE and shared by the fov/decode/inference sections.

    Three sections each opening a CSI camera was the first draft's mistake: it
    triples the chance of a device conflict and makes the capture timings
    describe three different warm-ups.
    """

    def __init__(self):
        self.by_role = {}
        self.error = None

    def add(self, role, frames):
        self.by_role.setdefault(role, [])
        self.by_role[role].extend(frames)

    def get(self, role):
        return self.by_role.get(role) or []


FRAMES = Frames()


def _open_camera(cfg, role):
    """(rig, cam) with the role's camera started, or (rig, None)."""
    import cameras as C
    rig = C.CameraRig(cfg)
    cams = rig.detect()
    cam = cams.get(role) or (list(cams.values())[0] if cams else None)
    if cam is None:
        return rig, None
    cam.start()
    return rig, cam


def _capture_into(role, cam, seconds=1.2, want=None):
    """Collect frames the way the mission does, and time each call."""
    want = want or 5
    frames, ms = [], []
    deadline = time.time() + seconds
    while len(frames) < want and time.time() < deadline:
        t0 = time.time()
        frame, ts, cnt = cam.latest()
        ms.append((time.time() - t0) * 1000.0)
        if frame is not None:
            frames.append(frame)
        else:
            time.sleep(0.05)
    FRAMES.add(role, frames)
    return frames, ms


# ---------------------------------------------------------------------------
# sections
# ---------------------------------------------------------------------------
def sec_system(args):
    out = {"hostname": socket.gethostname(), "kernel": platform.release(),
           "machine": platform.machine(), "python": platform.python_version(),
           "cpu_count": os.cpu_count()}
    model = _read("/proc/device-tree/model")
    if model:
        out["board"] = model.replace("\x00", "").strip()
    try:
        with open("/proc/uptime") as fh:
            out["uptime_s"] = float(fh.read().split()[0])
    except Exception:
        pass
    temp = _read("/sys/class/thermal/thermal_zone0/temp")
    if temp and temp.isdigit():
        out["cpu_temp_c"] = round(int(temp) / 1000.0, 1)
    m = re.search(r"MemTotal:\s+(\d+) kB", _read("/proc/meminfo") or "")
    if m:
        out["mem_total_mb"] = round(int(m.group(1)) / 1024.0)
    try:
        import psutil
        out["cpu_pct"] = psutil.cpu_percent(interval=0.5)
        out["mem_used_pct"] = psutil.virtual_memory().percent
        out["disk_free_gb"] = round(psutil.disk_usage("/").free / 1e9, 1)
        out["load_avg"] = list(os.getloadavg())
    except Exception:
        pass
    # A throttled Pi cannot hold a framerate, so this must come BEFORE any
    # timing number is believed.
    rc, txt = _run(["vcgencmd", "get_throttled"], timeout=10)
    if rc == 0 and "throttled=" in txt:
        val = int(txt.strip().split("=")[1], 16)
        out["throttled_raw"] = "0x%X" % val
        out["throttled_now"] = bool(val & 0x7)
        out["throttled_ever"] = bool(val & 0xF0000)
        out["throttled_bits"] = [n for bit, n in (
            (0x1, "under-voltage NOW"), (0x2, "arm freq capped NOW"),
            (0x4, "currently throttled"), (0x8, "soft temp limit NOW"),
            (0x10000, "under-voltage occurred"), (0x20000, "arm capped"),
            (0x40000, "throttled occurred"), (0x80000, "soft temp limit"))
            if val & bit]
    rc, txt = _run(["vcgencmd", "measure_clock", "arm"], timeout=10)
    if rc == 0:
        out["arm_clock"] = txt.strip()
    return out


def sec_deps(args):
    out = {}
    for name in ("numpy", "cv2", "yaml", "pymavlink", "psutil", "onnxruntime",
                 "onnx", "qrcode", "PIL", "fastapi", "uvicorn", "hailo_platform"):
        try:
            mod = __import__(name)
            out[name] = getattr(mod, "__version__", "present")
        except Exception as e:
            out[name] = "MISSING (%s)" % type(e).__name__
    return out


def sec_hailo(args):
    """Is the accelerator usable, or will the detector fall back to CPU?

    The brief is explicit: never silently fall back to CPU. So this reports
    both what the platform says and what the detector ladder would choose.
    """
    out = {}
    try:
        import hailo_platform
        out["hailo_platform"] = getattr(hailo_platform, "__version__", "present")
        try:
            out["hailo_devices"] = [str(d) for d in hailo_platform.Device.scan()]
        except Exception as e:
            out["hailo_devices_error"] = repr(e)
    except Exception as e:
        out["hailo_platform"] = "MISSING (%s: %s)" % (type(e).__name__, e)
    rc, txt = _run([sys.executable, os.path.join(HERE, "hef_info.py"),
                    "--list-device"], timeout=60)
    out["hef_info_rc"] = rc
    out["hef_info"] = txt.strip()[-2000:]
    for label, path in (("hef", os.path.join(PI, "models", "qr_yolov8n.hef")),
                        ("onnx", os.path.join(PI, "models", "qr_yolov8n.onnx"))):
        out["%s_present" % label] = os.path.exists(path)
        if os.path.exists(path):
            out["%s_bytes" % label] = os.path.getsize(path)
    return out


def sec_detector(args):
    """Which rung of the ladder get_detector() lands on, and why.

    Runs in a CHILD process. This is not tidiness - on the real Pi the in-process
    version died with SIGBUS while constructing the Hailo detector, and since a
    signal cannot be caught it took the whole survey with it and no report was
    ever written. A detector that cannot be built is now a reported result.
    """
    out = {"probe": "child process (a native crash in Hailo/ONNX must not kill "
                    "the survey)"}
    code = """
import json, os
cfg = __import__("yaml").safe_load(open(%r)) or {}
import detector as D
det = D.get_detector(cfg.get("detector", {}) or {})
info = {"chosen": getattr(det, "name", type(det).__name__)}
info["is_hailo"] = "hailo" in str(info["chosen"]).lower()
for attr in ("input_size", "tile_bottom", "tile_front", "conf", "conf_thr",
             "configure_api"):
    if hasattr(det, attr):
        info[attr] = getattr(det, attr)
print("PIJSON" + json.dumps(info))
""" % os.path.abspath(args.config)
    res = _child(code, timeout=180)
    if res["signal"]:
        out["crashed"] = True
        out["signal"] = res["signal"]
        out["error"] = ("the detector ladder died by %s. A native accelerator "
                        "killed the process; see hailo_steps for the step and "
                        "the HailoRT message" % res["signal"])
        out["hailort_errors"] = _hailort_errors(res["stderr"])
        out["stderr_tail"] = res["stderr"][-1200:]
    elif res["timed_out"]:
        out["crashed"] = True
        out["error"] = "timed out after 180s"
    else:
        marker = [l for l in res["stdout"].splitlines() if l.startswith("PIJSON")]
        if marker:
            try:
                out.update(json.loads(marker[-1][len("PIJSON"):]))
            except Exception as e:
                out["error"] = "could not parse the child's result: %s" % e
        else:
            out["error"] = "no result from the child (rc=%s)" % res["rc"]
        if res["rc"] not in (0, None) and "error" not in out:
            out["rc"] = res["rc"]
        if res["stderr"]:
            errs = _hailort_errors(res["stderr"])
            if errs:
                out["stderr_warnings"] = errs
    out["ladder_note"] = ("get_detector picks HEF -> ONNX -> .pt -> classical; "
                          "a name without 'hailo' means the accelerator was NOT "
                          "used, which is a result in itself")
    return out


def sec_hailo_steps(args):
    """Walk Hailo initialisation one step at a time, each in its own child.

    Written after the detector section took the whole survey down with SIGBUS:
    knowing only "it crashed" is not actionable. These are the exact steps
    HailoYolo8Detector.__init__ performs, in order, so the answer is a specific
    step name plus HailoRT's own error text - which names the cause (page size,
    driver version, firmware) and therefore the fix.
    """
    cfg = _cfg(args.config)
    hef = ((cfg.get("detector") or {}).get("hef_path") or "")
    hef_abs = os.path.join(PI, hef) if hef and not os.path.isabs(hef) else hef
    # The coarse step 5 ("build the detector") crashed on the real Pi with
    # SIGSEGV while its four predecessors passed, which narrows it to everything
    # AFTER VDevice() - about eight calls. So step 5 is now a ladder of its own:
    # the same calls HailoYolo8Detector.__init__ makes, in order, each one doing
    # strictly more than the last. The first to die is the culprit.
    pre = ('import hailo_platform as hp\n'
           'from hailo_platform import HEF, VDevice, InferVStreams, FormatType\n'
           'from hailo_decode import hailo_configure_params, hailo_vstream_params\n'
           'hef = HEF(%r)\n'
           'vdev = VDevice()\n' % hef_abs)
    steps = [
        ("1. import hailo_platform",
         'import hailo_platform as hp; print("version:", '
         'getattr(hp, "__version__", "?"))'),
        ("2. hailo_platform.Device.scan()",
         'import hailo_platform as hp; print("devices:", '
         '[str(d) for d in hp.Device.scan()])'),
        ("3. load the HEF file (hp.HEF)",
         'import hailo_platform as hp; hef = hp.HEF(%r); print("HEF ok:", '
         'type(hef).__name__)' % hef_abs),
        ("4. open the device (hp.VDevice)",
         'import hailo_platform as hp; v = hp.VDevice(); print("VDevice ok")'),
        # The label named hef.create_configure_params while the body called the
        # helper, so a report reading "CRASHED at: 5. hef.create_configure_params"
        # would have named a call that never ran. Label and body now agree: this
        # is the helper production uses, and it says which generation won.
        ("5. hailo_configure_params(hef, hp), the path production takes",
         pre + """
rep = []
cfg = hailo_configure_params(hef, hp, report=rep)
print("configure params ok:", type(cfg).__name__)
print("which generation won:", rep[0]["api"] if rep else "NOT RECORDED")
print("  and it returned a:", rep[0]["returns"] if rep else "?")
"""),
        # Hypothesis to test, not assume: hailo_configure_params() tries several
        # API generations and RETURNS THE FIRST THAT DOES NOT RAISE. On a runtime
        # where the modern call exists but rejects the argument, it silently falls
        # through to an older constructor whose params object the 4.2x
        # configure() then reads as the wrong type - which is a C++ crash, not a
        # Python one. This step reports every generation's availability and result
        # so that stops being a guess.
        ("5b. every configure-params API generation, probed on its own",
         pre + """
iface = None
for n in ("HailoStreamInterface", "StreamInterface"):
    iface = getattr(hp, n, None)
    if iface is not None:
        break
pcie = getattr(iface, "PCIe", "PCIe") if iface is not None else "PCIe"
print("interface class:", type(iface).__name__ if iface is not None else "NONE",
      "| PCIe =", pcie)
mk = getattr(hef, "create_configure_params", None)
print("hef.create_configure_params:", "present" if callable(mk) else "ABSENT")
if callable(mk):
    try:
        print("  -> ok:", type(mk(pcie)).__name__)
    except Exception as e:
        print("  -> RAISED: %s: %s" % (type(e).__name__, e))
cp = getattr(hp, "ConfigureParams", None)
print("hp.ConfigureParams:", "present" if cp is not None else "ABSENT")
if cp is not None:
    for label, fn in (
            ("create_from_hef(hef, interface=pcie)",
             lambda: cp.create_from_hef(hef, interface=pcie)),
            ("create_from_hef(hef)",
             lambda: cp.create_from_hef(hef))):
        try:
            print("  %s -> ok: %s" % (label, type(fn()).__name__))
        except Exception as e:
            print("  %s -> RAISED: %s: %s" % (label, type(e).__name__, e))
"""),
        ("6. vdev.configure(hef, cfg)",
         pre + 'cfg = hailo_configure_params(hef, hp)\n'
               'ng = vdev.configure(hef, cfg)[0]\n'
               'print("network group ok:", type(ng).__name__)'),
        ("7. network_group.create_params()",
         pre + 'cfg = hailo_configure_params(hef, hp)\n'
               'ng = vdev.configure(hef, cfg)[0]\n'
               'ngp = ng.create_params()\n'
               'print("params ok:", type(ngp).__name__)'),
        ("8. input/output vstream params",
         pre + 'cfg = hailo_configure_params(hef, hp)\n'
               'ng = vdev.configure(hef, cfg)[0]\n'
               'ngp = ng.create_params()\n'
               'ip = hailo_vstream_params(hp, ng, ngp, True, FormatType.UINT8)\n'
               'op = hailo_vstream_params(hp, ng, ngp, False, FormatType.FLOAT32)\n'
               'print("vstream params ok:", len(ip), "in,", len(op), "out")'),
        ("9. InferVStreams(ng, in, out).__enter__()",
         pre + 'cfg = hailo_configure_params(hef, hp)\n'
               'ng = vdev.configure(hef, cfg)[0]\n'
               'ngp = ng.create_params()\n'
               'ip = hailo_vstream_params(hp, ng, ngp, True, FormatType.UINT8)\n'
               'op = hailo_vstream_params(hp, ng, ngp, False, FormatType.FLOAT32)\n'
               'st = InferVStreams(ng, ip, op)\n'
               'st.__enter__()\n'
               'print("InferVStreams entered")\n'
               'st.__exit__(None, None, None)\n'
               'print("InferVStreams exited cleanly")'),
        # The question the crash report could not answer: does Hailo actually
        # INFER, or does it die before the first frame? Markers are printed as it
        # goes and the child runs unbuffered, so a crash leaves the last marker
        # behind - "A/B/C" before dying means the streams never came up, "E" means
        # it ran an inference and the crash came afterwards, at teardown.
        ("9b. one real inference through the entered InferVStreams",
         pre + 'cfg = hailo_configure_params(hef, hp)\n'
               'ng = vdev.configure(hef, cfg)[0]\n'
               'ngp = ng.create_params()\n'
               'ip = hailo_vstream_params(hp, ng, ngp, True, FormatType.UINT8)\n'
               'op = hailo_vstream_params(hp, ng, ngp, False, FormatType.FLOAT32)\n'
               'import numpy as np\n'
               'ii = hef.get_input_vstream_infos()[0]\n'
               'print("A: building InferVStreams", flush=True)\n'
               'st = InferVStreams(ng, ip, op)\n'
               'st.__enter__()\n'
               'print("B: streams entered", flush=True)\n'
               'act = getattr(ng, "activate", None)\n'
               'act_cm = act(ngp) if callable(act) else None\n'
               'if act_cm is not None:\n'
               '    act_cm.__enter__()\n'
               'print("C: activated", flush=True)\n'
               'frame = np.zeros((1,) + tuple(ii.shape), dtype=np.uint8)\n'
               'res = st.infer({ii.name: frame})\n'
               'print("D: infer returned", flush=True)\n'
               'names = sorted(res.keys()) if hasattr(res, "keys") else res\n'
               'print("   outputs:", names, flush=True)\n'
               'if hasattr(res, "keys"):\n'
               '    for k in sorted(res.keys()):\n'
               '        a = np.asarray(res[k])\n'
               '        print("   %s %s %s" % (k, a.shape, a.dtype), flush=True)\n'
               'print("E: done - inference works", flush=True)\n'
               'if act_cm is not None:\n'
               '    act_cm.__exit__(None, None, None)\n'
               'st.__exit__(None, None, None)\n'
               'print("F: closed cleanly", flush=True)'),
        ("10. ng.activate(ngp).__enter__()",
         pre + 'cfg = hailo_configure_params(hef, hp)\n'
               'ng = vdev.configure(hef, cfg)[0]\n'
               'ngp = ng.create_params()\n'
               'act = getattr(ng, "activate", None)\n'
               'if callable(act):\n'
               '    act(ngp).__enter__()\n'
               '    print("activate entered")\n'
               'else:\n'
               '    print("no activate() on this runtime")'),
        ("11. hef input/output vstream infos",
         pre + 'print("inputs:", [i.name for i in hef.get_input_vstream_infos()],\n'
               '      "outputs:", len(hef.get_output_vstream_infos()))'),
        ("12. the whole constructor, as the ladder builds it",
         'import json, os, yaml, detector as D;'
         ' cfg = yaml.safe_load(open(%r)) or {};'
         ' det = D.get_detector(cfg.get("detector", {}) or {});'
         ' print("chosen:", getattr(det, "name", type(det).__name__))'
         % os.path.abspath(args.config)),
        # The lifecycle main.py actually lives: get_detector(), work, then
        # det.close() from the finally at main.py:724. Step 12 abandons the
        # detector at process exit instead - a different question, kept, but it
        # must not be mistaken for what the mission does.
        ("12b. the whole constructor, then close it the way main.py does",
         'import json, os, yaml, detector as D;'
         ' cfg = yaml.safe_load(open(%r)) or {};'
         ' det = D.get_detector(cfg.get("detector", {}) or {});'
         ' print("chosen:", getattr(det, "name", type(det).__name__));'
         ' det.close();'
         ' print("closed cleanly")'
         % os.path.abspath(args.config)),
    ]
    out = {"steps": [], "hef_path": hef_abs or None,
           "why": "each step runs alone in a child process, so the first "
                  "CRASHED step is the culprit and the rest still get probed",
           "caveat": "%d processes open the device one after another; that is "
                     "safe but it is not a test of running them together"
                     % len(steps)}
    if not hef_abs or not os.path.exists(hef_abs):
        out["note"] = ("no HEF at %r - the ladder cannot choose Hailo at all. "
                       "Compile one (training/compile_hef.md) before reading "
                       "anything else here" % (hef or "detector.hef_path"))
    # The constructor-family steps run get_detector(), which can exit 0 by
    # falling back to the CPU. 12b follows 12 now, so "the last step" stopped
    # being a definition of the constructor - name it instead.
    ctor_name = next((lab for lab, _ in steps if lab.startswith("12. ")), None)
    out["ctor_step"] = ctor_name
    for name, code in steps:
        r = _child(code, timeout=90)
        rec = {"step": name}
        if r["timed_out"]:
            rec["timed_out"] = True
        elif r["signal"]:
            rec["signal"] = r["signal"]
            rec["hailort_errors"] = _hailort_errors(r["stderr"])
            rec["stderr_tail"] = r["stderr"][-600:]
            # What it managed to print says whether the crash was AT the call in
            # the label or AFTER it - e.g. a surviving "streams entered" means the
            # crash is in the teardown, not in entering the streams.
            rec["stdout_tail"] = r["stdout"].strip()[-600:]
        elif r["rc"] == 0:
            rec["ok"] = True
            rec["stdout"] = r["stdout"].strip()[-300:]
            errs = _hailort_errors(r["stderr"])
            if errs:
                rec["warnings"] = errs          # warnings can precede a crash
        else:
            rec["rc"] = r["rc"]
            rec["stdout_tail"] = r["stdout"].strip()[-600:]
            rec["error"] = (r["stderr"] or r["stdout"]).strip().splitlines()[-1:] 
            rec["error"] = rec["error"][0] if rec["error"] else "failed"
            errs = _hailort_errors(r["stderr"])
            if errs:
                rec["hailort_errors"] = errs
        out["steps"].append(rec)
    # Three outcomes, not two. The first version only looked for signals, so a
    # machine with NO hailo_platform at all reported "all 5 init steps
    # completed" while four of them had failed - the same "verdict that does not
    # match reality" mistake this branch keeps having to fix.
    crashed = [r["step"] for r in out["steps"] if r.get("signal")]
    failed = [r["step"] for r in out["steps"] if not r.get("ok")]
    out["first_crash"] = crashed[0] if crashed else None
    out["first_failure"] = failed[0] if failed else None
    if crashed:
        out["verdict"] = "CRASHED at: %s" % crashed[0]
    elif failed:
        out["verdict"] = "FAILED at: %s" % failed[0]
    else:
        out["verdict"] = "ok - every step completed"
    return out


def sec_cameras(args):
    out = {}
    rc, txt = _run([sys.executable, os.path.join(HERE, "list_cams.py"),
                    "--config", args.config], timeout=60)
    out["list_cams_rc"] = rc
    out["list_cams"] = txt.strip()[-3000:]
    out["dev_video"] = sorted(p for p in os.listdir("/dev") if p.startswith("video"))
    rc, txt = _run(["v4l2-ctl", "--list-devices"], timeout=30)
    if rc == 0:
        out["v4l2_devices"] = txt.strip()[:1500]
        out["v4l2_note"] = "v4l2-ctl present"
    else:
        out["v4l2_note"] = "v4l2-ctl NOT installed (sudo apt install v4l-utils)"
    if args.no_capture:
        out["capture"] = "skipped (--no-capture)"
        return out
    cap = {"per_role": {}}
    cfg = _cfg(args.config)
    try:
        rig, cam = _open_camera(cfg, args.role)
    except Exception as e:
        out["capture"] = {"error": "could not open the camera stack: %s: %s"
                                   % (type(e).__name__, e)}
        return out
    try:
        cap["detected"] = {n: {"kind": getattr(c, "kind", None),
                               "index": getattr(c, "index", None),
                               "model": getattr(c, "model", None),
                               "facing": getattr(c, "facing", None)}
                           for n, c in (rig.cams or {}).items()}
        if cam is None:
            cap["error"] = ("no camera detected for role %r - see list_cams "
                            "above" % args.role)
            out["capture"] = cap
            return out
        frames, ms = _capture_into(args.role, cam, want=args.samples)
        ms_sorted = sorted(ms)
        entry = {
            "started": True,
            "requested_size": list(cam.size) if hasattr(cam, "size") else None,
            "frames": [list(f.shape) for f in frames],
            "latest_ms_median": round(ms_sorted[len(ms_sorted) // 2], 1) if ms else None,
            "latest_ms_max": round(max(ms), 1) if ms else None,
            "hfov_deg_config": getattr(cam, "hfov_deg", None),
            "vfov_deg_config": getattr(cam, "vfov_deg", None),
            "kind": getattr(cam, "kind", None),
        }
        if frames and entry["requested_size"]:
            got = list(frames[0].shape[:2][::-1])
            if got != list(entry["requested_size"]):
                entry["size_warning"] = ("the sensor produced %s while %s was "
                                         "requested - some backends resize, and "
                                         "a resized frame adds no optical detail"
                                         % (got, entry["requested_size"]))
        cap["per_role"][args.role] = entry
    except Exception as e:
        cap["per_role"][args.role] = {"error": "%s: %s" % (type(e).__name__, e)}
    finally:
        try:
            cam.stop()
        except Exception:
            pass
        try:
            rig.stop_all()
        except Exception:
            pass
    out["capture"] = cap
    return out


def _sheet_rect(frame):
    """(long_px, short_px, long_is_horizontal) of the bright centred sheet.

    Returns None when there is no sheet to measure. Design notes, all from
    measuring this on synthetic frames rather than trusting the first version:

    * The FIRST version used a brightness percentile and was wrong twice over:
      on a flat frame the percentile is the frame's own value, so the whole
      picture became "the sheet" (measured: 640 px wide), and around a real
      sheet the threshold crept into the ground's upper tail, overestimating
      the sheet by ~6% - which biased every FOV it reported about 5 degrees
      LOW. Reporting a wrong number as MEASURED is the one thing this tool must
      not do, so the percentile is gone.
    * Otsu instead, plus three guards: the sheet must be genuinely brighter than
      the frame median, must be a sane fraction of the picture (not the whole
      frame, not a speck), and must be centred, because distortion is worst at
      the edges and this measures the CENTRE field of view.
    * minAreaRect gives the rectangle's true sides under a few degrees of
      yaw/roll, and the rect's own orientation decides which image axis the
      long side runs along - so a sheet rotated in view is handled without the
      operator having to say anything.
    """
    try:
        import cv2
        import numpy as np
    except Exception:
        return None
    gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape[:2]
    blur = cv2.GaussianBlur(gray, (7, 7), 0)
    otsu_t, mask = cv2.threshold(blur, 0, 255,
                                 cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    frame_med = float(np.median(blur))
    # No bright object at all -> nothing white to measure. A uniformly textured
    # floor must not qualify, however its brightness happens to be distributed.
    if otsu_t < max(60.0, frame_med * 1.15):
        return None
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            np.ones((5, 5), np.uint8))
    res = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = res[0] if len(res) == 2 else res[1]
    best = None
    for c in cnts:
        area = cv2.contourArea(c)
        frac = area / float(w * h)
        # Floor lowered from 2% to 0.5%: measured, at 2% a legitimate sheet
        # (A3 long side at 2 m on a 75 deg lens = 277x196 px = 1.8%) was
        # REJECTED outright. Ceiling stays at 60%: above that there is no
        # contrast and a FOV from it would be meaningless.
        if frac < 0.005 or frac > 0.60:
            # > 60% means the operator's setup has no contrast (or the sheet
            # fills the view); either way a FOV from it would be meaningless
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        if bw < 20 or bh < 20:
            continue
        if x <= 1 or y <= 1 or x + bw >= w - 1 or y + bh >= h - 1:
            continue                      # clipped by the frame -> unusable
        if not (abs((x + bw / 2.0) - w / 2.0) < 0.35 * w
                and abs((y + bh / 2.0) - h / 2.0) < 0.35 * h):
            continue                      # not centred -> distortion zone
        (cx, cy), (rw, rh), ang = cv2.minAreaRect(c)
        long_px, short_px = max(rw, rh), min(rw, rh)
        if short_px < 15:
            continue
        if short_px / max(1.0, long_px) < 0.30:
            continue                      # a streak, not a sheet
        # WHICH IMAGE AXIS the long side runs along CANNOT come from
        # minAreaRect's (rw, rh): those are measured in the rect's OWN rotated
        # frame, so `rw >= rh` is not an image-axis statement. Measured: that
        # mistake made a landscape sheet report "vertical" and skewed the
        # recovered FOV by -15 deg. The axis-aligned bounding box IS in image
        # coordinates, so it decides the axis; minAreaRect still supplies the
        # more accurate side lengths. A sheet rotated far from square-on is
        # refused rather than guessed at.
        if abs(bw - bh) > 0 and min(bw, bh) / max(1.0, max(bw, bh)) > 0.98:
            # near-square: the axis is ambiguous, but so is the sheet's
            # orientation - accept and treat the longer rect side as the long one
            pass
        # rotation guard: the long edge must be close to an image axis
        pts = cv2.boxPoints(((cx, cy), (rw, rh), ang))
        edges = [(pts[(i + 1) % 4][0] - pts[i][0], pts[(i + 1) % 4][1] - pts[i][1])
                 for i in range(4)]
        E = [(abs(ex), abs(ey), (ex * ex + ey * ey) ** 0.5) for ex, ey in edges]
        long_edge = max(E, key=lambda e: e[2])
        if long_edge[2] > 0:
            skew_deg = math.degrees(math.atan2(long_edge[1], long_edge[0]))
            skew = min(abs(skew_deg), abs(90.0 - abs(skew_deg)))
            if skew > 25.0:
                continue                  # rotated too far to attribute an axis
        # the sheet's own pixels must really be brighter than the floor
        sub = gray[int(max(0, cy - rh / 2)):int(min(h, cy + rh / 2)),
                   int(max(0, cx - rw / 2)):int(min(w, cx + rw / 2))]
        if sub.size and float(sub.mean()) < frame_med * 1.25:
            continue
        if best is None or area > best[0]:
            best = (area, long_px, short_px, bw >= bh)
    if best is None:
        return None
    return best[1], best[2], best[3]


def sec_fov(args):
    """Measure the real HFOV/VFOV from a sheet of known size.

    Which image axis the sheet's long side runs along decides the formula, and
    that is read from the MEASUREMENT, not from what the operator assumed.
    """
    if not args.dist_m or not args.panel_mm:
        return {"measured": False,
                "why": "pass --dist-m <lens to sheet, metres> and --panel-mm "
                       "<sheet LONG side, mm> to turn the vendor FOV into a "
                       "measured one"}
    out = {"measured": False, "dist_m": args.dist_m,
           "panel_long_mm": args.panel_mm, "panel_short_mm": args.panel_mm_h or None,
           "role": args.role}
    try:
        import numpy as np
        import cv2
    except Exception as e:
        out["error"] = "needs numpy + cv2: %r" % (e,)
        return out
    frames = FRAMES.get(args.role)
    if not frames:
        try:
            cfg = _cfg(args.config)
            rig, cam = _open_camera(cfg, args.role)
            try:
                if cam is not None:
                    _capture_into(args.role, cam, want=max(3, args.samples))
                    frames = FRAMES.get(args.role)
            finally:
                try:
                    cam.stop()
                except Exception:
                    pass
                try:
                    rig.stop_all()
                except Exception:
                    pass
        except Exception as e:
            out["error"] = "no frames and could not capture: %s: %s" % (
                type(e).__name__, e)
            return out
    rects, frame0 = [], None
    for f in frames:
        r = _sheet_rect(f)
        if r:
            rects.append(r)
            if frame0 is None:
                frame0 = f
    if not rects or frame0 is None:
        out["error"] = ("no sheet found - a WHITE sheet on a DARKER floor, "
                        "directly under the lens, centred, not touching the "
                        "edges of the picture, and clearly brighter than the "
                        "ground around it")
        out["frames_tried"] = len(frames)
        return out
    longs = sorted(r[0] for r in rects)
    shorts = sorted(r[1] for r in rects)
    med_long, med_short = longs[len(longs) // 2], shorts[len(shorts) // 2]
    horizontal = sum(1 for r in rects if r[2]) >= (len(rects) + 1) // 2
    h, w = frame0.shape[:2]
    long_m = args.panel_mm / 1000.0
    # the long side spans `med_long` px along the axis it runs on
    span_px = float(w if horizontal else h)
    ground_long_m = long_m * span_px / med_long
    hfov = float(np.degrees(2.0 * np.arctan(
        (ground_long_m / 2.0) / args.dist_m)))
    out.update({
        "measured": True,
        "frame_size": [int(w), int(h)],
        "sheet_px_samples": [[int(a), int(b)] for a, b in
                             sorted((r[0], r[1]) for r in rects)],
        "sheet_long_px_median": int(med_long),
        "sheet_short_px_median": int(med_short),
        "sheet_px_spread_pct": round(100.0 * (longs[-1] - longs[0]) / max(1.0, med_long), 2),
        "long_side_runs": "horizontal (left-right)" if horizontal else "vertical",
        "mm_per_px_on_ground": round(ground_long_m * 1000.0 / span_px, 3),
        "hfov_deg": round(hfov, 2),
        "hfov_deg_config": (_cfg(args.config).get("cameras", {})
                            .get(args.role, {}) or {}).get("hfov_deg"),
        "caveats": ["single centred sheet: this measures the CENTRE field of "
                    "view; corner distortion is NOT measured",
                    "assumes the sheet is flat and perpendicular to the lens",
                    "the distance is whatever the operator measured with a tape",
                    "a bench measurement, NOT a flight result"],
    })
    if args.panel_mm_h:
        short_m = args.panel_mm_h / 1000.0
        # the short side spans the OTHER axis
        other_px = float(h if horizontal else w)
        ground_short_m = short_m * other_px / med_short
        out["vfov_deg"] = round(float(np.degrees(2.0 * np.arctan(
            (ground_short_m / 2.0) / args.dist_m))), 2)
        out["vfov_deg_config"] = (_cfg(args.config).get("cameras", {})
                                  .get(args.role, {}) or {}).get("vfov_deg")
        declared = long_m / short_m
        measured = med_long / max(1.0, med_short)
        out["aspect_check"] = {
            "declared_long_over_short": round(declared, 3),
            "measured_long_over_short": round(float(measured), 3),
            "agree_within_10pct": abs(measured - declared) / declared < 0.10,
        }
        if not out["aspect_check"]["agree_within_10pct"]:
            out["caveats"].append(
                "measured aspect disagrees with the declared sheet by >10%%: "
                "either a side is partly out of view or the floor is not "
                "perpendicular - treat both angles as suspect")
    try:
        cv2.imwrite(os.path.join(args.out_dir, "fov_%s.png" % args.role), frame0)
        out["frame_saved"] = os.path.join(args.out_dir, "fov_%s.png" % args.role)
    except Exception:
        pass
    return out


def sec_decode(args):
    """Does the REAL camera + the configured decoder actually decode?"""
    out = {"attempted": False}
    if args.no_decode:
        out["why"] = "skipped (--no-decode)"
        return out
    frames = FRAMES.get(args.role)
    if not frames:
        out["why"] = ("no captured frames - run without --no-capture, or pass "
                      "--dist-m/--panel-mm so a capture is requested")
        return out
    try:
        import decoder as DEC
    except Exception as e:
        out["error"] = "decoder import failed: %r" % (e,)
        return out
    try:
        out["attempted"] = True
        payloads, times = [], []
        for f in frames:
            t0 = time.time()
            p = DEC.decode_frame(f)
            times.append((time.time() - t0) * 1000.0)
            if p:
                payloads.append(p)
        out["frames"] = len(frames)
        out["decoded"] = bool(payloads)
        out["payloads"] = payloads[:3]
        out["decode_ms_median"] = round(sorted(times)[len(times) // 2], 1)
        px = _sheet_px(frames[0])
        out["sheet_px"] = int(px)
        if px and args.qr_modules:
            out["px_per_module_measured"] = round(px / float(args.qr_modules), 2)
        out["note"] = ("full-frame decode, no ROI hint: the optimistic case")
    except Exception as e:
        out["error"] = "%s: %s" % (type(e).__name__, e)
    return out


def sec_inference(args):
    """Detector cost on REAL captured frames - the number that decides fps."""
    out = {"attempted": False}
    if args.no_infer:
        out["why"] = "skipped (--no-infer)"
        return out
    frames = FRAMES.get(args.role)
    if not frames:
        out["why"] = "no captured frames (see the cameras section)"
        return out
    try:
        import detector as D
    except Exception as e:
        out["error"] = "detector import failed: %r" % (e,)
        return out
    try:
        cfg = _cfg(args.config)
        det = D.get_detector(cfg.get("detector", {}) or {})
        out["attempted"] = True
        out["detector"] = getattr(det, "name", type(det).__name__)
        out["is_hailo"] = "hailo" in str(out["detector"]).lower()
        times, nboxes = [], 0
        for f in frames[:max(1, args.infer_frames)]:
            t0 = time.time()
            boxes = det.detect(f)
            times.append((time.time() - t0) * 1000.0)
            nboxes += len(boxes or [])
        times.sort()
        out["frames"] = len(times)
        out["detect_ms_median"] = round(times[len(times) // 2], 1)
        out["detect_ms_min"] = round(times[0], 1)
        out["detect_ms_max"] = round(times[-1], 1)
        out["fps_from_detect"] = round(1000.0 / max(0.001, times[len(times) // 2]), 2)
        out["boxes_total"] = nboxes
        out["note"] = ("inference only - capture and decode are measured "
                       "separately; not a flight result, and not a Hailo result "
                       "unless is_hailo is true")
    except Exception as e:
        out["error"] = "%s: %s" % (type(e).__name__, e)
    return out


def sec_fc(args):
    """Read the Pixhawk. OFF by default: connecting takes the serial port."""
    if not args.with_fc:
        return {"connected": False,
                "why": "pass --with-fc to read the autopilot. It takes the "
                       "serial port, so only with mission_pi STOPPED and the "
                       "vehicle DISARMED on the bench/ground"}
    out = {}
    dev = ["--device", args.fc_device] if args.fc_device else []
    rc, txt = _run([sys.executable, os.path.join(HERE, "fc_probe.py"), "--fence"] + dev,
                   timeout=90)
    out["fc_probe_rc"] = rc
    out["fc_probe"] = txt.strip()[-2500:]
    rc, txt = _run([sys.executable, os.path.join(HERE, "fc_probe.py"),
                    "--fence-check"] + dev, timeout=120)
    out["fence_check_rc"] = rc
    out["fence_check"] = txt.strip()[-2500:]
    out["fence_check_verdict"] = ("FENCE-READY" if rc == 0
                                  else "NOT fence-ready (rc=%d)" % rc)
    return out


def sec_tests(args):
    if not args.with_tests:
        return {"ran": False, "why": "pass --with-tests (takes minutes on a Pi)"}
    rc, txt = _run(["bash", os.path.join(HERE, "run_tests.sh")], timeout=3600)
    tail = txt.strip().splitlines()[-14:]
    return {"ran": True, "rc": rc, "tail": tail,
            "summary": next((l for l in reversed(tail) if "passed" in l), "")}


SECTIONS = (("system", sec_system), ("deps", sec_deps), ("hailo", sec_hailo),
            ("hailo_steps", sec_hailo_steps),
            ("detector", sec_detector), ("cameras", sec_cameras),
            ("fov", sec_fov), ("decode", sec_decode),
            ("inference", sec_inference), ("fc", sec_fc), ("tests", sec_tests))


def code_stamp(repo=None):
    """Which commit produced this report - so a stale checkout cannot hide.

    The Pi ran this tool for a full round against a checkout one commit behind,
    and the only clue was that the output looked old. Every downstream conclusion
    depended on recognising a step label from memory. A report that does not name
    its own code is ambiguous; this removes the ambiguity. Returns None when git
    is unavailable - it must never be the reason a survey fails.
    """
    repo = repo or os.path.dirname(os.path.abspath(os.path.join(HERE, os.pardir)))
    try:
        def _git(*a):
            r = subprocess.run(["git", "-C", repo] + list(a), capture_output=True,
                               timeout=10)
            return r.stdout.decode("utf-8", "replace").strip() if r.returncode == 0 \
                else None
        commit = _git("rev-parse", "--short", "HEAD")
        if not commit:
            return None
        return {"commit": commit,
                "branch": _git("rev-parse", "--abbrev-ref", "HEAD"),
                "dirty": bool(_git("status", "--porcelain"))}
    except Exception:
        return None


def build(args):
    try:
        os.makedirs(args.out_dir, exist_ok=True)
    except Exception:
        args.out_dir = "."
    wanted = None if args.sec == "all" else {args.sec}
    report = {"tool": "pi_field_report", "ts": time.time(),
              "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
              "config": os.path.abspath(args.config),
              "code": code_stamp(),
              "read_only": True, "sections": {}}
    for name, fn in SECTIONS:
        if wanted is not None and name not in wanted:
            continue
        t0 = time.time()
        try:
            report["sections"][name] = fn(args)
        except Exception as e:                  # a section must never abort the run
            report["sections"][name] = {"error": "%s: %s" % (type(e).__name__, e)}
        report["sections"][name]["_sec_ms"] = int((time.time() - t0) * 1000)
        print("  %-9s done (%d ms)" % (name, report["sections"][name]["_sec_ms"]),
              file=sys.stderr)
    return report


def summarise(r):
    """ASCII-only, partial-report tolerant.

    It is read over ssh into terminals of unknown encoding, and `--sec` leaves
    whole sections absent - so every block below is guarded by whether its
    section actually ran, rather than printing a row of "None".
    """
    s = r["sections"]
    L = []
    sysd = s.get("system") or {}
    who = sysd.get("hostname") or socket.gethostname() or "unknown host"
    L.append("=== PI FIELD REPORT %s (%s) ===" % (r.get("utc"), who))
    code = r.get("code") or {}
    if code.get("commit"):
        L.append("code       : %s%s" % (
            code["commit"], " on %s" % code["branch"] if code.get("branch") else ""))
        if code.get("dirty"):
            L.append("             the working tree has uncommitted changes - this "
                     "report may not match the commit above")
    if sysd:
        L.append("board      : %s | %s | python %s | %s cores"
                 % (sysd.get("board", "?"), sysd.get("kernel"), sysd.get("python"),
                    sysd.get("cpu_count")))
        if "cpu_temp_c" in sysd:
            L.append("thermal    : %.1f C | throttled_now=%s throttled_ever=%s"
                     % (sysd["cpu_temp_c"], sysd.get("throttled_now"),
                        sysd.get("throttled_ever")))
            for b in sysd.get("throttled_bits") or []:
                L.append("             - %s" % b)
        if "mem_total_mb" in sysd:
            L.append("memory     : %s MB total | %s%% used | disk free %s GB"
                     % (sysd["mem_total_mb"], sysd.get("mem_used_pct"),
                        sysd.get("disk_free_gb")))
    if s.get("deps"):
        miss = [k for k, v in s["deps"].items()
                if isinstance(v, str) and v.startswith("MISSING")]
        L.append("deps       : missing: %s" % (", ".join(miss) if miss else "none"))
    h = s.get("hailo") or {}
    if h:
        L.append("hailo      : platform=%s | devices=%s | hef=%s onnx=%s"
                 % (h.get("hailo_platform"), h.get("hailo_devices"),
                    h.get("hef_present"), h.get("onnx_present")))
    hs = s.get("hailo_steps") or {}
    if hs:
        _steps = hs.get("steps") or []
        # The last step is the WHOLE CONSTRUCTOR, and it can exit 0 by falling
        # back to an ONNX or classical detector. Counting that with the Hailo
        # steps made a machine with no working Hailo print "(1/13 steps ok)" -
        # which reads like progress and is the opposite of it. The brief forbids
        # a silent CPU fallback being presented as success, so the denominator is
        # the Hailo steps alone, and the constructor's fallback gets its own line.
        _ctor = hs.get("ctor_step")
        # every step whose name says "the whole constructor" can exit 0 via a
        # CPU fallback - 12 and 12b both - so neither may enter this count.
        _halo = [r for r in _steps
                 if "the whole constructor" not in str(r.get("step") or "")]
        _ctor_rec = [r for r in _steps if r.get("step") == _ctor]
        _ctor_rec = _ctor_rec[0] if _ctor_rec else {}
        n_ok = sum(1 for r in _halo if r.get("ok"))
        n_all = len(_halo)

        def _fell_back():
            if _ctor_rec.get("ok"):
                L.append("             the constructor step exited 0 anyway - it "
                         "did not use Hailo, it fell back to: %s"
                         % (_ctor_rec.get("stdout") or "an unnamed detector"))

        if hs.get("first_crash"):
            L.append("hailosteps : CRASHED at: %s (%d/%d Hailo steps ok)"
                     % (hs["first_crash"], n_ok, n_all))
            for st in hs.get("steps") or []:
                if st.get("signal"):
                    L.append("             died by %s" % st["signal"])
                    for line in (st.get("hailort_errors") or [])[:4]:
                        L.append("             %s" % line)
            for _i, _cr in enumerate(r for r in _steps if r.get("signal")):
                _got_to = [ln for ln in (_cr.get("stdout_tail") or "").splitlines()
                           if ln.strip()]
                if not _got_to:
                    continue
                if _i == 0:
                    L.append("             last output before it died: %s" % _got_to[-1])
                else:
                    L.append("             %s died too - last output: %s"
                             % (_cr.get("step") or "?", _got_to[-1]))
            _fell_back()
        elif hs.get("first_failure"):
            L.append("hailosteps : FAILED at: %s (%d/%d Hailo steps ok)"
                     % (hs["first_failure"], n_ok, n_all))
            for st in (hs.get("steps") or []):
                if not st.get("ok"):
                    seen = []
                    for line in ([st.get("error") or st.get("signal") or "failed"]
                                 + list(st.get("hailort_errors") or [])[:3]):
                        if line and line not in seen:
                            seen.append(line)
                            L.append("             %s" % line)
                    break
            _fell_back()
        else:
            L.append("hailosteps : all %d init steps completed"
                     % len(_steps))
        if hs.get("note"):
            L.append("             %s" % hs["note"])
    d = s.get("detector") or {}
    if d:
        if d.get("crashed"):
            L.append("detector   : CRASHED by %s - the ladder could not be built"
                     % d.get("signal"))
            for line in (d.get("hailort_errors") or [])[:4]:
                L.append("             %s" % line)
        else:
            L.append("detector   : chosen=%s | IS_HAILO=%s"
                     % (d.get("chosen"), d.get("is_hailo")))
            if d.get("error"):
                L.append("             error: %s" % d["error"])
    c = s.get("cameras") or {}
    if c:
        L.append("cameras    : /dev/%s" % (", ".join(c.get("dev_video") or [])
                                           or "none"))
        cap = c.get("capture")
        if isinstance(cap, dict):
            L.append("             detected: %s"
                     % list((cap.get("detected") or {}).keys()))
            for name, e in (cap.get("per_role") or {}).items():
                if e.get("error"):
                    L.append("             %s: ERROR %s" % (name, e["error"]))
                elif e.get("started"):
                    L.append("             %s: requested=%s frames=%s latest_ms=%s/%s"
                             % (name, e.get("requested_size"), e.get("frames"),
                                e.get("latest_ms_median"), e.get("latest_ms_max")))
                    if e.get("size_warning"):
                        L.append("             %s: %s" % (name, e["size_warning"]))
            if cap.get("error"):
                L.append("             %s" % cap["error"])
        else:
            L.append("             capture: %s" % cap)
    f = s.get("fov") or {}
    if f:
        if f.get("measured"):
            L.append("FOV        : MEASURED hfov=%.2f deg%s (config says hfov=%s%s)"
                     % (f["hfov_deg"],
                        (" vfov=%.2f deg" % f["vfov_deg"]) if f.get("vfov_deg") else "",
                        f.get("hfov_deg_config"),
                        (" vfov=%s" % f.get("vfov_deg_config"))
                        if f.get("vfov_deg_config") else ""))
            L.append("             sheet long/short %s px | spread %s%% | %s mm/px "
                     "| %s | at %s m"
                     % (f.get("sheet_long_px_median"), f.get("sheet_px_spread_pct"),
                        f.get("mm_per_px_on_ground"), f.get("long_side_runs"),
                        f["dist_m"]))
            if f.get("aspect_check"):
                ac = f["aspect_check"]
                L.append("             aspect declared %s vs measured %s -> %s"
                         % (ac["declared_long_over_short"],
                            ac["measured_long_over_short"],
                            "consistent" if ac["agree_within_10pct"]
                            else "INCONSISTENT (>10%), treat as suspect"))
        else:
            L.append("FOV        : NOT MEASURED - %s"
                     % (f.get("error") or f.get("why") or "?"))
    dec = s.get("decode") or {}
    if dec:
        if dec.get("attempted"):
            L.append("decode     : decoded=%s payloads=%s in %s ms"
                     % (dec.get("decoded"), dec.get("payloads"),
                        dec.get("decode_ms_median")))
        else:
            L.append("decode     : not attempted - %s"
                     % (dec.get("error") or dec.get("why") or "?"))
    inf = s.get("inference") or {}
    if inf:
        if inf.get("attempted"):
            L.append("inference  : %s median %s ms (min %s max %s) = %s fps | "
                     "IS_HAILO=%s"
                     % (inf.get("detector"), inf.get("detect_ms_median"),
                        inf.get("detect_ms_min"), inf.get("detect_ms_max"),
                        inf.get("fps_from_detect"), inf.get("is_hailo")))
        else:
            L.append("inference  : not attempted - %s"
                     % (inf.get("error") or inf.get("why") or "?"))
    fc = s.get("fc") or {}
    if fc:
        if fc.get("fc_probe_rc") == 0:
            L.append("fc         : connected")
            L.append("             fence: %s" % fc.get("fence_check_verdict"))
        else:
            L.append("fc         : %s"
                     % (fc.get("error") or fc.get("why") or "?"))
    t = s.get("tests") or {}
    if t:
        L.append("tests      : %s" % (t.get("summary") if t.get("ran")
                                      else t.get("why")))
    L.append("=== end ===")
    return "\n".join(L)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=os.path.join(PI, "config.yaml"))
    # NOT the current directory. Reports used to land in mission_pi/ and two of
    # them were swept into a commit - the tool must not leave artefacts in the
    # repository it is inspecting, and `git status` on the Pi should stay clean.
    ap.add_argument("--out-dir", default=os.path.expanduser("~/pi_field_reports"),
                    help="where to write the JSON and the FOV frame "
                         "(default %(default)s)")
    ap.add_argument("--role", default="bottom", choices=("bottom", "front"))
    ap.add_argument("--samples", type=int, default=5)
    ap.add_argument("--infer-frames", type=int, default=5)
    ap.add_argument("--dist-m", type=float, default=0.0,
                    help="tape-measured lens-to-sheet distance, metres")
    ap.add_argument("--panel-mm", type=float, default=0.0,
                    help="sheet LONG side, mm (A3 = 420). Which image axis it "
                         "runs along is read from the picture, not assumed")
    ap.add_argument("--panel-mm-h", type=float, default=0.0,
                    help="sheet SHORT side, mm (A3 = 297). Give it to also get "
                         "VFOV and an aspect sanity check")
    ap.add_argument("--qr-modules", type=int, default=0,
                    help="modules across the printed QR, for px/module")
    ap.add_argument("--no-capture", action="store_true")
    ap.add_argument("--no-decode", action="store_true")
    ap.add_argument("--no-infer", action="store_true")
    ap.add_argument("--with-fc", action="store_true")
    ap.add_argument("--fc-device", default=None)
    ap.add_argument("--with-tests", action="store_true")
    ap.add_argument("--json", default=None, help="explicit output path")
    ap.add_argument("--sec", default="all",
                    choices=("all",) + tuple(n for n, _ in SECTIONS),
                    help="run one section only (faster when you just want one "
                         "answer, e.g. --sec fov)")
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    print("pi_field_report: READ-ONLY survey. Stop mission_pi first if you want "
          "capture timings.", file=sys.stderr)
    report = build(args)
    # Include the section and the pid: a second-resolution timestamp alone let
    # a quick `--sec fov` run OVERWRITE the full survey written seconds earlier.
    path = args.json or os.path.join(args.out_dir, "field_report_%s%s.json" % (
        time.strftime("%Y%m%d_%H%M%S"),
        "" if args.sec == "all" else "_" + args.sec))
    with open(path, "w") as fh:
        json.dump(report, fh, indent=2, sort_keys=True, default=str)
    print(summarise(report))
    print("\nfull report: %s" % path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
