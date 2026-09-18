"""Fake-QR filter + environment presets (the window-grill fix).

Problem (2026-09): a diamond window grill read as "qr" by YOLO at conf
0.16-0.24 and painted orange boxes across the whole ground view — pure
noise for a ground-level (kabaddi/night) run. Two independent problems:

  * the DETECTOR confidence floor (0.25 default) is deliberately low for
    15 m recall — real QRs at 15 m score ~0.5, but grill blobs ride the
    0.15-0.30 band and survive it;
  * the CUE path (bbox -> steer flight) never required a decode, so any
    surviving blob could drive the drone at a window.

Fix — a post-detection filter that keeps a box ONLY when it passes all:
  1. conf  >= qr_filter.min_conf            (default 0.35, above the grill band)
  2. side  >= qr_filter.min_side_px         (default 15 px — kill specks)
  3. 0.4 <= w/h <= 2.5                      (QRs are ~square; grill shards
                                             are tall/narrow diamonds)
plus an OPTIONAL strict mode `require_decode_for_cue`: a flight cue may
only come from a box that ACTUALLY decoded to a payload — only real QRs
trigger the mission. The decode/consensus path was already decode-only;
this closes the cue path too.

Presets — one named environment setting (detector conf, tile grid, decode
cadence, cue strictness, ISP boost profile). Defined once in
config.yaml `qr_presets:` and applied everywhere from the same dict:
  * mission pipeline   : POST /api/qr/preset {"preset": "dark"}
  * webcam bench tool  : tools/test_yolo_webcam_qr.py --preset dark,
                         or live keys p / 1-7
  * startup            : detector.preset in config.yaml

A preset's conf is the DETECTOR sensitivity; qr_filter.min_conf stays the
safety floor on top (a 0.25-conf preset still cannot admit a 0.24 grill
blob).

Stdlib-only: usable from tests, the tool, the server, and the mission.
"""
import logging

log = logging.getLogger("qr_filter")

DEFAULT_FILTER = {
    "min_conf": 0.35,
    "min_side_px": 15,
    "aspect_min": 0.4,
    "aspect_max": 2.5,
    "require_decode_for_cue": False,
}

# Presets used when config.yaml has no qr_presets block (the webcam tool
# runs standalone). Order here = the 1-7 key order in the tool.
DEFAULT_PRESETS = {
    "day": {
        "conf_thr": 0.35, "tile_grid": [3, 3], "full_decode_every_n": 3,
        "require_decode_for_cue": False, "boost": "qr_boost_day",
        "desc": "daylight 15 m search (default)",
    },
    "far": {
        "conf_thr": 0.30, "tile_grid": [3, 3], "full_decode_every_n": 3,
        "require_decode_for_cue": False, "boost": "qr_boost_day",
        "desc": "max range 15 m — more recall, still above grill conf",
    },
    "dark": {
        "conf_thr": 0.30, "tile_grid": [3, 3], "full_decode_every_n": 2,
        "require_decode_for_cue": True, "boost": "qr_boost_lowlight",
        "desc": "night ground (a.k.a. dark_qr_boost) — only decodes count",
    },
    "kabaddi": {
        "conf_thr": 0.30, "tile_grid": [1, 1], "full_decode_every_n": 1,
        "require_decode_for_cue": True, "boost": "qr_boost_lowlight",
        "desc": "ground level, close range — full frame, only real QRs trigger",
    },
    "low": {
        "conf_thr": 0.25, "tile_grid": [3, 3], "full_decode_every_n": 2,
        "require_decode_for_cue": True, "boost": "qr_boost_lowlight",
        "desc": "indoor low light — sensitive but strict on cues",
    },
    "aggressive": {
        "conf_thr": 0.25, "tile_grid": [3, 3], "full_decode_every_n": 2,
        "require_decode_for_cue": False, "boost": "qr_boost_aggressive",
        "desc": "bright high-contrast day — detector sensitivity up",
    },
    "bench": {
        "conf_thr": 0.35, "tile_grid": [1, 1], "full_decode_every_n": 1,
        "require_decode_for_cue": False, "boost": "normal",
        "desc": "SITL/bench — full frame, fastest, no boost",
    },
}

# The user's other-session tool was launched with --preset dark_qr_boost;
# map those friendly names onto the canonical preset keys.
PRESET_ALIASES = {
    "dark_qr_boost": "dark",
    "qr_boost_dark": "dark",
    "night": "dark",
    "ground": "kabaddi",
    "15m": "far",
}


def normalize_preset_name(name):
    """'dark_qr_boost' -> 'dark'; unknown names pass through."""
    key = str(name or "").strip().lower()
    return PRESET_ALIASES.get(key, key)


class QrFilter:
    """Geometry + confidence gate on detector boxes.

    `box` is anything with numeric x/y/w/h/conf (detector.BBox). Returns
    (kept: bool, reason: str) from should_keep() — reason is what the
    overlay shows on a rejected ("fake") box.
    """

    def __init__(self, min_conf=DEFAULT_FILTER["min_conf"],
                 min_side_px=DEFAULT_FILTER["min_side_px"],
                 aspect_min=DEFAULT_FILTER["aspect_min"],
                 aspect_max=DEFAULT_FILTER["aspect_max"],
                 require_decode_for_cue=DEFAULT_FILTER["require_decode_for_cue"]):
        self.min_conf = float(min_conf)
        self.min_side_px = int(min_side_px)
        self.aspect_min = float(aspect_min)
        self.aspect_max = float(aspect_max)
        self.require_decode_for_cue = bool(require_decode_for_cue)

    @classmethod
    def from_config(cls, cfg):
        f = (cfg or {}).get("qr_filter", {}) or {}
        return cls(
            min_conf=f.get("min_conf", DEFAULT_FILTER["min_conf"]),
            min_side_px=f.get("min_side_px", DEFAULT_FILTER["min_side_px"]),
            aspect_min=f.get("aspect_min", DEFAULT_FILTER["aspect_min"]),
            aspect_max=f.get("aspect_max", DEFAULT_FILTER["aspect_max"]),
            require_decode_for_cue=f.get(
                "require_decode_for_cue", DEFAULT_FILTER["require_decode_for_cue"]),
        )

    def should_keep(self, box):
        try:
            w, h = float(box.w), float(box.h)
            conf = float(box.conf)
        except Exception:
            return False, "bad-box"
        if conf < self.min_conf:
            return False, "conf %.2f < %.2f" % (conf, self.min_conf)
        if w < self.min_side_px or h < self.min_side_px:
            return False, "size %dx%d < %d" % (int(w), int(h), self.min_side_px)
        ar = (w / h) if h > 0 else 0.0
        if ar < self.aspect_min or ar > self.aspect_max:
            return False, "aspect %.2f (want %.1f-%.1f)" % (
                ar, self.aspect_min, self.aspect_max)
        return True, "ok"

    def filter_boxes(self, boxes):
        return [b for b in boxes if self.should_keep(b)[0]]

    def as_dict(self):
        return {"min_conf": self.min_conf, "min_side_px": self.min_side_px,
                "aspect_min": self.aspect_min, "aspect_max": self.aspect_max,
                "require_decode_for_cue": self.require_decode_for_cue}


def load_presets(cfg=None):
    """Ordered {name: preset-dict} from config, else built-ins.

    Every preset is validated/normalized (missing keys get defaults) so a
    hand-edited config can't crash the tool or the server.
    """
    raw = (cfg or {}).get("qr_presets", None)
    if not isinstance(raw, dict) or not raw:
        raw = DEFAULT_PRESETS
    out = {}
    for name, p in raw.items():
        p = dict(p or {})
        name = normalize_preset_name(name)
        base = DEFAULT_PRESETS.get(name, DEFAULT_PRESETS["day"])
        out[name] = {
            "conf_thr": float(p.get("conf_thr", base["conf_thr"])),
            "iou_thr": float(p.get("iou_thr", 0.45)),
            "tile_grid": [int(x) for x in (p.get("tile_grid", base["tile_grid"]))],
            "full_decode_every_n": int(p.get("full_decode_every_n",
                                             base["full_decode_every_n"])),
            "require_decode_for_cue": bool(p.get(
                "require_decode_for_cue", base["require_decode_for_cue"])),
            "boost": str(p.get("boost", base["boost"])),
            "desc": str(p.get("desc", base["desc"])),
        }
    if not out:
        out = dict(DEFAULT_PRESETS)
    return out


def preset_names(cfg=None):
    """Ordered list of preset names (the 1-7 key order)."""
    return list(load_presets(cfg).keys())


def apply_preset(mission, name, cfg=None):
    """Apply a named preset to a Mission (or a duck-typed stand-in).

    Mutates: mission.qf (filter), mission.detector thresholds,
    mission.tile_rows/cols, mission.full_decode_every,
    mission.preset_name; applies the ISP boost to the bottom cam.
    Returns the applied preset dict, or None when the name is unknown.
    """
    cfg = cfg if cfg is not None else getattr(mission, "cfg", None) or {}
    presets = load_presets(cfg)
    name = normalize_preset_name(name)
    if name not in presets:
        log.warning("unknown preset %r (have %s)", name, sorted(presets))
        return None
    p = presets[name]

    qf = getattr(mission, "qf", None)
    if qf is None:
        qf = QrFilter.from_config(cfg)
        mission.qf = qf
    qf.require_decode_for_cue = p["require_decode_for_cue"]

    det = getattr(mission, "detector", None)
    if det is not None and hasattr(det, "set_thresholds"):
        try:
            det.set_thresholds(conf_thr=p["conf_thr"], iou_thr=p["iou_thr"])
        except Exception as e:
            log.debug("preset %s: set_thresholds failed: %r", name, e)

    try:
        tg = p["tile_grid"]
        mission.tile_rows, mission.tile_cols = int(tg[0]), int(tg[1])
    except Exception:
        pass
    try:
        mission.full_decode_every = max(1, p["full_decode_every_n"])
    except Exception:
        pass
    try:
        mission.preset_name = name
    except Exception:
        pass

    # ISP boost profile on the bottom cam (best effort — a dead camera
    # must not break a preset switch).
    try:
        rig = getattr(mission, "rig", None)
        if rig is not None and p.get("boost"):
            cam = None
            for c in (rig.cams or {}).values():
                if getattr(c, "facing", None) == "bottom":
                    cam = c
                    break
            if cam is None:
                cam = (rig.cams or {}).get("cam2") or (rig.cams or {}).get("cam1")
            if cam is not None and hasattr(cam, "apply_qr_profile"):
                cam.apply_qr_profile(p["boost"])
                log.info("preset %s: boost %s -> %s", name, p["boost"], cam.name)
    except Exception as e:
        log.debug("preset %s: boost apply failed: %r", name, e)

    log.info("preset -> %s (conf %.2f, tile %dx%d, decode_every %d, "
             "cue-strict=%s, boost %s)", name, p["conf_thr"],
             p["tile_grid"][0], p["tile_grid"][1], p["full_decode_every_n"],
             p["require_decode_for_cue"], p["boost"])
    return p
