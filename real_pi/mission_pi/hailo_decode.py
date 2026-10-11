#!/usr/bin/env python3
"""Decode ANY output layout a Hailo HEF can hand us, and configure HailoRT
across API generations.

Why this file exists
--------------------
`detector.py` originally assumed two output layouts and one HailoRT API call.
The UC-517's HEF (compiled by the user, HailoRT 4.23) broke both assumptions:

  * `ConfigureParams.create_from_hef(hef, interface="PCIe")` no longer exists —
    4.23 wants `hef.create_configure_params(StreamInterface.PCIe)`.
  * the NMS output is `(1, 5, 100)`: **five rows** (y_min, x_min, y_max, x_max,
    score) x 100 detections, normalised. That is features-major storage, so
    the "100 boxes of 6 values" intuition reads it backwards.

Both facts are verifiable in Hailo's own source: hailort's python bindings bind
`create_configure_params(StreamInterface)` on the HEF object, and
`HailoRTTransformUtils` documents the NMS tensor as 5 `BBOX_PARAMS` values per
box, normalised, in TAPPAS order (y_min, x_min, y_max, x_max, score).

Everything here is pure Python/numpy: no Hailo runtime needed, so every branch
is unit-tested. On the Pi a wrong guess in this file is indistinguishable from
"the model detects nothing", and that is the most expensive kind of bug to
find in the field.
"""
import numpy as np

#: Values per box in a Hailo NMS tensor (normalised 0..1).
NMS_FIELDS = 5
#: ...in THIS order. Authoritative, from HailoRT's own buffer layout — tappas
#: `core/hailo/libs/postprocesses/common/structures.hpp`:
#:     typedef struct { uint16_t y_min, x_min, y_max, x_max, score; } hailo_bbox_t;
#: (five float32 for a FLOAT32 vstream). So it is y_min first, NOT x_min, and
#: the values are normalised, not pixels.
#: `NMS_ORDER` is only overridable so a field mismatch costs a config edit
#: instead of a code change: for any well-formed box BOTH readings are
#: geometrically valid (x<->y swap), so nothing can auto-detect which is meant
#: — the tell is that the box lands mirrored across the frame diagonal.
NMS_ORDER = "yxyx"
NMS_ORDERS = ("yxyx", "xyxy")
#: A raw ultralytics head has thousands of anchors; NMS keeps <= ~1000 boxes.
#: This count is the only thing that tells (1, 5, 8400) from (1, 5, 100).
RAW_HEAD_MIN_ANCHORS = 1000

STYLES = ("A", "A'", "A''", "B")
#: What the runtime tries, in order — a HEF is only unusable when none of these
#: match any of its outputs.
DECODE_ORDER = ("A", "A''", "B", "A'", "A'-flat")


def style_of_shape(shape):
    """One tensor shape -> which decoder branch would read it (or None).

    The single source of truth for "can the runtime use this output": both
    hailo_decode and tools/hef_info.py answer from here, so a HEF that the
    inspector calls usable can never fail to parse on the Pi.
    """
    shp = [int(d) for d in (shape or ()) if d is not None]
    if not shp:
        return None
    if len(shp) >= 2 and shp[-1] == 6:
        return "A"
    if len(shp) == 4 and shp[-1] == NMS_FIELDS:
        return "A''"
    if (len(shp) == 3 and shp[0] == 1 and shp[1] == 5
            and shp[2] >= RAW_HEAD_MIN_ANCHORS):
        return "B"
    if len(shp) == 3 and shp[1] % NMS_FIELDS == 0:
        return "A'"
    if len(shp) == 2 and shp[1] == NMS_FIELDS and shp[0] <= RAW_HEAD_MIN_ANCHORS:
        return "A'-flat"
    return None


def describe_shape(name, shape):
    """One output tensor -> the sentence printed by hef_info / logged on the Pi."""
    shp = [int(d) for d in (shape or ()) if d is not None]
    style = style_of_shape(shp)
    if style == "A":
        return ("style A — NMS with class id, %s %s" % (name, shp))
    if style == "A''":
        return "style A'' — NMS with class axis, %s %s" % (name, shp)
    if style == "B":
        return ("style B — raw ultralytics head, %s %s (NMS on the Pi CPU)"
                % (name, shp))
    if style == "A'":
        return ("style A' — Hailo NMS rows (y_min,x_min,y_max,x_max,score "
                "normalised), %s %s" % (name, shp))
    if style == "A'-flat":
        return "style A'-flat — NMS rows, %s %s" % (name, shp)
    return ("UNREADABLE shape %s %s — the runtime would return no boxes for "
            "every frame" % (name, shp))





def hailo_configure_params(hef, hp=None):
    """Build HailoRT configure params across API generations.

    Modern (HailoRT 4.2x): `hef.create_configure_params(StreamInterface.PCIe)`.
    Older: `ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)`,
    and older still took the string "PCIe". Newest first, and the last error is
    reported if nothing works — a version bump must produce a sentence, not a
    bare TypeError deep inside the mission (which is how the HAT sat idle while
    the mission happily ran on ONNX).
    """
    if hp is None:
        import hailo_platform as hp
    iface = None
    for name in ("HailoStreamInterface", "StreamInterface"):
        iface = getattr(hp, name, None)
        if iface is not None:
            break
    pcie = getattr(iface, "PCIe", None) if iface is not None else None
    if pcie is None:
        pcie = "PCIe"

    tries = []
    mk = getattr(hef, "create_configure_params", None)
    if callable(mk):
        tries.append(lambda: mk(pcie))
    cp = getattr(hp, "ConfigureParams", None)
    if cp is not None:
        tries.append(lambda: cp.create_from_hef(hef, interface=pcie))
        tries.append(lambda: cp.create_from_hef(hef, interface="PCIe"))
        tries.append(lambda: cp.create_from_hef(hef))
    last = None
    for fn in tries:
        try:
            return fn()
        except Exception as e:          # probing APIs is the point here
            last = e
    raise RuntimeError("no HailoRT API worked to configure the HEF (%r) — "
                       "please report `hailortcli --version`" % (last,))


def hailo_vstream_params(hp, ng, ng_params, use_input, format_type):
    """`InputVStreamParams.make` / `OutputVStreamParams.make` across versions.

    The current signature takes the CONFIGURED NETWORK GROUP (as Hailo's model
    zoo does); older builds took the group's params object.
    """
    cls = hp.InputVStreamParams if use_input else hp.OutputVStreamParams
    last = None
    for target in (ng, ng_params):
        if target is None:
            continue
        try:
            return cls.make(target, format_type=format_type)
        except Exception as e:
            last = e
    raise RuntimeError("could not create %s vstream params (%r)"
                       % ("input" if use_input else "output", last))


def nms_shape_of(vstream_info):
    """(number_of_classes, max_bboxes_per_class) from a Hailo vstream info.

    HailoRT declares this itself for NMS outputs; it is the runtime's own
    answer to "how many classes and how many boxes", so it beats any guess
    based on tensor shapes. -> (None, None) when the info has no nms_shape.
    """
    shp = getattr(vstream_info, "nms_shape", None)
    if shp is None:
        return None, None
    return (getattr(shp, "number_of_classes", None),
            getattr(shp, "max_bboxes_per_class", None))


# ------------------------------------------------------------------ decoding --
def order_of(value):
    """Coerce a user/config value to one of NMS_ORDERS ('auto' -> NMS_ORDER)."""
    v = str(value or "auto").strip().lower()
    return v if v in NMS_ORDERS else NMS_ORDER


def _rows_to_boxes(block, input_size, conf_thr, scale, px, py, iw, ih,
                   order=NMS_ORDER):
    """(5, N) Hailo NMS block -> frame-space boxes [(x1,y1,x2,y2,score), ...].

    Coordinates are normalised to the network input; both normalised and
    (in case a build emits pixels) input-pixel values are accepted. `order` is
    the documented Hailo layout unless a config override says otherwise.
    """
    boxes = []
    if block.shape[0] != NMS_FIELDS:
        return boxes
    order = order_of(order)
    for j in range(block.shape[1]):
        sc = float(block[4, j])
        if sc < conf_thr:
            continue
        a, b, c, d = (float(block[0, j]), float(block[1, j]),
                      float(block[2, j]), float(block[3, j]))
        x1, y1, x2, y2 = (b, a, d, c) if order == "yxyx" else (a, b, c, d)
        if max(x1, y1, x2, y2) <= 1.5:              # normalised -> input px
            x1, y1, x2, y2 = (x1 * input_size, y1 * input_size,
                              x2 * input_size, y2 * input_size)
        X1 = max(0.0, (x1 - px) / scale)
        Y1 = max(0.0, (y1 - py) / scale)
        X2 = min(float(iw), (x2 - px) / scale)
        Y2 = min(float(ih), (y2 - py) / scale)
        if X2 > X1 and Y2 > Y1:
            boxes.append((X1, Y1, X2, Y2, sc))
    return boxes


def _box_rows_to_boxes(det, input_size, conf_thr, scale, px, py, iw, ih):
    """(N, 6) NMS rows [x1,y1,x2,y2,score,class] -> frame-space boxes."""
    boxes = []
    for x1, y1, x2, y2, sc, cl in det:
        sc = float(sc)
        if sc < conf_thr or int(cl) != 0:
            continue
        x1, y1, x2, y2 = float(x1), float(y1), float(x2), float(y2)
        if max(x1, y1, x2, y2) <= 1.5:
            x1, y1, x2, y2 = (x1 * input_size, y1 * input_size,
                              x2 * input_size, y2 * input_size)
        X1 = max(0.0, (x1 - px) / scale)
        Y1 = max(0.0, (y1 - py) / scale)
        X2 = min(float(iw), (x2 - px) / scale)
        Y2 = min(float(ih), (y2 - py) / scale)
        if X2 > X1 and Y2 > Y1:
            boxes.append((X1, Y1, X2, Y2, sc))
    return boxes


def _raw_head_to_boxes(a, conf_thr, scale, px, py, iw, ih):
    """(1, 5, N>=1000) ultralytics head (cxcywh + conf, input px) -> boxes."""
    boxes = []
    for cx, cy, w, h, sc in a[0].transpose(1, 0):
        sc = float(sc)
        if sc < conf_thr:
            continue
        x1 = float(cx - w / 2) - px
        y1 = float(cy - h / 2) - py
        x2 = float(cx + w / 2) - px
        y2 = float(cy + h / 2) - py
        X1, Y1 = max(0.0, x1 / scale), max(0.0, y1 / scale)
        X2 = min(float(iw), x2 / scale)
        Y2 = min(float(ih), y2 / scale)
        if X2 > X1 and Y2 > Y1:
            boxes.append((X1, Y1, X2, Y2, sc))
    return boxes


#: Score position per layout — every Hailo YOLO layout carries the box score
#: in the same slot, which is what makes one reader enough.
def tensor_peek(a, style):
    """{'max_score': float, 'filled': int} for a tensor of a known style.

    Added after the 2026-10-12 field run returned `raw_boxes=0` for 936 frames
    with NO explanation available: a HEF that never fires and a HEF whose boxes
    are all under our confidence floor look identical from the outside. This
    reads the tensor itself, below any threshold, so the log can say which one
    it is. `filled` counts rows with any score at all (> 0).
    """
    out = {"max_score": -1.0, "filled": 0}
    try:
        x = np.asarray(a, dtype=np.float32)
        if style == "A":
            sc = x.reshape(-1, 6)[:, 4]
        elif style == "A'":
            sc = x[0, 4::NMS_FIELDS] if x.ndim == 3 else x.reshape(-1)
        elif style == "A''":
            sc = x[..., 4].reshape(-1)
        elif style == "A'-flat":
            sc = x[..., 4].reshape(-1)
        elif style == "B":
            sc = x[0, 4].reshape(-1) if x.ndim == 3 else x.reshape(-1)
        else:
            return out
        if sc.size:
            out["max_score"] = float(sc.max())
            out["filled"] = int((sc > 0).sum())
    except Exception:
        pass
    return out


def decode_hailo_outputs(outs, input_size=640, conf_thr=0.35, nms_fn=None,
                         scale=1.0, px=0.0, py=0.0, iw=640, ih=640, note=None,
                         xy_order="auto", stats=None):
    """Any supported HEF output -> frame-space boxes; applies nms_fn at the end.

    Layouts, tried in the order the runtime prefers:
      A    (..., 6)        per box [x1,y1,x2,y2,score,class]
      A''  (b, c, N, 5)    explicit class axis, 5 values per box
      B    (1, 5, >=1000)  raw ultralytics head (cxcywh + conf, input px)
      A'   (b, 5*C, N)     Hailo NMS fixed tensor — features-major rows, which
                           is what the UC-517's HEF returns as (1, 5, 100)
      (N, 5) additionally accepted for either reading.
    A list/tuple output (HailoRT's per-class NMS form) is flattened, each
    element carrying its class index.

    `note(style_or_None)` is called once with the branch that fired, so a field
    log says WHICH layout was used instead of leaving it to guesswork. It is
    called even when the branch yields NO boxes — the layout of a HEF does not
    depend on whether it happens to detect something, and hiding that made a
    silent zero-box run undiagnosable.

    `stats` (optional dict, filled in place) receives
    {"style", "max_score", "filled"} straight off the tensor, BELOW any
    threshold, so "the model sees nothing" and "the boxes are under our floor"
    can be told apart.
    """
    arg = dict(input_size=int(input_size), conf_thr=float(conf_thr),
               scale=scale, px=px, py=py, iw=iw, ih=ih)
    if str(xy_order or "auto").lower() in NMS_ORDERS:
        # only the rows-based layouts (A', and (N,5)) carry bare x/y pairs
        arg["order"] = str(xy_order).lower()
    flat = []
    for o in outs or []:
        if isinstance(o, (list, tuple)):
            for ci, e in enumerate(o):
                arr = np.asarray(e)
                if arr.size:
                    flat.append((arr, ci))
        else:
            arr = np.asarray(o)
            if arr.size:
                flat.append((arr, 0))

    def _finish(style, boxes):
        if note:
            note(style)
        if stats is not None:
            stats["style"] = style
        return nms_fn(boxes) if (nms_fn and boxes) else boxes

    def _peek(a, style):
        """Remember what the tensor held, even when nothing passes."""
        if stats is None:
            return
        stats.update(tensor_peek(a, style))
        stats["style"] = style

    # ---- A: NMS with the class id in the last column ------------------------
    for a, _ci in flat:
        if a.ndim >= 2 and a.shape[-1] == 6:
            _peek(a, "A")
            boxes = _box_rows_to_boxes(a.reshape(-1, 6), **arg)
            if boxes:
                return _finish("A", boxes)

    # ---- A'': explicit class axis ------------------------------------------
    for a, _ci in flat:
        if a.ndim == 4 and a.shape[-1] == NMS_FIELDS:
            boxes = []
            for ci in range(a.shape[1]):
                boxes += _rows_to_boxes(a[0, ci].T, **arg)
            _peek(a, "A''")
            if boxes:
                return _finish("A''", boxes)

    # ---- B: raw head. Must come BEFORE A' — both are (1,5,N) and only the
    #         anchor count tells them apart (8400 anchors vs 100 NMS slots) --
    for a, _ci in flat:
        if (a.ndim == 3 and a.shape[0] == 1 and a.shape[1] == 5
                and a.shape[2] >= RAW_HEAD_MIN_ANCHORS):
            boxes = _raw_head_to_boxes(a, **{k: arg[k] for k in
                                             ("conf_thr", "scale", "px", "py",
                                              "iw", "ih")})
            _peek(a, "B")
            if boxes:
                return _finish("B", boxes)

    # ---- A': Hailo NMS fixed tensor, features-major rows -------------------
    for a, cls_hint in flat:
        if a.ndim == 3 and a.shape[1] % NMS_FIELDS == 0:
            boxes = []
            if cls_hint:                       # per-class element of a list
                boxes += _rows_to_boxes(a[0], **arg)
            else:                              # (b, 5*C, N) rows per class
                for ci in range(max(1, a.shape[1] // NMS_FIELDS)):
                    lo = ci * NMS_FIELDS
                    boxes += _rows_to_boxes(a[0][lo:lo + NMS_FIELDS], **arg)
            _peek(a, "A'")
            if boxes:
                return _finish("A'", boxes)

    # ---- (N, 5) either reading ---------------------------------------------
    for a, _ci in flat:
        if (a.ndim == 2 and a.shape[1] == NMS_FIELDS
                and a.shape[0] <= RAW_HEAD_MIN_ANCHORS):
            _peek(a, "A'-flat")
            boxes = _rows_to_boxes(a.T, **arg)
            if boxes:
                return _finish("A'-flat", boxes)

    # No boxes anywhere — but a layout may still have been RECOGNISED, and that
    # is the difference between "the model sees nothing" (a real answer) and
    # "we cannot read this HEF" (a bug on our side). Say which one it is.
    if note:
        note((stats or {}).get("style") or None)
    return []


def describe_outputs(shapes):
    """[(name, shape), ...] -> human sentences about what we can read."""
    out = []
    for name, shape in shapes or []:
        shp = [int(d) for d in (shape or []) if d is not None]
        if not shp:
            continue
        if shp[-1] == 6 and len(shp) >= 2:
            out.append("style A — NMS with class id, %s %s" % (name, shp))
        elif len(shp) == 4 and shp[-1] == NMS_FIELDS:
            out.append("style A'' — NMS with class axis, %s %s"
                       % (name, shp))
        elif (len(shp) == 3 and shp[0] == 1 and shp[1] == 5
                and shp[2] >= RAW_HEAD_MIN_ANCHORS):
            out.append("style B — raw ultralytics head, %s %s (NMS on the Pi "
                       "CPU)" % (name, shp))
        elif len(shp) == 3 and shp[1] % NMS_FIELDS == 0:
            out.append("style A' — Hailo NMS rows (y_min,x_min,y_max,x_max,"
                       "score normalised), %s %s" % (name, shp))
        elif len(shp) == 2 and shp[1] == NMS_FIELDS:
            out.append("style A'-flat — NMS rows, %s %s" % (name, shp))
        else:
            out.append("UNREADABLE shape %s %s — the runtime would return no "
                       "boxes for every frame" % (name, shp))
    return out
