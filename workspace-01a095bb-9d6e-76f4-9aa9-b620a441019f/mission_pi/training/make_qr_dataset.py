#!/usr/bin/env python3
"""Synthetic A3-QR detection dataset (YOLO format) — unlimited, altitude-matched.

Renders random QR payloads, warps them with perspective/rotation (as seen
from a drone), pastes 1-3 per image on procedural ground backgrounds
(+ optional real --bg-dir), and applies the degradations that actually
happen at 15 m: small scale, motion blur, exposure/brightness swings.

Scale truth (Pi Cam 3, 66 deg HFOV): an A3 QR (297 mm side) is ~70 px in
a 12 MP frame at 15 m, ~10 px after a naive 640 resize, ~22 px inside a
2x2 tile. So the default --px range (10-140) brackets tiled detect duty.
"""
import argparse
import os
import random
import sys

try:
    import cv2
    import numpy as np
    import qrcode
    from PIL import Image
except Exception as e:
    sys.exit("need qrcode pillow numpy opencv-python: pip install qrcode pillow numpy opencv-python (%r)" % (e,))

ALPHABET = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"


def rand_payload(rng, nchars=6):
    return "".join(rng.choice(ALPHABET) for _ in range(nchars))


def make_background(rng, size):
    kind = rng.randrange(4)
    base = [(88, 110, 72), (110, 110, 112), (128, 118, 100), (70, 90, 60)][kind]
    bg = np.full((size, size, 3), base, np.uint8)
    noise = rng.integers(-28, 28, (size, size, 3)).astype(np.int16)
    bg = np.clip(bg.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    # blotches + distractor rectangles (vents, panels, shadows)
    for _ in range(rng.integers(2, 6)):
        x, y = rng.integers(0, size, 2)
        r = rng.integers(20, 120)
        c = tuple(int(v) for v in rng.integers(30, 200, 3))
        cv2.circle(bg, (int(x), int(y)), int(r), c, -1)
    for _ in range(rng.integers(0, 3)):
        x, y = rng.integers(0, size - 60, 2)
        w, h = rng.integers(20, 140, 2)
        c = tuple(int(v) for v in rng.integers(20, 220, 3))
        cv2.rectangle(bg, (int(x), int(y)), (int(x + w), int(y + h)), c, -1)
    return bg


def render_qr(payload, side_px):
    qr = qrcode.QRCode(border=2, box_size=max(2, side_px // 32))
    qr.add_data(payload)
    qr.make(fit=True)
    img = qr.make_image(fill_color="black", back_color="white").convert("RGB")
    img = img.resize((side_px, side_px), Image.NEAREST)
    return cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)


def paste_warped(bg, qr, rng):
    H, W = bg.shape[:2]
    s = qr.shape[0]
    # random quad: rotate + perspective + position
    ang = rng.uniform(0, 360)
    M = cv2.getRotationMatrix2D((s / 2, s / 2), ang, 1.0)
    qr_r = cv2.warpAffine(qr, M, (s, s), borderValue=(255, 255, 255))
    j = s * 0.18
    src = np.float32([[0, 0], [s, 0], [s, s], [0, s]])
    dst = src + rng.uniform(-j, j, (4, 2)).astype(np.float32)
    P = cv2.getPerspectiveTransform(src, dst)
    qr_w = cv2.warpPerspective(qr_r, P, (s, s), borderValue=(255, 255, 255))
    x0 = int(rng.integers(-s // 4, W - 3 * s // 4))
    y0 = int(rng.integers(-s // 4, H - 3 * s // 4))
    x1, y1 = max(0, x0), max(0, y0)
    x2, y2 = min(W, x0 + s), min(H, y0 + s)
    if x2 <= x1 or y2 <= y1:
        return None
    bg[y1:y2, x1:x2] = qr_w[y1 - y0:y2 - y0, x1 - x0:x2 - x0]
    # label = axis-aligned bbox of the visible quad
    corners = (dst + np.float32([x0, y0])).reshape(-1, 2)
    cx1, cy1 = np.clip(corners.min(0), 0, [W, H])
    cx2, cy2 = np.clip(corners.max(0), 0, [W, H])
    if cx2 - cx1 < 6 or cy2 - cy1 < 6:
        return None
    return (cx1, cy1, cx2, cy2)


def degrade(img, rng, small):
    # brightness/exposure swing (any-light duty)
    g = rng.uniform(0.45, 1.6)
    img = np.clip(img.astype(np.float32) * g, 0, 255).astype(np.uint8)
    if rng.random() < 0.5:  # motion blur (flight vibration)
        k = int(rng.integers(3, 9 if small else 6)) | 1
        ang = rng.uniform(0, 180)
        ker = np.zeros((k, k), np.float32)
        cv2.line(ker, (0, k // 2), (k - 1, k // 2), 1.0, 1)
        M = cv2.getRotationMatrix2D((k / 2, k / 2), ang, 1.0)
        ker = cv2.warpAffine(ker, M, (k, k))
        ker /= ker.sum() + 1e-6
        img = cv2.filter2D(img, -1, ker)
    if rng.random() < 0.3:  # gaussian noise (high gain)
        nz = rng.normal(0, rng.uniform(2, 9), img.shape)
        img = np.clip(img.astype(np.float32) + nz, 0, 255).astype(np.uint8)
    return img


def gen_split(out, split, n, args, rng):
    idir = os.path.join(out, "images", split)
    ldir = os.path.join(out, "labels", split)
    os.makedirs(idir, exist_ok=True)
    os.makedirs(ldir, exist_ok=True)
    for i in range(n):
        bg = make_background(rng, args.img)
        labels = []
        for _ in range(int(rng.integers(1, 4))):
            # bias toward SMALL (altitude duty): sqrt distribution
            side = int(args.px_min + (args.px_max - args.px_min) * (rng.random() ** 1.6))
            qr = render_qr(rand_payload(rng, args.chars), side)
            box = paste_warped(bg, qr, rng)
            if box:
                labels.append(box)
        img = degrade(bg, rng, small=True)
        name = "%s_%05d" % (split, i)
        cv2.imwrite(os.path.join(idir, name + ".jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 88])
        with open(os.path.join(ldir, name + ".txt"), "w") as f:
            for (x1, y1, x2, y2) in labels:
                cx, cy = (x1 + x2) / 2 / args.img, (y1 + y2) / 2 / args.img
                w, h = (x2 - x1) / args.img, (y2 - y1) / args.img
                f.write("0 %.5f %.5f %.5f %.5f\n" % (cx, cy, w, h))
    print("%s: %d images" % (split, n))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="qr_dataset")
    ap.add_argument("--n-train", type=int, default=3000)
    ap.add_argument("--n-val", type=int, default=400)
    ap.add_argument("--img", type=int, default=640)
    ap.add_argument("--px-min", type=int, default=10)
    ap.add_argument("--px-max", type=int, default=140)
    ap.add_argument("--chars", type=int, default=6)
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()
    rng = np.random.default_rng(args.seed)
    random.seed(args.seed)
    gen_split(args.out, "train", args.n_train, args, rng)
    gen_split(args.out, "val", args.n_val, args, rng)
    with open(os.path.join(args.out, "qr.yaml"), "w") as f:
        f.write("path: %s\ntrain: images/train\nval: images/val\nnames:\n  0: qr\n"
                % os.path.abspath(args.out))
    print("wrote %s (qr.yaml + images/labels)" % args.out)


if __name__ == "__main__":
    main()
