#!/usr/bin/env python3
"""Make the A3 QR test panel PNG (print it, and reuse it as the Gazebo texture).

  pip install qrcode pillow
  python tools/make_qr_panel.py --text MISSION-QR-001 --out panel.png
  # print panel.png at 100% on A3 (297 x 420 mm portrait), lay it flat.

The PNG is rendered at 150 DPI in exact A3 ratio with a quiet zone,
a black border (helps detectors find the panel edges), and a caption.
"""
import argparse
import os
import sys

A3_W_MM, A3_H_MM = 297.0, 420.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--text", default="MISSION-QR-001")
    ap.add_argument("--out", default="qr_panel_a3.png")
    ap.add_argument("--dpi", type=int, default=150)
    args = ap.parse_args()
    try:
        import qrcode
        from PIL import Image, ImageDraw
    except Exception:
        print("need: pip install qrcode pillow")
        return 1

    w = int(A3_W_MM / 25.4 * args.dpi)
    h = int(A3_H_MM / 25.4 * args.dpi)
    qr = qrcode.QRCode(box_size=10, border=4)
    qr.add_data(args.text)
    qr.make(fit=True)
    code = qr.make_image(fill_color="black", back_color="white").convert("RGB")

    panel = Image.new("RGB", (w, h), "white")
    side = int(min(w, h) * 0.72)          # QR side, quiet zone included
    code = code.resize((side, side), Image.NEAREST)
    panel.paste(code, ((w - side) // 2, int(h * 0.08)))
    d = ImageDraw.Draw(panel)
    bw = max(3, args.dpi // 25)           # black border
    d.rectangle([bw // 2, bw // 2, w - bw // 2 - 1, h - bw // 2 - 1],
                outline="black", width=bw)
    d.text((w // 2 - len(args.text) * 4, int(h * 0.90)), args.text, fill="black")
    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    panel.save(args.out, dpi=(args.dpi, args.dpi))
    print("wrote %s (%dx%d px, %.0fx%.0f mm @ %d dpi): %r" % (
        args.out, w, h, A3_W_MM, A3_H_MM, args.dpi, args.text))
    print("print at 100%% scale on A3 and lay flat; expect relay text: QR:%s" % args.text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
