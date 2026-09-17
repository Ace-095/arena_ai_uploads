#!/usr/bin/env python3
"""Make the A3 QR test panel PNG (print it, and reuse it as the Gazebo texture).

  pip install qrcode pillow
  python tools/make_qr_panel.py --text MISSION-QR-001 --out panel.png
  # print panel.png at 100% on A3 (297 x 420 mm portrait), lay it flat.

The PNG is rendered at 150 DPI in exact A3 ratio with a quiet zone,
a black border (helps detectors find the panel edges), and a caption.

NEW: --full flag makes QR cover ENTIRE A3 (95% of panel), not QR in A3 with big white border.
User requested: QR should be entire A3, not small QR in white A3 — white part big, QR small is wrong.
--full makes QR fill entire A3 white part, so Gazebo A3 detection is honest.

  python tools/make_qr_panel.py --text MISSION-QR-001 --out sim/models/qr_panel_a3/materials/textures/qr.png --full
  python tools/make_qr_panel.py --text MISSION-QR-001 --out sim/models/qr_panel_big/materials/textures/qr.png --full --dpi 300
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
    ap.add_argument("--full", action="store_true", help="QR covers ENTIRE A3 (95% fill), not QR in A3 with white border — user requested entire A3 is QR")
    ap.add_argument("--border", type=int, default=4, help="QR border quiet zone (default 4, full mode uses 2)")
    args = ap.parse_args()
    try:
        import qrcode
        from PIL import Image, ImageDraw
    except Exception:
        print("need: pip install qrcode pillow")
        return 1

    w = int(A3_W_MM / 25.4 * args.dpi)
    h = int(A3_H_MM / 25.4 * args.dpi)
    
    if args.full:
        # FULL A3 QR — QR covers entire white part, 95% fill, minimal white border
        # User: QR should be entire A3, not QR in A3
        qr = qrcode.QRCode(box_size=10, border=args.border if args.border != 4 else 2)  # small border for full
        qr.add_data(args.text)
        qr.make(fit=True)
        code = qr.make_image(fill_color="black", back_color="white").convert("RGB")
        # Resize to 95% of A3 panel — QR IS the A3, not inside A3
        side_w = int(w * 0.95)
        side_h = int(h * 0.95)
        # For A3 portrait 297x420, QR should be square covering max possible
        side = min(side_w, side_h)
        code = code.resize((side, side), Image.NEAREST)
        panel = Image.new("RGB", (w, h), "white")
        # Center QR, covering entire white part
        panel.paste(code, ((w - side) // 2, (h - side) // 2))
        # Thin black border for panel edge detection (not big white)
        d = ImageDraw.Draw(panel)
        bw = max(2, args.dpi // 50)
        d.rectangle([bw // 2, bw // 2, w - bw // 2 - 1, h - bw // 2 - 1], outline="black", width=bw)
        print(f"FULL A3 mode: QR covers 95% of A3 panel {w}x{h}px — QR IS entire A3")
    else:
        # Old mode: QR in A3 with big white border (72% fill)
        qr = qrcode.QRCode(box_size=10, border=args.border)
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
        print(f"Standard mode: QR 72% of A3 panel {w}x{h}px — QR in A3 with white border")

    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    panel.save(args.out, dpi=(args.dpi, args.dpi))
    print("wrote %s (%dx%d px, %.0fx%.0f mm @ %d dpi): %r" % (
        args.out, w, h, A3_W_MM, A3_H_MM, args.dpi, args.text))
    print("print at 100%% scale on A3 and lay flat; expect relay text: QR:%s" % args.text)
    if args.full:
        print("FULL mode: QR covers entire A3 white part — for Gazebo honest A3 @15m YOLO test")
    return 0


if __name__ == "__main__":
    sys.exit(main())
