#!/usr/bin/env python3
"""
generate_qr_texture.py — generate a decodable QR PNG for the Gazebo qr_box model.

Run this script ONCE to produce the texture file that the qr_box Gazebo model
applies as its material. The resulting PNG is committed to version control so
the Gazebo world can be launched without running this script each time.

Usage:
    cd sim/textures
    python3 generate_qr_texture.py

    # Custom payload (defaults to SYNTHETIC_QR_PAYLOAD env var or fallback string)
    SYNTHETIC_QR_PAYLOAD="MY_MISSION_CODE" python3 generate_qr_texture.py

Output:
    ../models/qr_box/materials/textures/qr_code.png

The payload string baked into this texture does NOT need to match the real
competition payload — the simulation only needs a real, decodable QR code
to test that the vision pipeline can detect and decode correctly. The
competition QR payload format is NEED TEAM INPUT.

Dependencies (all in requirements.txt):
    qrcode[pil]>=7.4
    Pillow>=10.0
"""

import os
import sys

# Default payload — override via env var
PAYLOAD = os.environ.get("SYNTHETIC_QR_PAYLOAD", "CACHE_QR_GAZEBO_SIM_V1")

# Output path relative to this script
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_PATH = os.path.join(
    SCRIPT_DIR, "..", "models", "qr_box", "materials", "textures", "qr_code.png"
)
OUTPUT_PATH = os.path.normpath(OUTPUT_PATH)

# Texture resolution. 512x512 is plenty for a 1m box model viewed from 5-20m.
TEXTURE_SIZE = 512

# White border fraction around the QR code (helps the detector locate corners)
BORDER_FRACTION = 0.1


def generate(payload: str, output_path: str, texture_size: int, border_fraction: float) -> None:
    import cv2
    import numpy as np

    # Generate QR matrix using OpenCV
    try:
        encoder = cv2.QRCodeEncoder.create()
        qr_matrix = encoder.encode(payload)
    except AttributeError:
        try:
            import qrcode
            qr = qrcode.QRCode(
                version=None,
                error_correction=qrcode.constants.ERROR_CORRECT_M,
                box_size=10,
                border=4,
            )
            qr.add_data(payload)
            qr.make(fit=True)
            qr_img = qr.make_image(fill_color="black", back_color="white").convert("L")
            qr_matrix = np.array(qr_img, dtype=np.uint8)
        except ImportError as e:
            print(f"ERROR: Missing dependency — {e}")
            sys.exit(1)

    # Convert to 0/255 if binary 0/1
    if qr_matrix.max() == 1:
        qr_matrix = qr_matrix * 255

    # Create canvas with white border
    canvas_size = texture_size
    border_px = int(canvas_size * border_fraction)
    inner_size = canvas_size - 2 * border_px

    # Background: white
    canvas = np.full((canvas_size, canvas_size, 3), 255, dtype=np.uint8)

    # Resize QR to inner size using nearest neighbor
    qr_resized = cv2.resize(qr_matrix, (inner_size, inner_size), interpolation=cv2.INTER_NEAREST)
    if qr_resized.ndim == 2:
        qr_bgr = cv2.cvtColor(qr_resized, cv2.COLOR_GRAY2BGR)
    else:
        qr_bgr = qr_resized

    canvas[border_px:border_px + inner_size, border_px:border_px + inner_size] = qr_bgr

    # Save PNG
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    cv2.imwrite(output_path, canvas)
    print(f"Generated QR texture: {output_path}")
    print(f"  Payload: {payload}")
    print(f"  Size:    {canvas_size}x{canvas_size} px")
    print()
    print("Verification: you can decode the QR from the command line with:")
    print(f"  zbarimg {output_path}")
    print("  — or open the PNG in any QR scanner app to confirm it decodes correctly.")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Generate QR texture for Gazebo qr_box model")
    parser.add_argument("--payload", default=PAYLOAD, help="QR code data string")
    parser.add_argument("--output", default=OUTPUT_PATH, help="Output PNG path")
    parser.add_argument("--size", type=int, default=TEXTURE_SIZE, help="Texture resolution (px)")
    args = parser.parse_args()

    generate(
        payload=args.payload,
        output_path=args.output,
        texture_size=args.size,
        border_fraction=BORDER_FRACTION,
    )
