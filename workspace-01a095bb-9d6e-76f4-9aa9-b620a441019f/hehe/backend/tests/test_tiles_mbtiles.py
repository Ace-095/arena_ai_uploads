import os
import sqlite3
import sys
import tempfile
import threading
import time
import requests

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, PROJECT_ROOT)
sys.path.insert(0, os.path.join(PROJECT_ROOT, "backend"))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "tools"))

# Set environment variables before importing app
temp_dir = tempfile.mkdtemp()
db_path = os.path.join(temp_dir, "test_field.mbtiles")
os.environ["MBTILES_PATH"] = db_path
os.environ["MAVLINK_DEVICE"] = "udpout:127.0.0.1:14562"
os.environ["MAVLINK_TIMEOUT_S"] = "2.0"

from mock_fc import run_mock_fc
from tools.fetch_tiles import xyz_y_to_tms_y, tms_y_to_xyz_y, latlon_to_xyz_tile, init_mbtiles_db
import uvicorn
import app

SERVER_PORT = 8002


def test_tms_xyz_roundtrip():
    """Verify round-trip TMS <-> XYZ coordinate conversions."""
    print("[Test] Testing TMS <-> XYZ math roundtrip...")
    for z in [0, 5, 10, 17, 18, 20]:
        max_y = (1 << z) - 1
        for y_xyz in [0, 1, max_y // 2, max_y]:
            tms_y = xyz_y_to_tms_y(z, y_xyz)
            recovered_y = tms_y_to_xyz_y(z, tms_y)
            assert recovered_y == y_xyz, f"Failed roundtrip for z={z}, y={y_xyz}"
    print("[Test] TMS <-> XYZ math roundtrip PASSED!")


def test_mbtiles_db_and_endpoint():
    """Verify MBTiles DB creation and FastAPI /tiles/{z}/{x}/{y}.png endpoint."""
    print("[Test] Testing MBTiles DB creation & API endpoint...")

    # Start mock FC thread for MAVLinkBus initialization
    stop_fc = threading.Event()
    fc_thread = threading.Thread(target=run_mock_fc, args=("udp:0.0.0.0:14562", stop_fc), daemon=True)
    fc_thread.start()
    time.sleep(1.0)

    # Initialize MBTiles DB
    conn = init_mbtiles_db(db_path, "77.59,12.97,77.60,12.98")
    cur = conn.cursor()

    # Insert a dummy PNG tile at zoom 18, x=1200, y_xyz=1500
    z = 18
    x_xyz = 1200
    y_xyz = 1500
    tms_y = xyz_y_to_tms_y(z, y_xyz)
    dummy_png_bytes = b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x01\x00\x00\x00\x01\x08\x06\x00\x00\x00\x1f\x15c4"

    cur.execute(
        "INSERT INTO tiles (zoom_level, tile_column, tile_row, tile_data) VALUES (?, ?, ?, ?)",
        (z, x_xyz, tms_y, dummy_png_bytes),
    )
    conn.commit()
    conn.close()

    # Start FastAPI server thread
    def run_server():
        uvicorn.run(app.app, host="127.0.0.1", port=SERVER_PORT, log_level="error")

    srv_thread = threading.Thread(target=run_server, daemon=True)
    srv_thread.start()
    time.sleep(2.0)

    try:
        # 1. Test existing tile endpoint GET /tiles/18/1200/1500.png
        url = f"http://127.0.0.1:{SERVER_PORT}/tiles/{z}/{x_xyz}/{y_xyz}.png"
        r = requests.get(url, timeout=5)
        assert r.status_code == 200, f"Expected 200, got {r.status_code}"
        assert r.content == dummy_png_bytes, "Tile content mismatch"
        assert r.headers["content-type"] == "image/png", f"Header mismatch: {r.headers.get('content-type')}"
        print(f"[Test] GET {url} -> 200 OK (Tile content matched!)")

        # 2. Test missing tile endpoint GET /tiles/18/9999/9999.png -> 404
        missing_url = f"http://127.0.0.1:{SERVER_PORT}/tiles/18/9999/9999.png"
        r_missing = requests.get(missing_url, timeout=5)
        assert r_missing.status_code == 404, f"Expected 404, got {r_missing.status_code}"
        print(f"[Test] GET {missing_url} -> 404 Not Found (Correct!)")

        print("[Test] MBTiles & Tile endpoint tests PASSED successfully!")
        return True
    finally:
        stop_fc.set()
        fc_thread.join(timeout=1.0)
        if os.path.exists(db_path):
            os.remove(db_path)


if __name__ == "__main__":
    test_tms_xyz_roundtrip()
    success = test_mbtiles_db_and_endpoint()
    exit(0 if success else 1)
