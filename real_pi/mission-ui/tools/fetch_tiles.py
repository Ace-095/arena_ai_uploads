#!/usr/bin/env python3
"""
tools/fetch_tiles.py — Offline MBTiles downloader (venue map prep).

Run this ONCE on any machine WITH internet, then copy the .mbtiles file to the
competition laptop (which is OFFLINE). The bridge serves it at /tiles/{z}/{x}/{y}.png.

    python tools/fetch_tiles.py --bbox "75.118,15.365,75.129,15.374" --zoom-min 15 --zoom-max 19 --out field.mbtiles

Ported from hehe/tools/fetch_tiles.py (reference only — math re-verified here):
MBTiles uses TMS y (0 = bottom) while OSM/XYZ uses y-from-top:
    tms_y = (2^z - 1) - xyz_y
"""
import argparse
import math
import sqlite3
import time
import urllib.request


def latlon_to_xyz_tile(lat, lon, zoom):
    lat_rad = math.radians(lat)
    n = 1 << zoom
    xtile = int((lon + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return max(0, min(n - 1, xtile)), max(0, min(n - 1, ytile))


def xyz_y_to_tms_y(zoom, xyz_y):
    return (1 << zoom) - 1 - xyz_y


def init_mbtiles_db(db_path, bbox_str, name="Mission Companion field tiles"):
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    cur.execute("CREATE TABLE IF NOT EXISTS metadata (name TEXT, value TEXT)")
    cur.execute("CREATE TABLE IF NOT EXISTS tiles (zoom_level INTEGER, tile_column INTEGER, tile_row INTEGER, tile_data BLOB)")
    cur.execute("CREATE UNIQUE INDEX IF NOT EXISTS tile_index ON tiles (zoom_level, tile_column, tile_row)")
    for key, val in [("name", name), ("type", "baselayer"), ("version", "1"),
                     ("description", "Offline venue tiles (see README offline-map section)"),
                     ("format", "png"), ("bounds", bbox_str)]:
        cur.execute("INSERT OR REPLACE INTO metadata (name, value) VALUES (?, ?)", (key, val))
    conn.commit()
    return conn


def fetch_tiles(bbox, zoom_min, zoom_max, output_file,
                tile_url_template="https://tile.openstreetmap.org/{z}/{x}/{y}.png",
                delay_s=0.15, user_agent="MissionCompanion-TileFetcher/1.0"):
    min_lon, min_lat, max_lon, max_lat = bbox
    conn = init_mbtiles_db(output_file, "%s,%s,%s,%s" % bbox)
    cur = conn.cursor()
    downloaded, skipped = 0, 0
    for z in range(zoom_min, zoom_max + 1):
        x_min, y_max = latlon_to_xyz_tile(min_lat, min_lon, z)
        x_max, y_min = latlon_to_xyz_tile(max_lat, max_lon, z)
        x_min, x_max = min(x_min, x_max), max(x_min, x_max)
        y_min, y_max = min(y_min, y_max), max(y_min, y_max)
        print("Zoom %d: %d tiles (x %d..%d, y %d..%d)" % (z, (x_max - x_min + 1) * (y_max - y_min + 1), x_min, x_max, y_min, y_max))
        for x in range(x_min, x_max + 1):
            for y in range(y_min, y_max + 1):
                tms_y = xyz_y_to_tms_y(z, y)
                cur.execute("SELECT 1 FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?", (z, x, tms_y))
                if cur.fetchone():
                    skipped += 1
                    continue
                url = tile_url_template.format(z=z, x=x, y=y)
                tile_data, err = None, None
                for attempt in range(3):  # OSM throttles/drops: retry, then re-run resumes gaps
                    try:
                        req = urllib.request.Request(url, headers={"User-Agent": user_agent})
                        with urllib.request.urlopen(req, timeout=15) as resp:
                            tile_data = resp.read()
                        break
                    except Exception as e:
                        err = e
                        time.sleep(delay_s * (attempt + 1))
                if tile_data is None:
                    print("  FAIL z=%d x=%d y=%d: %s" % (z, x, y, err))
                    continue
                cur.execute("INSERT OR REPLACE INTO tiles (zoom_level, tile_column, tile_row, tile_data) VALUES (?, ?, ?, ?)",
                            (z, x, tms_y, tile_data))
                conn.commit()
                downloaded += 1
                time.sleep(delay_s)
    conn.close()
    print("Done. downloaded=%d skipped=%d -> %s" % (downloaded, skipped, output_file))
    return downloaded


def main():
    ap = argparse.ArgumentParser(description="Fetch OSM tiles into an MBTiles file (run while online).")
    ap.add_argument("--bbox", required=True, help="min_lon,min_lat,max_lon,max_lat")
    ap.add_argument("--zoom-min", type=int, default=15)
    ap.add_argument("--zoom-max", type=int, default=19)
    ap.add_argument("--out", default="field.mbtiles")
    ap.add_argument("--tile-url", default="https://tile.openstreetmap.org/{z}/{x}/{y}.png")
    ap.add_argument("--delay", type=float, default=0.15)
    a = ap.parse_args()
    parts = [float(p.strip()) for p in a.bbox.split(",")]
    assert len(parts) == 4, "--bbox must be 4 floats"
    fetch_tiles(tuple(parts), a.zoom_min, a.zoom_max, a.out, a.tile_url, a.delay)


if __name__ == "__main__":
    main()
