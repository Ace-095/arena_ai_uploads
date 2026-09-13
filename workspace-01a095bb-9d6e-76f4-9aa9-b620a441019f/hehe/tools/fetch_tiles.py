#!/usr/bin/env python3
"""
tools/fetch_tiles.py — Offline MBTiles downloader.

Downloads OpenStreetMap tiles for a specified lat/lon bounding box and zoom range,
storing them in a standard MBTiles SQLite database.

IMPORTANT: MBTiles uses TMS y indexing, where y=0 is at the bottom of the map,
whereas OpenStreetMap / standard Web Mercator XYZ uses y=0 at the top.
Conversion formula:
    tms_y = (2^z - 1) - xyz_y
    xyz_y = (2^z - 1) - tms_y
"""

import argparse
import math
import os
import sqlite3
import time
import urllib.request
import urllib.error
from typing import Tuple


def latlon_to_xyz_tile(lat: float, lon: float, zoom: int) -> Tuple[int, int]:
    """Convert latitude, longitude, and zoom level to standard XYZ tile x, y."""
    lat_rad = math.radians(lat)
    n = 1 << zoom
    xtile = int((lon + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    # Clamp tile coordinates to valid bounds
    xtile = max(0, min(n - 1, xtile))
    ytile = max(0, min(n - 1, ytile))
    return xtile, ytile


def xyz_y_to_tms_y(zoom: int, xyz_y: int) -> int:
    """Convert standard XYZ tile y coordinate to MBTiles TMS tile y coordinate."""
    return (1 << zoom) - 1 - xyz_y


def tms_y_to_xyz_y(zoom: int, tms_y: int) -> int:
    """Convert MBTiles TMS tile y coordinate to standard XYZ tile y coordinate."""
    return (1 << zoom) - 1 - tms_y


def init_mbtiles_db(db_path: str, bbox_str: str, name: str = "Field Tiles") -> sqlite3.Connection:
    """Initialize an MBTiles SQLite database schema."""
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()

    cur.execute("CREATE TABLE IF NOT EXISTS metadata (name TEXT, value TEXT)")
    cur.execute(
        "CREATE TABLE IF NOT EXISTS tiles (zoom_level INTEGER, tile_column INTEGER, tile_row INTEGER, tile_data BLOB)"
    )
    cur.execute(
        "CREATE UNIQUE INDEX IF NOT EXISTS tile_index ON tiles (zoom_level, tile_column, tile_row)"
    )

    metadata = [
        ("name", name),
        ("type", "baselayer"),
        ("version", "1"),
        ("description", "Offline field tiles for autonomous search pipeline"),
        ("format", "png"),
        ("bounds", bbox_str),
    ]

    for key, val in metadata:
        cur.execute(
            "INSERT OR REPLACE INTO metadata (name, value) VALUES (?, ?)", (key, val)
        )

    conn.commit()
    return conn


def fetch_tiles(
    bbox: Tuple[float, float, float, float],
    zoom_min: int,
    zoom_max: int,
    output_file: str,
    tile_url_template: str = "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
    delay_s: float = 0.1,
    user_agent: str = "AeroClub-TileFetcher/1.0",
) -> int:
    """Fetch tiles within bbox across zoom_min to zoom_max and store in MBTiles."""
    min_lon, min_lat, max_lon, max_lat = bbox
    bbox_str = f"{min_lon},{min_lat},{max_lon},{max_lat}"

    conn = init_mbtiles_db(output_file, bbox_str)
    cur = conn.cursor()

    total_downloaded = 0
    total_skipped = 0

    for z in range(zoom_min, zoom_max + 1):
        # Note: min_lat corresponds to max xyz_y, max_lat corresponds to min xyz_y
        x_min, y_max = latlon_to_xyz_tile(min_lat, min_lon, z)
        x_max, y_min = latlon_to_xyz_tile(max_lat, max_lon, z)

        if x_min > x_max:
            x_min, x_max = x_max, x_min
        if y_min > y_max:
            y_min, y_max = y_max, y_min

        count_z = (x_max - x_min + 1) * (y_max - y_min + 1)
        print(f"Zoom {z}: {count_z} tiles (x: {x_min}..{x_max}, y: {y_min}..{y_max})")

        for x in range(x_min, x_max + 1):
            for y in range(y_min, y_max + 1):
                tms_y = xyz_y_to_tms_y(z, y)

                # Check if tile already exists in database
                cur.execute(
                    "SELECT 1 FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?",
                    (z, x, tms_y),
                )
                if cur.fetchone():
                    total_skipped += 1
                    continue

                url = tile_url_template.format(z=z, x=x, y=y)
                req = urllib.request.Request(url, headers={"User-Agent": user_agent})

                try:
                    with urllib.request.urlopen(req, timeout=10) as resp:
                        tile_data = resp.read()

                    cur.execute(
                        "INSERT OR REPLACE INTO tiles (zoom_level, tile_column, tile_row, tile_data) VALUES (?, ?, ?, ?)",
                        (z, x, tms_y, tile_data),
                    )
                    conn.commit()
                    total_downloaded += 1
                    time.sleep(delay_s)
                except Exception as e:
                    print(f"Failed to fetch tile z={z} x={x} y={y} ({url}): {e}")

    conn.close()
    print(f"Finished! Downloaded: {total_downloaded}, Skipped (already existed): {total_skipped}")
    return total_downloaded


def main():
    parser = argparse.ArgumentParser(description="Fetch OSM tiles into an MBTiles file.")
    parser.add_argument(
        "--bbox",
        type=str,
        required=True,
        help="Bounding box min_lon,min_lat,max_lon,max_lat (e.g. 77.590,12.970,77.595,12.975)",
    )
    parser.add_argument("--zoom-min", type=int, default=17, help="Minimum zoom level (default: 17)")
    parser.add_argument("--zoom-max", type=int, default=20, help="Maximum zoom level (default: 20)")
    parser.add_argument("--out", type=str, default="field.mbtiles", help="Output MBTiles file path")
    parser.add_argument(
        "--tile-url",
        type=str,
        default="https://tile.openstreetmap.org/{z}/{x}/{y}.png",
        help="Tile server URL template",
    )
    parser.add_argument("--delay", type=float, default=0.1, help="Delay between requests in seconds")

    args = parser.parse_args()

    parts = [float(p.strip()) for p in args.bbox.split(",")]
    if len(parts) != 4:
        raise ValueError("--bbox must be 4 comma-separated floats: min_lon,min_lat,max_lon,max_lat")
    bbox = (parts[0], parts[1], parts[2], parts[3])

    fetch_tiles(
        bbox=bbox,
        zoom_min=args.zoom_min,
        zoom_max=args.zoom_max,
        output_file=args.out,
        tile_url_template=args.tile_url,
        delay_s=args.delay,
    )


if __name__ == "__main__":
    main()
