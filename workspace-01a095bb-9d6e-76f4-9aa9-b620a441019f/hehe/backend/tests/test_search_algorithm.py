"""
Unit and dry-run acceptance test for search algorithm (Phase 6).

Run from backend/ directory:
    python3 tests/test_search_algorithm.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from geo_utils import ENU, latlon_to_enu, point_in_polygon
from models import ENUPoint, GeofenceStatus, Vertex
from search_algorithm import SearchController, LawnmowerStrategy, ExpandingSquareStrategy
import state

ORIGIN_LAT = 12.971598
ORIGIN_LON = 77.594562


def setup_dummy_fence():
    """Create a 50m x 50m square polygon around EKF origin."""
    # Vertices ~25m East/North of origin
    vertices = [
        Vertex(lat=ORIGIN_LAT + 0.000225, lon=ORIGIN_LON + 0.000225),  # NE
        Vertex(lat=ORIGIN_LAT + 0.000225, lon=ORIGIN_LON - 0.000225),  # NW
        Vertex(lat=ORIGIN_LAT - 0.000225, lon=ORIGIN_LON - 0.000225),  # SW
        Vertex(lat=ORIGIN_LAT - 0.000225, lon=ORIGIN_LON + 0.000225),  # SE
    ]

    vertices_enu = [latlon_to_enu(v.lat, v.lon, ORIGIN_LAT, ORIGIN_LON) for v in vertices]
    polygon_nodes = [ENU(p.x, p.y) for p in vertices_enu]

    status = GeofenceStatus(
        loaded=True,
        armable=True,
        reason="Dummy fence for unit test",
        vertex_count=len(vertices),
        vertices_latlon=vertices,
        vertices_enu=[ENUPoint(x=p.x, y=p.y) for p in vertices_enu],
        centroid_enu=ENUPoint(x=0.0, y=0.0),
        max_radius_m=35.0,
        origin_lat=ORIGIN_LAT,
        origin_lon=ORIGIN_LON,
        fc_readback_matched=True,
    )
    state.set(status)
    return polygon_nodes, status


def test_expanding_square_containment():
    polygon_nodes, fence = setup_dummy_fence()
    controller = SearchController()

    waypoints = controller.run_dry_run(strategy_name="expanding_square", altitude_m=5.0, step_m=3.0)
    assert len(waypoints) > 0, "Expanding square should generate waypoints"

    for wp in waypoints:
        pt = ENU(wp.enu.x, wp.enu.y)
        assert point_in_polygon(pt, polygon_nodes), (
            f"WP #{wp.index} ENU=({wp.enu.x:.2f}, {wp.enu.y:.2f}) strictly OUTSIDE polygon!"
        )
        dist = math.hypot(wp.enu.x, wp.enu.y)
        assert dist <= fence.max_radius_m + 0.1, (
            f"WP #{wp.index} dist={dist:.2f}m exceeds max_radius={fence.max_radius_m}m!"
        )

    print(f"PASS: Expanding square dry-run generated {len(waypoints)} valid waypoints inside polygon.")


def test_lawnmower_containment():
    polygon_nodes, fence = setup_dummy_fence()
    controller = SearchController()

    waypoints = controller.run_dry_run(strategy_name="lawnmower", altitude_m=5.0, step_m=4.0)
    assert len(waypoints) > 0, "Lawnmower should generate waypoints"

    for wp in waypoints:
        pt = ENU(wp.enu.x, wp.enu.y)
        assert point_in_polygon(pt, polygon_nodes), (
            f"Lawnmower WP #{wp.index} ENU=({wp.enu.x:.2f}, {wp.enu.y:.2f}) OUTSIDE polygon!"
        )
        dist = math.hypot(wp.enu.x, wp.enu.y)
        assert dist <= fence.max_radius_m + 0.1, (
            f"Lawnmower WP #{wp.index} dist={dist:.2f}m exceeds max_radius={fence.max_radius_m}m!"
        )

    print(f"PASS: Lawnmower dry-run generated {len(waypoints)} valid waypoints inside polygon.")


def test_fail_closed_unarmable():
    state.clear()  # No fence loaded -> armable=False
    controller = SearchController()

    try:
        controller.run_dry_run()
        assert False, "Expected RuntimeError when fence is not armable"
    except RuntimeError as e:
        print(f"PASS: Fail-closed on unarmable fence caught correctly ({e}).")


def test_on_detection():
    setup_dummy_fence()
    controller = SearchController()
    controller.run_dry_run()

    bbox = {"x": 100, "y": 100, "w": 50, "h": 50}
    controller.on_detection(bbox, 0.92)

    status = controller.get_status()
    assert status.state == "target_found", f"Expected status 'target_found', got '{status.state}'"
    assert status.target is not None and status.target["confidence"] == 0.92
    print("PASS: on_detection() callback updated controller state correctly.")


def main():
    failures = []
    tests = [
        ("expanding_square_containment", test_expanding_square_containment),
        ("lawnmower_containment", test_lawnmower_containment),
        ("fail_closed_unarmable", test_fail_closed_unarmable),
        ("on_detection_callback", test_on_detection),
    ]

    for name, test_fn in tests:
        try:
            test_fn()
        except Exception as e:
            print(f"FAIL  {name}: {e}")
            failures.append(name)

    if failures:
        print(f"\n{len(failures)} test(s) failed: {failures}")
        sys.exit(1)

    print("\nAll Search Algorithm unit & dry-run acceptance tests passed!")


if __name__ == "__main__":
    main()
