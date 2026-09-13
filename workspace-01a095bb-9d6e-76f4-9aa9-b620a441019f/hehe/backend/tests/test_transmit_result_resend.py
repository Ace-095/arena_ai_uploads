"""
test_transmit_result_resend.py — Confirms TRANSMIT_RESULT resends
STATUSTEXT to Mission Planner periodically instead of a single fire-and-
forget attempt, and that it's still bounded (transitions to RTL after
the configured window, doesn't hang forever).

Tests the REAL MissionFSM.start() background thread — not a duplicated
copy of the state logic — so a bug in the actual implementation would
actually be caught here.

Run: python3 tests/test_transmit_result_resend.py
"""

import os
import sys
import time
from unittest.mock import patch, MagicMock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ.setdefault("MAVLINK20", "1")

# Short window so this test runs in a couple seconds, not 15+.
os.environ["STATUSTEXT_RESEND_INTERVAL_S"] = "0.3"
os.environ["STATUSTEXT_RESEND_WINDOW_S"] = "1.0"

import fsm
from config import config
from models import GeofenceStatus
import state


def _setup_armable_fence():
    """_check_failsafe_conditions() checks fence armability on every
    tick for any state outside INIT/PRE_FLIGHT_CHECK/FAILSAFE/RTL/LAND/
    MISSION_COMPLETE. Without this, jumping straight to CONFIRMED (as
    these tests do, to isolate TRANSMIT_RESULT specifically) triggers an
    immediate FAILSAFE via the geofence-unarmable path before
    TRANSMIT_RESULT ever runs — confirmed by reproduction: the first
    version of this test showed 0 send_statustext calls because of
    exactly this."""
    state.set(GeofenceStatus(loaded=True, armable=True, reason="test"))


def main():
    failures = []

    def check(name, fn):
        try:
            fn()
            print(f"PASS  {name}")
        except Exception as e:
            print(f"FAIL  {name}: {e}")
            failures.append(name)

    def do_resends_multiple_times_within_window():
        _setup_armable_fence()
        mock_bus = MagicMock()
        mission = fsm.MissionFSM(mock_bus)
        mission._last_heartbeat = time.time()  # keep the link-health check happy

        with patch("fsm.mf.send_statustext") as mock_send:
            mission._status.payload = "27"
            mission.start()  # runs the REAL background thread
            mission._transition_to("CONFIRMED", "test setup")

            deadline = time.time() + 5.0
            while mission.get_status().state not in ("RTL", "LAND", "MISSION_COMPLETE"):
                if time.time() > deadline:
                    raise AssertionError(
                        f"never left TRANSMIT_RESULT within 5s — got stuck in "
                        f"'{mission.get_status().state}'"
                    )
                time.sleep(0.05)

            time.sleep(0.1)
            call_count = mock_send.call_count

        mission.abort()

        assert call_count >= 2, (
            f"expected send_statustext called multiple times over a "
            f"{config.STATUSTEXT_RESEND_WINDOW_S}s window at "
            f"{config.STATUSTEXT_RESEND_INTERVAL_S}s intervals, but it was only "
            f"called {call_count} time(s) — this is the actual point of the fix, "
            f"a single call would mean the resend logic isn't working."
        )
        print(f"  (send_statustext called {call_count} times over the {config.STATUSTEXT_RESEND_WINDOW_S}s window)")

    def do_stays_bounded_even_if_send_always_fails():
        """If send_statustext raises every time (e.g. link genuinely down
        for the whole window), TRANSMIT_RESULT must still eventually give
        up and transition onward — not get stuck retrying forever."""
        _setup_armable_fence()
        mock_bus = MagicMock()
        mission = fsm.MissionFSM(mock_bus)
        mission._last_heartbeat = time.time()

        with patch("fsm.mf.send_statustext", side_effect=RuntimeError("simulated link down")):
            mission._status.payload = "27"
            mission.start()
            mission._transition_to("CONFIRMED", "test setup")

            deadline = time.time() + 5.0
            while mission.get_status().state not in ("RTL", "LAND", "MISSION_COMPLETE"):
                if time.time() > deadline:
                    raise AssertionError(
                        f"got stuck in '{mission.get_status().state}' even though "
                        f"every send attempt failed — the window bound isn't working"
                    )
                time.sleep(0.05)

        mission.abort()

    check("TRANSMIT_RESULT resends STATUSTEXT multiple times within the window", do_resends_multiple_times_within_window)
    check("TRANSMIT_RESULT still transitions onward even if every send fails", do_stays_bounded_even_if_send_always_fails)

    print()
    if failures:
        print(f"{len(failures)} test(s) failed: {failures}")
        sys.exit(1)
    print("All resend tests passed.")


if __name__ == "__main__":
    main()
