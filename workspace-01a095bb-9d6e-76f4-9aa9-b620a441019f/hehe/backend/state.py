"""
Holds the Pi-side view of "what fence is currently loaded" for the UI to
poll. This is a convenience cache, not a safety record — if the Pi
restarts, the Pixhawk's own fence storage is untouched and remains the
authoritative safety boundary regardless of what's in this file.
"""

import json
import os
import threading
from typing import Optional

from models import GeofenceStatus


class FenceState:
    def __init__(self, state_file: str):
        self._lock = threading.Lock()
        self._state_file = state_file
        self._status = GeofenceStatus(loaded=False, armable=False, reason="No fence uploaded yet")
        self._load()

    def _load(self) -> None:
        if os.path.exists(self._state_file):
            try:
                with open(self._state_file, "r") as f:
                    data = json.load(f)
                self._status = GeofenceStatus(**data)
            except Exception:
                # Corrupt/old state file — treat as "no fence", never guess.
                self._status = GeofenceStatus(
                    loaded=False, armable=False, reason="State file unreadable, treating as no fence"
                )

    def _save(self) -> None:
        with open(self._state_file, "w") as f:
            f.write(self._status.model_dump_json())

    def get(self) -> GeofenceStatus:
        with self._lock:
            return self._status

    def set(self, status: GeofenceStatus) -> None:
        with self._lock:
            self._status = status
            self._save()

    def clear(self) -> None:
        with self._lock:
            self._status = GeofenceStatus(loaded=False, armable=False, reason="Fence cleared")
            self._save()


from config import config

_state = FenceState(config.STATE_FILE)


def get() -> GeofenceStatus:
    return _state.get()


def set(status: GeofenceStatus) -> None:
    _state.set(status)


def clear() -> None:
    _state.clear()

