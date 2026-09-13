"""
network_monitor.py — internet connectivity status (for the LTE dongle /
3rd comms leg from the original project notes).

INVARIANT: this is purely informational. Nothing safety-critical reads
this value — MAVLink is over USB, the browser UI is over the local LAN
via WebSocket/WebRTC. Internet availability (or lack of it) must never
gate arming, flight, or any FSM state transition. This module exists
only to drive the UI's "Internet status" indicator from the original
problem statement's spec.
"""

import socket
from typing import Optional


def check_internet_status(timeout_s: float = 1.0, host: str = "8.8.8.8", port: int = 53) -> bool:
    """
    Attempt a raw TCP connection to a well-known, reliable host (Google's
    public DNS, port 53) rather than an HTTP request — cheaper, doesn't
    depend on DNS resolution working, and doesn't touch any particular
    website's uptime. Standard technique for "is there a route out",
    not specific to this project.

    Returns False on ANY failure (timeout, no route, DNS down, etc.) —
    deliberately doesn't distinguish failure modes, since the only thing
    the UI needs is up/down for a status indicator, not a full diagnosis.
    """
    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return True
    except OSError:
        return False
