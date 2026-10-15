#!/usr/bin/env python3
"""Regression test: only mock serves Pi-side WS routes (no fake-open).

Real-mode handle_ws used to 101-accept /ws/telemetry and then never send
a byte: the UI showed a green "open" lamp with zero data, and the
connect button (connected => disconnect) trapped users at "closed by
user". ws_servable() is the gate; handle_ws 404s the rest. Stdlib only.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "bridge"))

from mp_bridge import ws_servable


def main():
    assert ws_servable("/ws/telemetry", True) is True
    assert ws_servable("/ws/telemetry", False) is False
    assert ws_servable("/ws/webrtc/cam1", True) is True
    assert ws_servable("/ws/webrtc/cam2", True) is True
    assert ws_servable("/ws/webrtc/cam1", False) is False
    assert ws_servable("/ws/nope", True) is False
    assert ws_servable("/ws/nope", False) is False
    print("WSSERV-TEST PASS")


if __name__ == "__main__":
    main()
