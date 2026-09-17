#!/usr/bin/env python3
"""Regression test: LogTailer.poll_once() must survive dirs + files.

The os.isdir typo killed the mp-tailer thread on Windows (where the MP log
dir exists) while empty watch lists on dev machines hid it. Stdlib only.
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "bridge"))

from mp_bridge import LogTailer


class StubHub:
    def __init__(self):
        self.events = []

    def emit_threadsafe(self, event, data):
        self.events.append((event, data))


def main():
    hub = StubHub()
    tailer = LogTailer(hub)
    with tempfile.TemporaryDirectory() as td:
        logf = os.path.join(td, "mp.log")
        with open(logf, "w") as f:
            f.write("old line\n")
        assert tailer.add_path(td) is True
        assert tailer.add_path(td) is False  # dup ignored
        tailer.poll_once()  # arms offsets at EOF — must not raise
        assert hub.events == [], hub.events
        with open(logf, "a") as f:
            f.write("MAVLink: heartbeat ok\nSTATUSTEXT QR:42 landed\n")
        tailer.poll_once()
        lines = [d["line"] for e, d in hub.events if e == "mp-line"]
        qrs = [(d["payload"], d["source"]) for e, d in hub.events if e == "mp-qr"]
        assert len(lines) == 2, hub.events
        assert qrs == [("42", "mp.log")], hub.events
        hub.events.clear()
        tailer.add_path(logf)  # a file path (not dir) also works
        with open(logf, "a") as f:
            f.write("one more\n")
        tailer.poll_once()
        assert any(d.get("line") == "one more" for e, d in hub.events if e == "mp-line"), hub.events
    print("TAILER-TEST PASS")


if __name__ == "__main__":
    main()
