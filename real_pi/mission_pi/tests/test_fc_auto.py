#!/usr/bin/env python3
"""Stdlib tests for the real_pi Pixhawk USB auto-connect (fc_link).

No pymavlink, no hardware: the glob is faked over a temp dir of real files
(so the permission checks see real nodes) and find_fc is monkeypatched with
a duck-typed connection stub.

Locked down:
  * candidate ordering: ArduPilot/Pixhawk by-id FIRST (stable across
    re-plugs), then other by-id, then ttyACM (a direct-USB Pixhawk IS a
    ttyACM) before ttyUSB (dongles);
  * unreadable devices are logged with the dialout/udev hint but still
    listed (they may become readable mid-flight);
  * reconnect(): if the old serial device has VANISHED (USB replug on a
    new port / Pi reboot), the link auto-RE-DETECTS instead of retrying
    a dead node forever — the real-flight "link just died" fix;
  * reconnect(): a network device (SITL tcp) is never "re-detect"ed —
    pymavlink autoreconnect handles those.
"""
import glob as real_glob
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                ".."))
import fc_link  # noqa

try:
    import pymavlink  # noqa
    fc_link._require_pymavlink()
except Exception:                       # noqa: BLE001
    fc_link._require_pymavlink = lambda: None
    print("INFO pymavlink not installed — stubbing the import guard")

FAILS = []


def check(name, cond, extra=""):
    print(("PASS " if cond else "FAIL ") + name, extra if not cond else "")
    if not cond:
        FAILS.append(name)


def touch(d, p):
    fp = os.path.join(d, p)
    os.makedirs(os.path.dirname(fp), exist_ok=True)
    with open(fp, "w") as f:
        f.write("")
    return fp


class FakeGlob:
    def __init__(self, root):
        self.root = root

    def glob(self, pattern):
        pat = pattern.replace("/dev/", self.root + "/")
        return sorted(real_glob.glob(pat))


def make_devices(root):
    """A plausible /dev picture: Pixhawk by-id + stray dongle + ttyACM/USB."""
    touch(root, "serial/by-id/usb-ArduPilot_ArduPilot_Pixhawk-001-if00")
    touch(root, "serial/by-id/usb-FTDI_FT232R-0123")
    touch(root, "ttyACM0")
    touch(root, "ttyUSB2")
    return [
        root + "/serial/by-id/usb-ArduPilot_ArduPilot_Pixhawk-001-if00",
        root + "/serial/by-id/usb-FTDI_FT232R-0123",
        root + "/ttyACM0",
        root + "/ttyUSB2",
    ]


def test_candidate_order():
    root = tempfile.mkdtemp(prefix="fcauto-")
    expected = make_devices(root)
    real = fc_link.glob
    fc_link.glob = FakeGlob(root)
    try:
        devs = fc_link.candidate_devices()
    finally:
        fc_link.glob = real
    check("order: ardupilot by-id first",
          devs and "ArduPilot" in os.path.basename(devs[0]), repr(devs))
    check("order: fti by-id second",
          len(devs) > 1 and "FTDI" in os.path.basename(devs[1]), repr(devs))
    check("order: ttyACM before ttyUSB",
          devs.count(root + "/ttyACM0") == 1
          and devs.count(root + "/ttyUSB2") == 1
          and devs.index(root + "/ttyACM0") < devs.index(root + "/ttyUSB2"),
          repr(devs))
    check("all devices present",
          all(e in devs for e in expected), repr((devs, expected)))


def test_unreadable_device_still_listed_with_hint():
    import logging
    root = tempfile.mkdtemp(prefix="fcauto2-")
    p = touch(root, "ttyACM9")
    real_glob_attr = fc_link.glob
    real_access = fc_link.os.access
    # force "no permission" deterministically (root would pass os.access)
    fc_link.os.access = lambda path, mode: False if path == p else True
    fc_link.glob = FakeGlob(root)
    msgs = []
    handler = logging.Handler()
    handler.emit = lambda r: msgs.append(r.getMessage())
    lg = logging.getLogger("fc_link")
    old = lg.level
    lg.addHandler(handler)
    lg.setLevel(logging.DEBUG)
    try:
        devs = fc_link.candidate_devices()
    finally:
        lg.removeHandler(handler)
        lg.setLevel(old)
        fc_link.glob = real_glob_attr
        fc_link.os.access = real_access
    check("unreadable: still a candidate", p in devs, repr(devs))
    check("unreadable: dialout/udev hint logged",
          any("dialout" in m for m in msgs), repr(msgs))


class StubConn:
    def __init__(self, sysid=1):
        self.target_system = sysid
        self.target_component = 1

    def close(self):
        pass


def test_reconnect_redetects_when_device_vanished():
    """Device node GONE (USB replug on a new port / Pi reboot): straight
    to auto re-detect, one find_fc call with device=None."""
    link = fc_link.FCLink()
    calls = []

    def fake_find_fc(bauds, device):
        calls.append(device)
        return StubConn(), "/dev/ttyACM1"   # the re-plug landed elsewhere

    real = fc_link.find_fc
    fc_link.find_fc = fake_find_fc
    for m in ("_read_loop", "_hb_loop", "_stream_loop"):
        setattr(fc_link.FCLink, m, lambda self: None)
    try:
        link._bauds = (115200,)
        link.device = "/dev/ttyACM0"        # does not exist on disk
        dev = link.reconnect()
    finally:
        fc_link.find_fc = real
    check("reconnect(vanished): straight to auto (device=None)",
          calls == [None], repr(calls))
    check("reconnect(vanished): acquired the new port",
          dev == "/dev/ttyACM1", dev)
    check("reconnect(vanished): counter bumped", link.reconnects == 1)


def test_reconnect_device_present_but_deaf():
    """Device node PRESENT but no heartbeat (FC rebooted on same port):
    retry the device once, THEN auto re-detect."""
    link = fc_link.FCLink()
    calls = []

    def fake_find_fc(bauds, device):
        calls.append(device)
        if device == tmp.name:
            raise RuntimeError("no FC heartbeat; tried: %s" % device)
        return StubConn(), "/dev/ttyACM1"

    real = fc_link.find_fc
    fc_link.find_fc = fake_find_fc
    for m in ("_read_loop", "_hb_loop", "_stream_loop"):
        setattr(fc_link.FCLink, m, lambda self: None)
    tmp = tempfile.NamedTemporaryFile(prefix="ttyACM0-")
    try:
        link._bauds = (115200,)
        link.device = tmp.name
        dev = link.reconnect()
    finally:
        fc_link.find_fc = real
        tmp.close()
    check("reconnect(deaf): device first, then auto fallback",
          calls == [tmp.name, None], repr(calls))
    check("reconnect(deaf): acquired via auto", dev == "/dev/ttyACM1", dev)


def test_reconnect_same_device_fast_path():
    link = fc_link.FCLink()
    calls = []

    def fake_find_fc(bauds, device):
        calls.append(device)
        return StubConn(), device

    real = fc_link.find_fc
    fc_link.find_fc = fake_find_fc
    for m in ("_read_loop", "_hb_loop", "_stream_loop"):
        setattr(fc_link.FCLink, m, lambda self: None)
    tmp = tempfile.NamedTemporaryFile(prefix="ttyACM0-")  # a "present" node
    try:
        link._bauds = (115200,)
        link.device = tmp.name
        dev = link.reconnect()
    finally:
        fc_link.find_fc = real
        tmp.close()
    check("reconnect: same device first (one call, no auto fallback)",
          calls == [tmp.name], repr(calls))
    check("reconnect: kept the same device", dev == tmp.name)


def test_reconnect_network_device_no_redetect():
    link = fc_link.FCLink()
    calls = []

    def fake_find_fc(bauds, device):
        calls.append(device)
        return StubConn(), device

    real = fc_link.find_fc
    fc_link.find_fc = fake_find_fc
    for m in ("_read_loop", "_hb_loop", "_stream_loop"):
        setattr(fc_link.FCLink, m, lambda self: None)
    try:
        link._bauds = (115200,)
        link.device = "tcp:127.0.0.1:5762"
        dev = link.reconnect()
    finally:
        fc_link.find_fc = real
    check("reconnect: tcp device never re-detects (autoreconnect owns it)",
          calls == ["tcp:127.0.0.1:5762"], repr(calls))
    check("reconnect: tcp device kept", dev == "tcp:127.0.0.1:5762")


if __name__ == "__main__":
    test_candidate_order()
    test_unreadable_device_still_listed_with_hint()
    test_reconnect_redetects_when_device_vanished()
    test_reconnect_device_present_but_deaf()
    test_reconnect_same_device_fast_path()
    test_reconnect_network_device_no_redetect()
    print("-" * 60)
    if FAILS:
        print("FAILED %d/%d: %s" % (len(FAILS), 6, ", ".join(FAILS)))
        sys.exit(1)
    print("all fc-auto tests passed")
