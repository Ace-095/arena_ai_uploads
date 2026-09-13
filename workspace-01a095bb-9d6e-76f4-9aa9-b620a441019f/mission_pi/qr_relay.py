"""QR result delivery on two routes (fire both, either can win):

1. MAVLink STATUSTEXT  `QR:<payload>` -> flight controller -> forwarded on
   the telemetry stream -> Mission Planner Messages tab (and mission-ui's
   MAVLink QR catcher). Resent every `interval_s` for `window_s` because a
   single 50-byte text can drown in telemetry traffic.
2. Pi websocket event  channel "qr" {payload, ...} -> laptop UI over the
   field router/LTE LAN (full payload, no 6-char MAVLink-regex limit).

NOTE: mission-ui's MAVLink QR regex only auto-latches [0-9A-Za-z]{1,6},
so keep competition payloads short if you want the MP-message path to
auto-capture in the UI. The websocket event always carries the full text.
"""
import logging
import threading
import time

log = logging.getLogger("qr_relay")


def relay_qr(payload, fc, hub, interval_s=2.0, window_s=15.0, severity=6):
    """Send `payload` on both routes. Returns the resend thread."""
    payload = str(payload or "").strip()
    if not payload:
        return None
    try:
        hub.push("qr", {"payload": payload, "source": "pi-ws", "ts": time.time()})
    except Exception as e:
        log.warning("qr ws event failed: %r", e)

    def _resend():
        text = ("QR:%s" % payload).encode("utf-8")[:50].decode("utf-8", "ignore")
        end = time.time() + window_s
        n = 0
        while time.time() < end:
            try:
                fc.send_statustext(text, severity=severity)
                n += 1
            except Exception as e:
                log.warning("statustext resend failed: %r", e)
                break
            time.sleep(interval_s)
        log.info("QR relay done: %r x%d over MAVLink + ws event", text, n)

    t = threading.Thread(target=_resend, name="qr-relay", daemon=True)
    t.start()
    return t
