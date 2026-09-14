"""QR result delivery on two routes (fire both, either can win):

1. MAVLink STATUSTEXT  `QR:<payload>` -> flight controller -> forwarded on
   the telemetry stream -> Mission Planner Messages tab (and mission-ui's
   MAVLink QR catcher). Resent every `interval_s` for `window_s` because a
   single 50-byte text can drown in telemetry traffic.
2. Pi websocket event  channel "qr" {payload, gps, ...} -> laptop UI over the
   field router/LTE LAN (full payload, no 6-char MAVLink-regex limit).

NOTE: mission-ui's MAVLink QR regex only auto-latches [0-9A-Za-z]{1,6},
so keep competition payloads short if you want the MP-message path to
auto-capture in the UI. The websocket event always carries the full text.

Total-loss recovery (store_forward.py): publish_once() reports per-route
proxies (mp_ok, ui_ok); when the first shot fails BOTH, relay_qr buffers
{code, gps, ts} to disk and a background flusher replays it on recovery.
Proxies, not acks: STATUSTEXT has no delivery confirmation (send-ok +
live FC link counts as mp_ok), and a connected WS client counts as ui_ok.
Duplicates after a flush are harmless identical lines.
"""
import logging
import threading
import time

log = logging.getLogger("qr_relay")


def _qr_text(payload):
    return ("QR:%s" % payload).encode("utf-8")[:50].decode("utf-8", "ignore")


def publish_once(payload, fc=None, hub=None, gps=None, severity=6):
    """One shot on both routes. Returns (mp_ok, ui_ok) — PROXIES, see above.

    gps: (lat, lon, alt|None) attached to the ws event for UI map pins.
    """
    payload = str(payload or "").strip()
    if not payload:
        return (False, False)
    try:
        n = hub.push("qr", {"payload": payload, "source": "pi-ws",
                            "ts": time.time(),
                            "gps": list(gps) if gps else None}) if hub else 0
        ui_ok = bool(n and n > 0)
    except Exception as e:
        log.debug("qr ws event failed: %r", e)
        ui_ok = False
    try:
        if fc is None:
            raise RuntimeError("no fc")
        fc.send_statustext(_qr_text(payload), severity=severity)
        mp_ok = bool(fc.link_ok())
    except Exception as e:
        log.debug("statustext send failed: %r", e)
        mp_ok = False
    return (mp_ok, ui_ok)


def relay_qr(payload, fc, hub, interval_s=2.0, window_s=15.0, severity=6,
             store=None, gps=None):
    """Send `payload` on both routes (first shot now + resend thread over
    the window). store: ResultStore — when the first shot fails BOTH
    routes, the result is buffered for the flusher. Returns the thread."""
    payload = str(payload or "").strip()
    if not payload:
        return None
    mp_ok, ui_ok = publish_once(payload, fc, hub, gps=gps, severity=severity)
    if store is not None and not (mp_ok or ui_ok):
        try:
            store.buffer(payload, gps)
        except Exception as e:
            log.warning("store buffer failed: %r", e)

    def _resend():
        text = _qr_text(payload)
        end = time.time() + window_s
        n = 0
        time.sleep(interval_s)  # first shot already went above; resends follow
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
