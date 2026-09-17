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


def _qr_text(payload, tag="QR"):
    return ("%s:%s" % (tag, payload)).encode("utf-8")[:50].decode(
        "utf-8", "ignore")


def announce(payload, fc=None, hub=None, gps=None, window_s=6.0,
             interval_s=1.5, severity=3, tag="QR_SEEN"):
    """FIRST-DECODE instant report — the 'no delay' half of the two-stage
    delivery (real_pi).

    The moment a QR decodes in a single frame (BEFORE the N-frame
    confirm-streak), this puts `QR_SEEN:<payload>` on the vehicle's
    STATUSTEXT stream — which the Pixhawk forwards to every GCS, so it
    appears in Mission Planner's Messages tab within one telemetry tick
    (~50 ms over USB) — and a `qr_seen` ws event on the Pi link.

    A short resend window (default 6 s) covers a single dropped packet:
    STATUSTEXT has no ack, so one shot is not "perfect". Duplicates after
    the confirmed `QR:<payload>` lands are harmless identical lines.

    Dedup is the CALLER's job (mission.py keeps the announced set) — call
    this at most once per distinct payload.
    """
    payload = str(payload or "").strip()
    if not payload:
        return None
    text = _qr_text(payload, tag=tag)
    try:
        if fc is not None:
            fc.send_statustext(text, severity=severity)
    except Exception as e:
        log.debug("qr_seen statustext failed: %r", e)
    try:
        if hub is not None:
            hub.push("qr_seen", {"payload": payload, "tag": tag,
                                 "ts": time.time(),
                                 "gps": list(gps) if gps else None})
            # also on the event channel: the mission-ui (01a0a7de build)
            # renders `event` envelopes, so the QR_SEEN line shows up in
            # the UI feed without any UI change.
            hub.push("event", {"type": "qr_seen", "payload": payload,
                               "tag": tag, "ts": time.time(),
                               "gps": list(gps) if gps else None})
        hub.push("log", {"level": "WARN",
                         "msg": "QR SEEN (first decode, pre-confirm): %r"
                                % payload})
    except Exception:
        pass

    def _resend():
        end = time.time() + max(0.0, float(window_s))
        while time.time() < end:
            time.sleep(max(0.2, float(interval_s)))
            try:
                fc.send_statustext(text, severity=severity)
            except Exception:
                break
        log.info("QR_SEEN relay done: %r (%.0fs window)", text, window_s)

    t = threading.Thread(target=_resend, name="qr-announce", daemon=True)
    t.start()
    return t


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
