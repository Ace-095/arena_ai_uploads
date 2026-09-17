"""Total-link-loss recovery: store & forward for QR results.

Gap it closes: search/detect/decode run fully onboard regardless of
comms, but result delivery dies silently if the MAVLink path (STATUSTEXT
-> MP) and the UI path (websocket -> laptop) are BOTH down at transmit
time. This module buffers {code, gps, timestamp} to disk and flushes
through the normal publish path when any link recovers.

Honesty notes (read before "improving" this):
- MAVLink STATUSTEXT has no acknowledgement. "MP path failed" is a proxy:
  the send raised, or the FC link itself is dead (fc.link_ok()). A flush
  can therefore duplicate a result MP already saw — harmless (an identical
  QR line printed twice).
- "UI path failed" is also a proxy: zero WS clients at push time, or the
  push raised. A connected-but-deaf browser still counts as delivered.
- Entries are removed ONLY after a publish attempt reports a route ok.
  Failed flushes stay buffered (attempts counter is informational).
- The flusher is a daemon thread: it never blocks the FSM — transmit
  keeps its window and post_action (RTL) fires on schedule even with all
  links down. "All comms down" becomes delayed-success, not silent loss.

Stdlib only; fc/hub are duck-typed so tests run anywhere.
"""
import json
import logging
import os
import threading
import time

log = logging.getLogger("store_fwd")


class ResultStore:
    """Append-style JSONL queue with atomic-drop rewrites."""

    def __init__(self, path, cap=500):
        self.path = path
        self.cap = max(10, int(cap))
        self._lock = threading.RLock()
        self._items = []
        self._seq = 0
        self._load()

    def _load(self):
        try:
            with open(self.path, encoding="utf-8") as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        d = json.loads(line)
                    except Exception:
                        continue
                    if isinstance(d, dict) and d.get("code"):
                        self._seq = max(self._seq, int(d.get("id", 0)))
                        d.setdefault("attempts", 0)
                        self._items.append(d)
            if self._items:
                log.warning("store: %d buffered result(s) from last run",
                            len(self._items))
        except FileNotFoundError:
            pass
        except Exception as e:
            log.warning("store load failed (%r) — starting empty", e)

    def _persist_locked(self):
        d = os.path.dirname(self.path)
        if d:
            os.makedirs(d, exist_ok=True)
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            for it in self._items:
                f.write(json.dumps(it) + "\n")
        os.replace(tmp, self.path)

    def buffer(self, code, gps=None):
        """Buffer one result. gps: (lat, lon, alt|None). Returns entry id."""
        with self._lock:
            self._seq += 1
            lat, lon, alt = (tuple(gps) + (None, None, None))[:3] if gps \
                else (None, None, None)
            it = {"id": self._seq, "code": str(code), "lat": lat, "lon": lon,
                  "alt": alt, "ts": time.time(), "attempts": 0}
            self._items.append(it)
            while len(self._items) > self.cap:
                lost = self._items.pop(0)
                log.error("store full — dropped oldest buffered result %r",
                          lost.get("code"))
            self._persist_locked()
            log.warning("store: buffered %r (id=%d, %d pending)",
                        code, it["id"], len(self._items))
            return it["id"]

    def pending(self):
        with self._lock:
            return [dict(it) for it in self._items]

    def drop(self, ident):
        """Remove an entry ONLY after a route confirmed it. Returns bool."""
        with self._lock:
            before = len(self._items)
            self._items = [it for it in self._items if it.get("id") != ident]
            if len(self._items) != before:
                self._persist_locked()
                return True
            return False

    def note_attempt(self, ident):
        with self._lock:
            for it in self._items:
                if it.get("id") == ident:
                    it["attempts"] = int(it.get("attempts", 0)) + 1
                    break


class StoreForward:
    """Background flusher: idles while links are down, replays the buffer
    through publish_once when any route recovers."""

    def __init__(self, fc, hub_ref, path="logs/pending_results.jsonl",
                 poll_s=5.0, enabled=True, publish_fn=None, cap=500):
        self.fc = fc
        self._hub_ref = hub_ref  # hub object or callable -> hub (late-bound ok)
        self.store = ResultStore(path, cap=cap)
        self.poll_s = max(1.0, float(poll_s))
        self.enabled = bool(enabled)
        self._publish = publish_fn  # (code, gps) -> (mp_ok, ui_ok)
        self._stop = threading.Event()
        self._thread = None

    def _hub(self):
        try:
            return self._hub_ref() if callable(self._hub_ref) else self._hub_ref
        except Exception:
            return None

    def link_up(self):
        """Either delivery route plausibly alive (proxies, see module doc)."""
        try:
            if self.fc is not None and bool(self.fc.link_ok()):
                return True
        except Exception:
            pass
        try:
            h = self._hub()
            if h is not None and int(h.n_clients) > 0:
                return True
        except Exception:
            pass
        return False

    def _publish_default(self, code, gps):
        from qr_relay import publish_once
        return publish_once(code, self.fc, self._hub(), gps=gps)

    def flush_once(self):
        """One pass over the buffer. Returns (flushed, kept)."""
        items = self.store.pending()
        if not items or not self.link_up():
            return (0, len(items))
        pub = self._publish or self._publish_default
        flushed = 0
        for it in items:
            code = it.get("code")
            gps = (it.get("lat"), it.get("lon"), it.get("alt"))
            try:
                mp_ok, ui_ok = pub(code, gps)
            except Exception as e:
                log.debug("flush %r failed: %r", code, e)
                mp_ok = ui_ok = False
            if mp_ok or ui_ok:
                self.store.drop(it.get("id"))
                flushed += 1
                log.warning("store: flushed %r via %s", code, "+".join(
                    [n for n, ok in (("mp", mp_ok), ("ui", ui_ok)) if ok]))
            else:
                self.store.note_attempt(it.get("id"))
        return (flushed, len(items) - flushed)

    def start(self):
        if not self.enabled or self._thread:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="store-fwd",
                                        daemon=True)
        self._thread.start()
        log.info("store&forward up (poll %.0fs, %d pending)",
                 self.poll_s, len(self.store.pending()))

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _loop(self):
        while not self._stop.wait(self.poll_s):
            try:
                self.flush_once()
            except Exception as e:
                log.debug("flush loop: %r", e)
