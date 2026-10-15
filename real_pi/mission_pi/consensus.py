"""N-frame consensus for QR payloads (stdlib only).

A single-frame decode is not trusted: the same payload must decode on N
consecutive frames (with a small miss tolerance so one blurred frame does
not wipe out a good streak) before it counts as CONFIRMED.
"""
import threading
import time


class ConsensusBuffer:
    def __init__(self, required_consecutive=3, miss_tolerance=1):
        self.required = max(1, int(required_consecutive))
        self.miss_tol = max(0, int(miss_tolerance))
        self._lock = threading.Lock()
        self.reset()

    def reset(self):
        with self._lock:
            self._value = None
            self._streak = 0
            self._miss = 0
            self._confirmed = False
            self._aux = None

    def update(self, value, aux=None):
        """Feed one frame's decode (value or None). Returns
        (newly_confirmed: bool, state: dict)."""
        with self._lock:
            good = bool(value)
            if good and value == self._value:
                self._streak += 1
                self._miss = 0
                if aux is not None:
                    self._aux = aux
            elif good and self._value is None:
                self._value = value
                self._streak = 1
                self._miss = 0
                self._confirmed = False
                if aux is not None:
                    self._aux = aux
            else:
                # blank frame or a disagreeing decode: tolerate a few,
                # then switch (new value) or give up (blank).
                self._miss += 1
                if self._miss > self.miss_tol:
                    if good:
                        self._value = value
                        self._streak = 1
                        if aux is not None:
                            self._aux = aux
                    else:
                        self._value = None
                        self._streak = 0
                    self._miss = 0
                    self._confirmed = False
            newly = False
            if self._streak >= self.required and not self._confirmed:
                self._confirmed = True
                newly = True
            state = {"value": self._value if self._confirmed else None,
                     "tracking": self._value,
                     "streak": self._streak,
                     "required": self.required,
                     "confirmed": self._confirmed,
                     "aux": self._aux,
                     "ts": time.time()}
            return newly, state

    def status(self):
        with self._lock:
            return {"value": self._value if self._confirmed else None,
                    "tracking": self._value,
                    "streak": self._streak,
                    "required": self.required,
                    "confirmed": self._confirmed,
                    "aux": self._aux,
                    "ts": time.time()}
