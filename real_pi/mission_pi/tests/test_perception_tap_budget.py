"""The tap must not cost the MISSION anything measurable, and must stay correct
when its two threads (detector + watchdog) overlap.

The install contract in perception_tap.py is "purely additive and observational".
Two ways to break that promise, neither of which any existing test caught:

1. **Cost.** `_wrap_detect_tiles` calls `buffer.note_boxes(...)`, and note_boxes
   FLUSHES the previous frame inside `ObservationBuffer._lock`, which runs
   `PerceptionPipeline.update(...)` SYNCHRONOUSLY on the detector's own thread.
   So every observed frame makes the mission's detector pay for the perception
   pipeline. Measured here: ~1.1 ms at 760x1014 and ~2.4 ms at 1520x2028, worst
   case ~3 ms. That is fine - but it is only fine while `update()` stays cheap,
   and nothing said so. A future change that decodes inside update() would add
   tens of milliseconds to every flight detection while every existing test
   still passed. The budget test below is the tripwire.

2. **Overlap.** The watchdog thread calls `flush()` while the detector thread is
   in `note_boxes()`, and the detector thread can be in `note_payload()` at the
   same time. Nothing tested that, and the counters an operator reads
   (`flushes`, `errors`) live on the same object.

Also pinned: the wrapper returns the ORIGINAL result object, not a copy, so an
observation can never alter what the mission sees.
"""
import os
import sys
import threading
import time
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_PI = os.path.dirname(_HERE)
if _PI not in sys.path:
    sys.path.insert(0, _PI)

import numpy as np                                       # noqa: E402

import perception as P                                   # noqa: E402
import perception_tap as PT                              # noqa: E402

#: Generous by design: the sandbox is shared and these are wall-clock numbers.
#: Measured medians are 0.4-2.4 ms and the worst single call 3.1 ms, so a real
#: regression of the kind that matters (an inline decode) is 10-100x these.
UPDATE_BUDGET_MS = 25.0
UPDATE_WORST_MS = 200.0
WRAPPER_OVERHEAD_MS = 10.0


def _frame(h=760, w=1014):
    rng = np.random.default_rng(7)
    return (rng.random((h, w, 3)) * 255).astype(np.uint8)


def _boxes(n=3):
    return [(100 + i * 40, 120, 60, 60, 0.8) for i in range(n)]


def _pipeline():
    return P.PerceptionPipeline()


class TestUpdateCostBudget(unittest.TestCase):
    """What one observed frame costs the thread the mission runs detection on."""

    def _time_updates(self, shape, n=8):
        p = _pipeline()
        f = _frame(*shape)
        b = _boxes()
        ts = []
        for _ in range(n):
            t0 = time.perf_counter()
            p.update(frame=f, boxes=b, payload=None, ts=time.time(), source="tap")
            ts.append((time.perf_counter() - t0) * 1000.0)
        ts.sort()
        return ts[len(ts) // 2], ts[-1]

    def test_pipeline_update_stays_within_the_budget(self):
        for shape in ((760, 1014), (1520, 2028), (480, 640)):
            med, worst = self._time_updates(shape)
            with self.subTest(frame=shape):
                self.assertLess(med, UPDATE_BUDGET_MS,
                                "PerceptionPipeline.update() median %.2f ms at %s "
                                "- it runs inline on the DETECTOR thread (see "
                                "ObservationBuffer._flush_locked), so this is "
                                "flight-path cost" % (med, shape))
                self.assertLess(worst, UPDATE_WORST_MS,
                                "worst update %.2f ms at %s" % (worst, shape))

    def test_the_wrapper_adds_little_besides_the_pipeline(self):
        """Overhead of the observation machinery itself: no pipeline work in
        the way (the buffer is empty, so nothing flushes)."""
        buf = PT.ObservationBuffer(_pipeline())
        f = _frame(480, 640)
        b = _boxes(2)
        buf.note_boxes(f, b)          # one real flush
        ts = []
        for _ in range(200):
            t0 = time.perf_counter()
            buf.note_boxes(f, b)      # flush of an empty buffer
            ts.append((time.perf_counter() - t0) * 1000.0)
        ts.sort()
        self.assertLess(ts[len(ts) // 2], WRAPPER_OVERHEAD_MS,
                        "the buffer's own per-frame cost is %.2f ms"
                        % ts[len(ts) // 2])


class FakeDetector(object):
    def __init__(self, result=None):
        self.name = "fake"
        self.result = result if result is not None else [("box", 1)]

    def detect_tiles(self, frame, rows=3, cols=3, overlap=0.0):
        return self.result

    def detect(self, frame):
        return self.result


class TestNoInterferenceOnTheHotPath(unittest.TestCase):
    def setUp(self):
        self.buf = PT.ObservationBuffer(_pipeline())

    def test_the_original_result_object_is_returned_unchanged(self):
        """Identity, not equality: the mission must receive the very object its
        detector produced, so the tap cannot rewrite a box on the way out."""
        det = FakeDetector()
        wrapped = PT._wrap_detect_tiles(lambda d, f, *a, **k: d.detect_tiles(f),
                                        self.buf)
        out = wrapped(det, _frame(240, 320))
        self.assertIs(out, det.result,
                      "the tap returned a different object than the detector")

    def test_the_caller_boxes_list_is_not_mutated(self):
        boxes = _boxes(3)
        snapshot = [tuple(b) for b in boxes]
        self.buf.note_boxes(_frame(240, 320), boxes)
        self.assertEqual([tuple(b) for b in boxes], snapshot,
                         "note_boxes mutated the caller's list")

    def test_a_disabled_buffer_still_passes_the_result_through(self):
        det = FakeDetector(result=[])
        wrapped = PT._wrap_detect_tiles(lambda d, f, *a, **k: d.detect_tiles(f),
                                        self.buf)
        self.assertEqual(wrapped(det, _frame(240, 320)), [])

    def test_an_error_still_reaches_the_caller_and_the_pipeline(self):
        """A detector failure must never be swallowed by the tap."""
        def boom(*a, **k):
            raise RuntimeError("HAILO_NOT_AVAILABLE")
        wrapped = PT._wrap_detect_tiles(boom, self.buf)
        with self.assertRaises(RuntimeError):
            wrapped(FakeDetector(), _frame(240, 320))
        self.assertEqual(self.buf.errors, 0)          # note_error, not a flush error


class TestThreadSafety(unittest.TestCase):
    """Detector thread + watchdog thread, on one buffer."""

    def test_concurrent_note_and_flush_lose_nothing(self):
        """Each noted frame must reach the pipeline EXACTLY ONCE, while another
        thread force-flushes continuously.

        Counted by identity of the box sets, not by a tally: a tally cannot tell
        "one frame flushed twice" from "one frame lost and another invented", and
        one flush is triggered by the NEXT note while a watchdog flush can also
        take it - so exactly-once is the property worth asserting.
        """
        p = _pipeline()
        buf = PT.ObservationBuffer(p, flush_timeout_s=0.05)
        f = _frame(480, 640)
        per_thread, n_threads = 60, 4
        stop = threading.Event()
        problems, seen, seen_lock = [], [], threading.Lock()

        real_update = p.update

        def counting_update(**kw):
            boxes = tuple(kw.get("boxes") or ())
            if boxes:
                with seen_lock:
                    seen.append(boxes[0][0])
            return real_update(**kw)
        p.update = counting_update

        def noter(tid):
            try:
                for i in range(per_thread):
                    buf.note_boxes(f, [("t%d-%d" % (tid, i),)])
                    if i % 7 == 0:
                        buf.note_payload("payload-%d-%d" % (tid, i))
                    if i % 11 == 0:
                        buf.pending()
                    if i % 13 == 0:
                        buf.age_s()
            except Exception as e:                    # noqa: BLE001
                problems.append("%r in noter %d" % (e, tid))

        def flusher():
            try:
                while not stop.is_set():
                    buf.flush(force=True)
                    time.sleep(0.001)
            except Exception as e:                    # noqa: BLE001
                problems.append("%r in flusher" % (e,))

        threads = [threading.Thread(target=noter, args=(i,)) for i in range(n_threads)]
        watch = threading.Thread(target=flusher)
        watch.start()
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=30)
        stop.set()
        watch.join(timeout=10)
        buf.flush(force=True)

        self.assertEqual(problems, [], "threads raised: %s" % problems)
        self.assertTrue(all(not t.is_alive() for t in threads))
        self.assertEqual(buf.errors, 0, "a flush failed while threads overlapped")

        noted = set("t%d-%d" % (tid, i) for tid in range(n_threads)
                    for i in range(per_thread))
        self.assertEqual(len(seen), len(set(seen)),
                         "a frame was flushed more than once: %s"
                         % [t for t in seen if seen.count(t) > 1][:5])
        self.assertEqual(set(seen), noted,
                         "missing %d, unexpected %d"
                         % (len(noted - set(seen)), len(set(seen) - noted)))
        self.assertGreaterEqual(buf.flushes, 1)

    def test_the_lock_serialises_flush_and_note(self):
        """A flush must not see a half-written frame: the boxes and the frame
        the pipeline receives must belong to the SAME note_boxes call."""
        p = _pipeline()
        buf = PT.ObservationBuffer(p, flush_timeout_s=0.001)
        pairs = []
        real_update = p.update

        def spy(**kw):
            if kw.get("boxes"):
                pairs.append((id(kw.get("frame")), tuple(kw["boxes"])))
            return real_update(**kw)
        p.update = spy

        frames = [_frame(120, 160) for _ in range(20)]
        for i, f in enumerate(frames):
            buf.note_boxes(f, [("iter-%d" % i,)])
            time.sleep(0.002)
        buf.flush(force=True)

        self.assertTrue(pairs, "no flush observed the boxes")
        for frame_id, boxes in pairs:
            self.assertEqual(len(boxes), 1)
            tag = boxes[0][0]
            idx = int(tag.split("-")[1])
            self.assertEqual(frame_id, id(frames[idx]),
                             "a flush paired boxes with a DIFFERENT frame's image")


if __name__ == "__main__":
    unittest.main(verbosity=2)
