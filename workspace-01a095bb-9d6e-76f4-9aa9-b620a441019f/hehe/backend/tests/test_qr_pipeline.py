"""
Unit and streak edge-case test suite for QR perception pipeline (Phase 7).

Run from backend/ directory:
    ./.venv/bin/python3 tests/test_qr_pipeline.py
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from camera_source import SyntheticQRSource
from models import BBox
from qr_pipeline import ConsensusBuffer, PlaceholderColorShapeDetector, decode_qr


def test_consensus_buffer_streak():
    buf = ConsensusBuffer(required_consecutive=3)

    # Frame 1
    newly_confirmed, res = buf.update("42", BBox(x=10, y=10, w=50, h=50))
    assert not newly_confirmed and not res.confirmed and res.streak == 1

    # Frame 2
    newly_confirmed, res = buf.update("42", BBox(x=10, y=10, w=50, h=50))
    assert not newly_confirmed and not res.confirmed and res.streak == 2

    # Frame 3 -> Should confirm
    newly_confirmed, res = buf.update("42", BBox(x=10, y=10, w=50, h=50))
    assert newly_confirmed and res.confirmed and res.streak == 3 and res.payload == "42"

    print("PASS: ConsensusBuffer streak reaching N=3 confirmed payload.")


def test_consensus_buffer_corrupted_frame_edge_case():
    """
    A single corrupted/no-decode frame must NOT wipe an in-progress
    streak — this was an explicit design requirement from the original
    plan and was previously violated by this exact test: it used to
    assert streak==0 after one bad frame, which was testing the bug,
    not the spec. Confirmed by direct reproduction last session: feeding
    good, good, blank, good, good never reached consensus under the old
    reset-on-any-miss behavior.
    """
    buf = ConsensusBuffer(required_consecutive=3, miss_tolerance=1)

    # Frame 1 & 2 valid
    buf.update("42", BBox(x=10, y=10, w=50, h=50))
    _, res2 = buf.update("42", BBox(x=10, y=10, w=50, h=50))
    assert res2.streak == 2 and not res2.confirmed

    # Frame 3 corrupted (None decode) -> must be ABSORBED, not wipe the streak
    newly_confirmed, res3 = buf.update(None, None)
    assert not newly_confirmed and not res3.confirmed and res3.streak == 2, (
        f"expected streak preserved at 2 after one tolerated miss, got {res3.streak}"
    )

    # Frame 4: back to a valid decode -> completes the streak immediately,
    # since progress wasn't discarded
    newly_confirmed, res4 = buf.update("42", BBox(x=10, y=10, w=50, h=50))
    assert newly_confirmed and res4.confirmed and res4.streak == 3 and res4.payload == "42"

    print("PASS: Single corrupted frame correctly absorbed without wiping the streak.")


def test_consensus_buffer_sustained_corruption_does_eventually_reset():
    """The miss-tolerance is not unlimited: enough CONSECUTIVE bad frames
    in a row must still eventually give up on a stale streak."""
    buf = ConsensusBuffer(required_consecutive=3, miss_tolerance=1)
    buf.update("42", BBox(x=10, y=10, w=50, h=50))
    buf.update("42", BBox(x=10, y=10, w=50, h=50))
    buf.update(None, None)  # miss 1 - tolerated, streak preserved at 2
    _, res = buf.update(None, None)  # miss 2 - exceeds tolerance, now resets
    assert res.streak == 0 and not res.confirmed, (
        f"expected streak to reset after exceeding miss_tolerance, got {res.streak}"
    )
    print("PASS: Sustained (not just single-frame) corruption still resets the streak.")


def test_consensus_buffer_mismatch_reset():
    """
    A single frame decoding to a DIFFERENT value than the one being
    tracked (a possible misread) must not immediately discard the
    existing streak — same miss-tolerance logic applies to mismatches
    as to None/empty decodes.
    """
    buf = ConsensusBuffer(required_consecutive=3, miss_tolerance=1)

    buf.update("TARGET_A", BBox(x=10, y=10, w=50, h=50))
    _, res2 = buf.update("TARGET_A", BBox(x=10, y=10, w=50, h=50))
    assert res2.streak == 2 and not res2.confirmed

    # A single mismatched read - tolerated, doesn't discard TARGET_A's progress
    _, res_b = buf.update("TARGET_B", BBox(x=10, y=10, w=50, h=50))
    assert res_b.streak == 2 and not res_b.confirmed, (
        f"expected TARGET_A's streak preserved at 2 after one tolerated mismatch, got {res_b.streak}"
    )

    # Back to TARGET_A - completes the streak
    newly_confirmed, res3 = buf.update("TARGET_A", BBox(x=10, y=10, w=50, h=50))
    assert newly_confirmed and res3.confirmed and res3.payload == "TARGET_A"
    print("PASS: Single mismatched frame tolerated, correct value still confirmed.")


def test_consensus_buffer_sustained_mismatch_does_switch():
    """If a DIFFERENT value keeps appearing beyond the tolerance, that's
    a genuine target change, not noise — the buffer should switch to it."""
    buf = ConsensusBuffer(required_consecutive=3, miss_tolerance=1)
    buf.update("TARGET_A", BBox(x=10, y=10, w=50, h=50))
    buf.update("TARGET_A", BBox(x=10, y=10, w=50, h=50))
    buf.update("TARGET_B", BBox(x=10, y=10, w=50, h=50))  # mismatch 1 - tolerated
    _, res = buf.update("TARGET_B", BBox(x=10, y=10, w=50, h=50))  # mismatch 2 - switches
    assert res.streak == 1, f"expected switch to TARGET_B with streak=1, got {res.streak}"
    buf.update("TARGET_B", BBox(x=10, y=10, w=50, h=50))
    newly_confirmed, res_final = buf.update("TARGET_B", BBox(x=10, y=10, w=50, h=50))
    assert newly_confirmed and res_final.payload == "TARGET_B"
    print("PASS: Sustained mismatch correctly treated as a genuine target change.")


def test_synthetic_qr_source_end_to_end():
    source = SyntheticQRSource(payload="SYNTHETIC_TARGET_99", fps=30.0)
    source.start()

    detector = PlaceholderColorShapeDetector()
    buf = ConsensusBuffer(required_consecutive=3)

    confirmed_payload = None

    try:
        for _ in range(15):
            frame = source.get_frame()
            assert frame is not None and frame.shape == (480, 640, 3)

            bbox = detector.detect_cache(frame)
            assert bbox is not None, "Detector failed to return candidate BBox"

            payload = decode_qr(frame, bbox)
            if payload:
                newly_confirmed, res = buf.update(payload, bbox)
                if newly_confirmed:
                    confirmed_payload = res.payload
                    break

        assert confirmed_payload == "SYNTHETIC_TARGET_99", f"Expected 'SYNTHETIC_TARGET_99', got '{confirmed_payload}'"
        print(f"PASS: End-to-end SyntheticQRSource decoding reached consensus for '{confirmed_payload}'.")

    finally:
        source.stop()


def main():
    failures = []
    tests = [
        ("test_consensus_buffer_streak", test_consensus_buffer_streak),
        ("test_consensus_buffer_corrupted_frame_edge_case", test_consensus_buffer_corrupted_frame_edge_case),
        ("test_consensus_buffer_sustained_corruption_does_eventually_reset", test_consensus_buffer_sustained_corruption_does_eventually_reset),
        ("test_consensus_buffer_mismatch_reset", test_consensus_buffer_mismatch_reset),
        ("test_consensus_buffer_sustained_mismatch_does_switch", test_consensus_buffer_sustained_mismatch_does_switch),
        ("test_synthetic_qr_source_end_to_end", test_synthetic_qr_source_end_to_end),
    ]

    for name, fn in tests:
        try:
            fn()
        except Exception as e:
            print(f"FAIL  {name}: {e}")
            import traceback
            traceback.print_exc()
            failures.append(name)

    if failures:
        print(f"\n{len(failures)} test(s) FAILED: {failures}")
        sys.exit(1)

    print("\nAll QR Perception Pipeline unit & streak edge-case tests passed!")


if __name__ == "__main__":
    main()
