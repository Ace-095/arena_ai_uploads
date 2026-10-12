#!/usr/bin/env bash
# run_tests.sh — one command for every mission_pi test, both styles.
#
# The folder contains two kinds of test and they need different drivers:
#   * unittest modules  (class Test*(unittest.TestCase) + unittest.main())
#   * self-checking scripts (they assert at import time and sys.exit(0) when
#     they pass) — pytest chokes on those, so `pytest tests/` alone is NOT a
#     valid "everything passed" signal for this repo.
# Running each file directly handles both: a unittest module runs its own
# unittest.main(), a script runs itself.
#
#   ./tools/run_tests.sh                 # everything
#   ./tools/run_tests.sh fence           # only the geofence / UI-fence matrix
#   ./tools/run_tests.sh --list fence    # just show what would run
#
# Exit status is 0 only if every selected file passed.
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PI="$(dirname "$HERE")"
cd "$PI" || exit 2

PY="${PYTHON:-}"
if [ -z "$PY" ]; then
  for cand in "$PI/../../.venv/bin/python" "$PI/../.venv/bin/python" python3; do
    if "${cand}" -c "import pymavlink" >/dev/null 2>&1; then PY="$cand"; break; fi
  done
  [ -z "$PY" ] && PY=python3
fi

FILTER=""
LIST_ONLY=0
for a in "$@"; do
  case "$a" in
    --list) LIST_ONLY=1 ;;
    fence) FILTER="fence" ;;
    *) echo "unknown argument: $a"; exit 2 ;;
  esac
done

# The fence matrix: wire protocol, HTTP API + DOM clicks, and the end-to-end
# run against the fake Pixhawk, plus the two suites that exercise the same
# endpoints (commands/presets) so a fence change cannot silently break them.
if [ "$FILTER" = "fence" ]; then
  FILES="tests/test_fence_protocol.py
tests/test_fence_e2e_fake_fc.py
tests/test_ui_command_api.py
tests/test_preset_stream.py"
else
  FILES="$(ls tests/test_*.py | sort)"
fi

echo "python: $PY"
"$PY" -c "import sys; print('version:', sys.version.split()[0])" 2>/dev/null
# Optional dependencies change WHICH files can do their work, so a SKIP must be
# attributable. (Measured: after a sandbox reset these vanish while the git
# worktree survives, which silently turned whole suites into skips.)
"$PY" - <<'DEPS' 2>/dev/null || true
try:
    import importlib
    have = []
    for name, why in (("numpy", "vision/tracker/bench suites"),
                      ("cv2", "vision + camera suites"),
                      ("yaml", "config loading"),
                      ("pymavlink", "SITL / FC link"),
                      ("onnx", "model-fact checks in test_hef_and_focus"),
                      ("onnxruntime", "the CPU detector ladder")):
        try:
            importlib.import_module(name)
            have.append(name)
        except Exception:
            print("optional dep MISSING: %-12s -> skips that need %s" % (name, why))
    print("optional deps present:", ", ".join(have) if have else "none")
except Exception:
    pass
DEPS

if [ "$LIST_ONLY" = "1" ]; then
  echo "$FILES"
  exit 0
fi

PASS=0; FAIL=0; SKIP=0
FAILED=""
START=$(date +%s)
for f in $FILES; do
  [ -f "$f" ] || continue
  name="$(basename "$f")"
  out="$(timeout 600 "$PY" "$f" 2>&1)"; rc=$?
  if [ $rc -eq 0 ]; then
    # SKIPPED means the file did no work that it needed to do - NOT merely that
    # it mentioned skipping. The old rule grepped a bare word "skipped"
    # anywhere, which mis-counted test_fence_e2e_fake_fc.py (24 tests, 23 of
    # them passing, one optional DOM test skipped) as a whole-file SKIP, and so
    # under-reported the coverage. Rules now, in order:
    #   1. all of a unittest run skipped  -> Ran N tests / OK (skipped=N)
    #   2. a self-checking script borrowing the word at line start, with no
    #      unittest run at all  -> "SKIP ..." / "SELF-SKIP" (the project's
    #      test_hef_and_focus.py prints "SKIP model facts (...)" with no colon,
    #      so the colon cannot be required - and this branch is only reachable
    #      when no "Ran N tests" line exists, which is what keeps it safe)
    #   3. a unittest run that ran nothing  -> Ran 0 tests
    ran="$(printf '%s' "$out" | sed -nE 's/^Ran ([0-9]+) tests?.*/\1/p' | tail -1)"
    allskipped="$(printf '%s' "$out" | sed -nE 's/^OK \(skipped=([0-9]+)\).*/\1/p' | tail -1)"
    if printf '%s' "$out" | grep -qE "^Ran 0 tests"; then
      SKIP=$((SKIP+1)); mark="SKIP"
    elif [ -n "$ran" ] && [ -n "$allskipped" ] && [ "$ran" = "$allskipped" ]; then
      SKIP=$((SKIP+1)); mark="SKIP"
    elif [ -n "$ran" ]; then
      PASS=$((PASS+1)); mark="PASS"
    elif printf '%s' "$out" | grep -qE "^(SKIP\b|SELF-SKIP)"; then
      SKIP=$((SKIP+1)); mark="SKIP"
    else
      PASS=$((PASS+1)); mark="PASS"
    fi
  else
    FAIL=$((FAIL+1)); mark="FAIL"; FAILED="$FAILED $name"
    printf '%s\n' "$out" | tail -25
  fi
  printf '%-6s %s\n' "$mark" "$name"
done

echo "------------------------------------------------------------"
echo "mission_pi tests: $PASS passed, $FAIL failed, $SKIP skipped  ($(( $(date +%s) - START ))s)"
if [ $FAIL -ne 0 ]; then
  echo "failed:$FAILED"
  echo
  echo "For the geofence path specifically:"
  echo "  $0 fence"
  exit 1
fi
exit 0
