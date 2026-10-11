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
    # A file counts as SKIPPED only when it says it ran nothing it needed to
    # (self-skipping tests) — matching the bare word "skip" anywhere would also
    # catch assertions like "PASS ready when all ok/skipped".
    if printf '%s' "$out" | grep -qE "^(SKIP|skipped)|Ran 0 tests|SELF-SKIP"; then
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
