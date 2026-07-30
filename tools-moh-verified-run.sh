#!/usr/bin/env bash
# Contention-safe measurement wrapper.
#
# Five Claude Code sessions edit this worktree concurrently, so a build/run pair is
# a race: another writer can disable or replace a probe between the build and the
# run, and the log then reports something the binary never contained. This wrapper
# closes that hole by asserting the probe marker is present in the *linked binary*
# immediately before running, and by recording the binary's hash next to the log so
# any result can be tied back to the exact build that produced it.
#
# Usage: tools-moh-verified-run.sh <tag> <duration_s> <interval_s> <marker> [VAR=VAL ...]
set -u
TAG="$1"; DUR="$2"; IVL="$3"; MARKER="$4"; shift 4
BIN=build-runtime-launcher/ps2xRuntime/ps2MOHFrontlineRunner

echo "== rebuild =="
cmake --build build-runtime-launcher --target ps2MOHFrontlineRunner -j3 2>&1 \
  | grep -E 'error|Linking CXX executable' | head -5

if ! strings "$BIN" 2>/dev/null | grep -q -- "$MARKER"; then
  echo "ABORT: marker '$MARKER' absent from the linked binary."
  echo "       Another session very likely edited or disabled the probe."
  strings "$BIN" 2>/dev/null | grep -o 'MOH:[a-z0-9-]*' | sort -u | head -20
  exit 3
fi

HASH="$(sha256sum "$BIN" | cut -c1-16)"
echo "== binary verified: marker present, sha=$HASH =="

./tools-moh-run-capture.sh "$TAG" "$DUR" "$IVL" "$@"
LOG="$(ls -t moh-${TAG}-*.log | head -1)"
echo "binary-sha=$HASH marker=$MARKER" >> "$LOG"

# Re-check after the run: if the binary changed underneath, the log is suspect.
HASH2="$(sha256sum "$BIN" | cut -c1-16)"
if [ "$HASH" != "$HASH2" ]; then
  echo "WARNING: binary changed during the run ($HASH -> $HASH2); treat $LOG as unreliable."
else
  echo "== binary stable across the run; $LOG is attributable =="
fi
echo "LOG=$LOG"
