#!/usr/bin/env bash
# Harnais de run + capture pour l'A/B visuel MOH (intertitre "Jour J" / rendu 3D).
# Usage: tools-moh-run-capture.sh <tag> <duree_s> <intervalle_s> [VAR=VAL ...]
# Lance le runner, capture la fenêtre X11 à intervalle régulier, log horodaté.
set -u
TAG="$1"; DUR="$2"; IVL="$3"; shift 3
STAMP="$(date +%Y%m%d-%H%M%S)"
LOG="moh-${TAG}-${STAMP}.log"
SHOTDIR="/tmp/moh-shots-${TAG}-${STAMP}"
mkdir -p "$SHOTDIR"

echo "LOG=$LOG"
echo "SHOTDIR=$SHOTDIR"
echo "ENV: $*"

env "$@" timeout "${DUR}s" \
  build-runtime-launcher/ps2xRuntime/ps2MOHFrontlineRunner \
  extracted/SLUS_203.68 > "$LOG" 2>&1 &
RUNNER_PID=$!

# Attendre l'apparition de la fenêtre du runner (titre "PS2-Recomp | ...")
WID=""
for _ in $(seq 1 80); do
  WID="$(xdotool search --name '^PS2-Recomp' 2>/dev/null | tail -1)"
  [ -n "$WID" ] && break
  kill -0 "$RUNNER_PID" 2>/dev/null || break
  sleep 0.25
done
echo "WINDOW=$WID PID=$RUNNER_PID"

N=0
while kill -0 "$RUNNER_PID" 2>/dev/null; do
  sleep "$IVL"
  kill -0 "$RUNNER_PID" 2>/dev/null || break
  N=$((N+1))
  OUT="$SHOTDIR/$(printf '%03d' "$N")-t$(date +%s).png"
  if [ -n "$WID" ]; then
    import -window "$WID" "$OUT" 2>/dev/null || true
  fi
done
wait "$RUNNER_PID" 2>/dev/null
RC=$?
echo "rc=$RC (124=timeout normal)"
echo "captures: $(ls "$SHOTDIR" | wc -l)"
