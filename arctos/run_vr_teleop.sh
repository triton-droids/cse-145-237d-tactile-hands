#!/usr/bin/env bash
# Launch the VR teleop stack: OpenVR->ROS publisher, then the teleop,
# optionally the visualizer. Sources ROS2 Jazzy and uses SYSTEM python
# (rclpy is not importable from conda).
#
#   ./run_vr_teleop.sh            # publisher + teleop
#   ./run_vr_teleop.sh --viz      # + 3D visualizer
#   ./run_vr_teleop.sh --viz --text   # + text visualizer
#
# Ctrl+C (or any exit) tears down ALL children. The teleop is sent
# SIGINT so its finally-block runs (stop joints, disconnect, free the
# CAN port) — this is what prevents the stale-/dev/ttyACM0 lockups.
set -euo pipefail

ROS_SETUP=/opt/ros/jazzy/setup.bash
PY=/usr/bin/python3
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

WANT_VIZ=0
VIZ_ARGS=()
for a in "$@"; do
    case "$a" in
        --viz)  WANT_VIZ=1 ;;
        --text) VIZ_ARGS+=(--text) ;;
        *) echo "unknown arg: $a" >&2; exit 2 ;;
    esac
done

[ -f "$ROS_SETUP" ] || { echo "missing $ROS_SETUP" >&2; exit 1; }
# ROS setup.bash references unbound vars (AMENT_TRACE_SETUP_FILES, …);
# disable nounset just for the source, then restore it.
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u

PUB_PID=""; VIZ_PID=""; TELEOP_PID=""
cleanup() {
    trap - EXIT INT TERM
    echo
    echo "[run] shutting down…"
    # Teleop first, with SIGINT, and give its finally time to stop the
    # arm + free the port before anything else goes away.
    if [ -n "$TELEOP_PID" ] && kill -0 "$TELEOP_PID" 2>/dev/null; then
        kill -INT "$TELEOP_PID" 2>/dev/null || true
        for _ in $(seq 1 20); do
            kill -0 "$TELEOP_PID" 2>/dev/null || break
            sleep 0.25
        done
        kill -9 "$TELEOP_PID" 2>/dev/null || true
    fi
    for pid in "$VIZ_PID" "$PUB_PID"; do
        [ -n "$pid" ] && kill -INT "$pid" 2>/dev/null || true
    done
    sleep 0.5
    for pid in "$VIZ_PID" "$PUB_PID"; do
        [ -n "$pid" ] && kill -9 "$pid" 2>/dev/null || true
    done
    echo "[run] done."
}
trap cleanup EXIT INT TERM

cd "$HERE"

echo "[run] starting VR->ROS publisher…"
"$PY" vr_controller_publisher.py & PUB_PID=$!
sleep 2
kill -0 "$PUB_PID" 2>/dev/null || { echo "[run] publisher failed" >&2; exit 1; }

if [ "$WANT_VIZ" = 1 ]; then
    echo "[run] starting visualizer…"
    "$PY" vr_visualizer.py "${VIZ_ARGS[@]}" & VIZ_PID=$!
fi

echo "[run] starting teleop (foreground)…"
"$PY" vr_teleop_rotation.py & TELEOP_PID=$!
wait "$TELEOP_PID"
