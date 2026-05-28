#!/usr/bin/env bash
# Launch the VR teleop stack: OpenVR->ROS publisher, the teleop, and by
# DEFAULT also the visualizer + calibration sliders + AmazingHand.
# Sources ROS2 Jazzy and uses SYSTEM python (rclpy is not importable
# from conda).
#
#   ./run_vr_teleop.sh                  # everything (viz + cal + hand)
#   ./run_vr_teleop.sh --no-hand        # everything except the hand
#   ./run_vr_teleop.sh --no-arm         # no arm/teleop (publisher + viz + cal + hand)
#   ./run_vr_teleop.sh --no-viz --no-cal  # publisher + teleop + hand
#   ./run_vr_teleop.sh --no-viz --no-cal --no-hand   # bare teleop
#   ./run_vr_teleop.sh --text           # text visualizer instead of 3D
#
# Ctrl+C (or any exit) tears down ALL children. Teleop and hand are
# sent SIGINT so their finally-blocks run (teleop: stop joints,
# disconnect, free the CAN port; hand: open then torque off) — this is
# what prevents the stale-/dev/ttyACM* lockups.
set -euo pipefail

ROS_SETUP=/opt/ros/jazzy/setup.bash
PY=/usr/bin/python3
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# On by default; --no-* opts out.
WANT_VIZ=1
WANT_HAND=1
WANT_CAL=1
WANT_ARM=1
VIZ_ARGS=()
for a in "$@"; do
    case "$a" in
        --no-viz)  WANT_VIZ=0 ;;
        --no-hand) WANT_HAND=0 ;;
        --no-cal)  WANT_CAL=0 ;;
        --no-arm)  WANT_ARM=0 ;;
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

PUB_PID=""; VIZ_PID=""; TELEOP_PID=""; HAND_PID=""; CAL_PID=""

# SIGINT a pid and wait up to ~5s for it to exit (so its finally runs).
graceful() {
    local pid="$1"
    [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null || return 0
    kill -INT "$pid" 2>/dev/null || true
    for _ in $(seq 1 20); do
        kill -0 "$pid" 2>/dev/null || return 0
        sleep 0.25
    done
    kill -9 "$pid" 2>/dev/null || true
}

cleanup() {
    trap - EXIT INT TERM
    echo
    echo "[run] shutting down…"
    # Hardware-touching children first, in parallel, each given time
    # for its cleanup (teleop: arm stop + free CAN port; hand: open +
    # torque off). Then the data sources.
    # NOTE: wait ONLY on the two graceful subshells. A bare `wait`
    # would also block on PUB/VIZ/CAL (also `&` background jobs of this
    # shell) which are still alive here -> cleanup would deadlock and
    # nothing below would ever get killed.
    graceful "$TELEOP_PID" & gp_teleop=$!
    graceful "$HAND_PID" & gp_hand=$!
    wait "$gp_teleop" "$gp_hand" 2>/dev/null || true
    for pid in "$VIZ_PID" "$CAL_PID" "$PUB_PID"; do
        [ -n "$pid" ] && kill -INT "$pid" 2>/dev/null || true
    done
    sleep 0.5
    for pid in "$VIZ_PID" "$CAL_PID" "$PUB_PID"; do
        [ -n "$pid" ] && kill -9 "$pid" 2>/dev/null || true
    done
    echo "[run] done."
}
trap cleanup EXIT INT TERM

cd "$HERE"

# J1 motor board is dead on CAN — exclude it so connect()/encoder sync
# doesn't time out. AXES in vr_ros_io.py also has J1 commented out.
export ARCTOS_JOINTS="${ARCTOS_JOINTS:-2,3,4,5,6}"

echo "[run] starting VR->ROS publisher…"
"$PY" vr_controller_publisher.py & PUB_PID=$!
sleep 2
kill -0 "$PUB_PID" 2>/dev/null || { echo "[run] publisher failed" >&2; exit 1; }

if [ "$WANT_VIZ" = 1 ]; then
    echo "[run] starting visualizer…"
    "$PY" vr_visualizer.py "${VIZ_ARGS[@]}" & VIZ_PID=$!
fi

if [ "$WANT_CAL" = 1 ]; then
    echo "[run] starting calibration sliders…"
    "$PY" vr_calibration_gui.py & CAL_PID=$!
fi

if [ "$WANT_HAND" = 1 ]; then
    echo "[run] starting AmazingHand (--live)…"
    "$PY" vr_hand_control.py --live & HAND_PID=$!
fi

if [ "$WANT_ARM" = 1 ]; then
    echo "[run] starting teleop (foreground)…"
    "$PY" vr_teleop_rotation.py & TELEOP_PID=$!
    wait "$TELEOP_PID"
else
    echo "[run] arm/teleop DISABLED (--no-arm) — running publisher"\
         "+ viz/cal/hand only. Ctrl+C to quit."
    # No teleop to wait on; block on the publisher so the stack stays up
    # until the publisher dies or Ctrl+C triggers cleanup().
    wait "$PUB_PID"
fi
