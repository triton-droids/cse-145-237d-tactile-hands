#!/usr/bin/env bash
# Kill any VR-teleop processes left running after a failed Ctrl+C, and
# free the serial ports. SIGINT first (so teleop stops the arm + frees
# the CAN port, and the hand opens + torque-off), then SIGKILL anything
# that ignores it.
#
#   ./cleanup.sh
#
# Safe to run anytime; does nothing if nothing is stuck.
set -u

SELF=$$
TARGETS=(
    run_vr_teleop.sh
    vr_controller_publisher.py
    vr_teleop_rotation.py
    vr_visualizer.py
    vr_hand_control.py
)
ARM_PORT=/dev/ttyACM0
HAND_PORT=/dev/ttyACM1

# Collect matching pids, excluding this script, its parent, and the grep.
pids=()
for pat in "${TARGETS[@]}"; do
    while read -r pid; do
        [ -z "$pid" ] && continue
        [ "$pid" = "$SELF" ] && continue
        [ "$pid" = "$PPID" ] && continue
        pids+=("$pid")
    done < <(pgrep -f -- "$pat" 2>/dev/null || true)
done
# Dedupe.
mapfile -t pids < <(printf '%s\n' "${pids[@]:-}" | sort -un | sed '/^$/d')

if [ "${#pids[@]}" -eq 0 ]; then
    echo "[cleanup] no VR-teleop processes running."
else
    echo "[cleanup] found ${#pids[@]} process(es):"
    for pid in "${pids[@]}"; do
        ps -o pid=,etime=,cmd= -p "$pid" 2>/dev/null | sed 's/^/  /'
    done

    echo "[cleanup] sending SIGINT (lets arm/hand clean up)…"
    for pid in "${pids[@]}"; do kill -INT "$pid" 2>/dev/null || true; done

    # Wait up to ~6s for graceful exit.
    for _ in $(seq 1 24); do
        alive=0
        for pid in "${pids[@]}"; do
            kill -0 "$pid" 2>/dev/null && alive=1
        done
        [ "$alive" = 0 ] && break
        sleep 0.25
    done

    for pid in "${pids[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "[cleanup] PID $pid ignored SIGINT -> SIGKILL"
            kill -9 "$pid" 2>/dev/null || true
        fi
    done
    sleep 0.5
    echo "[cleanup] processes cleared."
fi

# Report / clear serial-port holders.
for port in "$ARM_PORT" "$HAND_PORT"; do
    [ -e "$port" ] || continue
    holders=$(lsof -t "$port" 2>/dev/null | grep -v "^$SELF\$" || true)
    if [ -n "$holders" ]; then
        echo "[cleanup] $port still held by: $holders -> SIGKILL"
        for h in $holders; do kill -9 "$h" 2>/dev/null || true; done
    else
        echo "[cleanup] $port free."
    fi
done

echo "[cleanup] done."
