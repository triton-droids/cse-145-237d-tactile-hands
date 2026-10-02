#!/usr/bin/env bash
# Set up a machine to run the ARCTOS arm tools, then prove it with the
# offline self-test. Safe to re-run: each step checks first and only changes
# what is missing.
#
#   ./setup.sh
#
# Targets Ubuntu 24.04 (what ROS 2 Jazzy runs on). The tools run on the
# system python3 -- no venv -- the same interpreter the VR teleop stack uses.
# Dependencies come from apt because 24.04 blocks `pip install` into the
# system python (PEP 668).

set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

# python module -> apt package that provides it
declare -A APT_PKG=(
  [can]=python3-can          # MKS CAN bus over the CANable (slcan)
  [serial]=python3-serial    # serial transport underneath slcan
  [tkinter]=python3-tk       # teleop_gui.py
)

missing=()
for mod in "${!APT_PKG[@]}"; do
  python3 -c "import $mod" 2>/dev/null || missing+=("${APT_PKG[$mod]}")
done

if ((${#missing[@]})); then
  if ! command -v apt-get >/dev/null; then
    echo "missing: ${missing[*]} -- no apt here; install the equivalents" >&2
    echo "(pip names: python-can>=4 pyserial; tkinter from your OS)" >&2
    exit 1
  fi
  echo "installing ${missing[*]} ..."
  sudo apt-get update
  sudo apt-get install -y "${missing[@]}"
else
  echo "python deps: ok"
fi

# The CANable and the hand's servo adapter are /dev/ttyACM*, owned by
# group dialout.
if ! id -nG "$USER" | grep -qw dialout; then
  sudo usermod -aG dialout "$USER"
fi
if id -nG | grep -qw dialout; then
  echo "serial access (dialout): ok"
else
  echo "serial access: $USER was added to dialout -- log out and back in" \
       "before talking to the arm"
fi

# A stray slcand from manual CAN debugging grabs ttys and makes the arm (or
# the hand) look unplugged. Nothing here needs it.
if pgrep -x slcand >/dev/null; then
  echo "WARNING: slcand is running and may be holding a serial port:" \
       "sudo killall slcand, then replug the adapters"
fi

python3 selftest.py
