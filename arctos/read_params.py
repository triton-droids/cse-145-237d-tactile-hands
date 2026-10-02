#!/usr/bin/env python3
"""Read and compare MKS servo system parameters across joints.

Uses the "read system parameter" command from section 5.9 of the MKS
SERVO42&57D CAN manual: send [0x00, code, CRC] and the board replies with
[code, param..., CRC].  Boards that do not support reading a given parameter
answer with FF FF.

Point it at a working joint and a suspect one to diff their configuration:

  python3 read_params.py 4 5 6
"""
import argparse
import time

import can

from arctos_arm import resolve_com_port

READ_PREFIX = 0x00
BITRATE = 500_000

MODE_NAMES = {
    0: "CR_OPEN", 1: "CR_CLOSE", 2: "CR_vFOC",
    3: "SR_OPEN", 4: "SR_CLOSE", 5: "SR_vFOC",
}


def fmt_ma(p):
    return f"{int.from_bytes(p[:2], 'big')} mA" if len(p) >= 2 else "?"


def fmt_mode(p):
    v = p[0]
    return f"{MODE_NAMES.get(v, '?')} ({v})"


def fmt_hold(p):
    # holdMa 0x00..0x08 maps to 10%..90%
    return f"{(p[0] + 1) * 10}%" if p[0] <= 8 else f"raw {p[0]}"


def fmt_onoff(p):
    return {0: "disabled", 1: "enabled"}.get(p[0], f"raw {p[0]}")


def fmt_raw(p):
    return " ".join(f"{b:02X}" for b in p)


# (code, label, formatter) -- codes from the manual's set-parameter sections
PARAMS = [
    (0x83, "Ma (working current)", fmt_ma),
    (0x9B, "HoldMa (hold current)", fmt_hold),
    (0x82, "Mode (work mode)", fmt_mode),
    (0x84, "MStep (subdivision)", fmt_raw),
    (0x85, "En pin active", fmt_raw),
    (0x86, "Dir (rotation)", fmt_raw),
    (0x88, "Locked-rotor protection", fmt_onoff),
    (0x89, "Subdivision interpolation", fmt_onoff),
    # 0x9E remaps En/Dir into limit inputs; if enabled with nothing wired,
    # those pins float and can trigger a spurious "end limit stopped".
    (0x9E, "Limit port remap", fmt_onoff),
    (0x90, "Home params (raw)", fmt_raw),
]


def read_param(bus, node, code, timeout=0.5):
    """Send a read-parameter request, return the param bytes or None."""
    crc = (node + READ_PREFIX + code) & 0xFF
    bus.send(can.Message(arbitration_id=node,
                         data=bytes([READ_PREFIX, code, crc]),
                         is_extended_id=False))
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.time()))
        if msg is None:
            break
        if msg.arbitration_id != node:
            continue
        data = bytes(msg.data)
        if len(data) < 3 or data[0] != code:
            continue
        param = data[1:-1]
        if param[:2] == b"\xff\xff":
            return "not readable"
        return param
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("joints", nargs="+", type=int, help="joint IDs to compare")
    ap.add_argument("--port", default=None)
    args = ap.parse_args()

    port = resolve_com_port(args.port)
    print(f"[params] using {port}\n")
    bus = can.Bus(interface="slcan", channel=port, bitrate=BITRATE)
    time.sleep(0.3)

    rows = []
    try:
        for code, label, fmt in PARAMS:
            cells = []
            for node in args.joints:
                param = read_param(bus, node, code)
                if param is None:
                    cells.append("no reply")
                elif isinstance(param, str):
                    cells.append(param)
                else:
                    try:
                        cells.append(fmt(param))
                    except Exception:
                        cells.append(fmt_raw(param))
            rows.append((label, cells))
    finally:
        bus.shutdown()

    width = max(len(label) for label, _ in rows) + 2
    header = "".ljust(width) + "".join(f"J{j}".ljust(22) for j in args.joints)
    print(header)
    print("-" * len(header))
    for label, cells in rows:
        print(label.ljust(width) + "".join(c.ljust(22) for c in cells))

    print("\nAny row where the working joint differs from the others is a suspect.")


if __name__ == "__main__":
    main()
