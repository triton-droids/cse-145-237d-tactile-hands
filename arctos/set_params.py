#!/usr/bin/env python3
"""Write MKS servo system parameters over CAN.

Settings persist in board EEPROM, so a change here outlives a power cycle.
Read them back with read_params.py afterwards to confirm.

  # give the wrist boards the same stall shutdown J4 already has
  python3 set_params.py 5 6 --protection on

  # raise working current (SERVO42D: multiples of 200, max 3000)
  python3 set_params.py 5 6 --ma 2400
"""
import argparse
import sys
import time

import can

from arctos_arm import resolve_com_port

BITRATE = 500_000
CMD_SET_MA = 0x83
CMD_SET_PROTECTION = 0x88
MAX_MA = 3000


def send_set(bus, node, code, payload, timeout=1.0):
    """Send a set-parameter frame; return True if the board reports success."""
    body = bytes([code]) + payload
    crc = (node + sum(body)) & 0xFF
    bus.send(can.Message(arbitration_id=node, data=body + bytes([crc]),
                         is_extended_id=False))
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.time()))
        if msg is None:
            break
        if msg.arbitration_id != node:
            continue
        data = bytes(msg.data)
        if len(data) >= 3 and data[0] == code:
            return data[1] == 1
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("joints", nargs="+", type=int)
    ap.add_argument("--ma", type=int,
                    help=f"working current in mA (multiples of 200, max {MAX_MA})")
    ap.add_argument("--protection", choices=["on", "off"],
                    help="locked-rotor (stall) protection")
    ap.add_argument("--port", default=None)
    args = ap.parse_args()

    if args.ma is None and args.protection is None:
        sys.exit("nothing to do: pass --ma and/or --protection")

    if args.ma is not None:
        if not 0 <= args.ma <= MAX_MA or args.ma % 200:
            sys.exit(f"--ma must be a multiple of 200 between 0 and {MAX_MA}")

    port = resolve_com_port(args.port)
    print(f"[params] using {port}\n")
    bus = can.Bus(interface="slcan", channel=port, bitrate=BITRATE)
    time.sleep(0.3)

    try:
        for node in args.joints:
            if args.protection is not None:
                val = 1 if args.protection == "on" else 0
                ok = send_set(bus, node, CMD_SET_PROTECTION, bytes([val]))
                print(f"  J{node} locked-rotor protection -> {args.protection}: "
                      f"{'OK' if ok else 'FAILED' if ok is False else 'no reply'}")
            if args.ma is not None:
                ok = send_set(bus, node, CMD_SET_MA,
                              args.ma.to_bytes(2, "big"))
                print(f"  J{node} Ma -> {args.ma} mA: "
                      f"{'OK' if ok else 'FAILED' if ok is False else 'no reply'}")
    finally:
        bus.shutdown()

    print("\nVerify with:  python3 read_params.py " +
          " ".join(str(j) for j in args.joints))


if __name__ == "__main__":
    main()
