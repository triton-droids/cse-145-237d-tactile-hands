#!/usr/bin/env python3
"""
Live monitor for a joint's limit-sensor inputs. Moves nothing.

Polls MKS command 0x34 (IO port status) and shows the input pins in real time.
With limit remapping enabled, IN_1 is the En pin and IN_2 is the Dir pin --
the two limit sensors on a two-sensor joint.

Run it, then wave a magnet past each sensor (or push the joint by hand toward
an end) and watch whether the bits change.

  python3 check_sensors.py 3
  python3 check_sensors.py 2 3        # watch several at once

  bit0 IN_1  = En  pin  (limit sensor 1)
  bit1 IN_2  = Dir pin  (limit sensor 2)
  bit2 OUT_1, bit3 OUT_2 = outputs, not sensors
"""
import sys
import time

import can

from arctos_arm import ArctosArm, resolve_com_port

CMD_READ_IO = 0x34


def read_io(bus, joint, timeout=0.3):
    body = bytes([CMD_READ_IO])
    crc = (joint + sum(body)) & 0xFF
    bus.send(can.Message(arbitration_id=joint, data=body + bytes([crc]),
                         is_extended_id=False))
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.time()))
        if msg is None:
            break
        if msg.arbitration_id != joint:
            continue
        data = bytes(msg.data)
        if len(data) >= 3 and data[0] == CMD_READ_IO:
            return data[1]
    return None


def describe(status):
    if status is None:
        return "no reply"
    bits = {
        "IN_1(En)": status & 0x01,
        "IN_2(Dir)": (status >> 1) & 0x01,
        "OUT_1": (status >> 2) & 0x01,
        "OUT_2": (status >> 3) & 0x01,
    }
    return "  ".join(f"{n}={v}" for n, v in bits.items())


def main():
    joints = [int(a) for a in sys.argv[1:]]
    if not joints:
        sys.exit("usage: check_sensors.py <joint> [joint ...]")

    port = resolve_com_port()
    bus = can.Bus(interface="slcan", channel=port, bitrate=ArctosArm.BITRATE)
    time.sleep(0.3)

    print(f"[sensors] {port} — polling IO status. Ctrl+C to stop.\n")
    print("Move each sensor's magnet into range (or hand-move the joint toward")
    print("an end) and watch IN_1 / IN_2. A bit that never changes means that")
    print("input never sees the sensor.\n")

    seen = {j: set() for j in joints}
    last = {j: None for j in joints}
    try:
        while True:
            for j in joints:
                st = read_io(bus, j)
                if st is not None:
                    seen[j].add(st)
                if st != last[j]:
                    stamp = time.strftime("%H:%M:%S")
                    raw = f"0x{st:02X}" if st is not None else "--"
                    print(f"{stamp}  J{j}  {raw}  {describe(st)}   <-- CHANGED")
                    last[j] = st
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\n--- summary ---")
        for j in joints:
            vals = sorted(seen[j])
            if not vals:
                print(f"  J{j}: no replies at all")
                continue
            in1 = {v & 0x01 for v in vals}
            in2 = {(v >> 1) & 0x01 for v in vals}
            print(f"  J{j}: saw states {[f'0x{v:02X}' for v in vals]}")
            print(f"       IN_1(En)  {'CHANGED — sensor works' if len(in1) > 1 else f'stuck at {in1.pop()}'}")
            print(f"       IN_2(Dir) {'CHANGED — sensor works' if len(in2) > 1 else f'stuck at {in2.pop()}'}")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
