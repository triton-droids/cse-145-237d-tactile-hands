#!/usr/bin/env python3
"""
Read joint positions without touching the motors.

Deliberately does NOT use ArctosArm.connect(): that syncs all six encoders and
its matching disconnect() fires an emergency stop at every joint. This just
opens the bus, asks for encoder values, and closes.

  python3 read_pos.py          # all six joints
  python3 read_pos.py 4        # just J4
"""
import sys
import time

import can

from arctos_arm import ArctosArm, resolve_com_port

CMD_READ_ENCODER = 0x31


def read_joint(bus, joint, timeout=0.6):
    body = bytes([CMD_READ_ENCODER])
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
        if len(data) != 8 or data[0] != CMD_READ_ENCODER:
            continue
        counts = int.from_bytes(data[1:7], byteorder="big", signed=True)
        revs = counts / ArctosArm.ENCODER_CPR
        deg = revs * 360.0 / ArctosArm.GEAR_RATIOS[joint]
        if not ArctosArm.INVERT_DIRECTION[joint]:
            deg = -deg          # same sign convention as _handle_encoder_response
        return counts, revs, deg
    return None


def main():
    joints = [int(a) for a in sys.argv[1:]] or list(ArctosArm.JOINTS)
    try:
        port = resolve_com_port()
        bus = can.Bus(interface="slcan", channel=port, bitrate=ArctosArm.BITRATE)
    except Exception as e:
        sys.exit(f"cannot open CAN bus: {e}\n"
                 f"(is jog.py or another tool still holding the port?)")
    time.sleep(0.3)
    print(f"[read_pos] {port}\n")
    print(f"{'joint':<22} {'degrees':>10} {'motor revs':>12} {'counts':>12}")
    print("-" * 60)
    try:
        for j in joints:
            r = read_joint(bus, j)
            name = f"J{j} ({ArctosArm.GEAR_RATIOS[j]:g}:1)"
            if r is None:
                print(f"{name:<22} {'no reply':>10}")
            else:
                counts, revs, deg = r
                print(f"{name:<22} {deg:>+10.3f} {revs:>+12.4f} {counts:>12d}")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
