"""
ARCTOS Arm Controller

Control the ARCTOS 6-DOF robotic arm over CAN bus (MKS SERVO42/57D firmware
V1.0.6, SLCAN adapter).

Design:
  * Transport:   python-can slcan Bus, one background RX thread.
  * RX is the single writer. Every inbound frame is CRC-checked and dispatched
    by (joint, command_code) to a pending concurrent.futures.Future.
  * current_angles[joint] is populated ONLY by 0x31 encoder responses. move
    commands never write it. connect() auto-syncs all six joints before
    returning, so current_angles is always real data.
  * Every TX method creates a Future and either returns it (wait=False) or
    blocks on .result() (wait=True, default). set_joint_angles fires all six
    joint moves in parallel and waits on the batch.
  * No source-file rewriting. Port resolution: explicit arg ->
    ARCTOS_COM_PORT env var -> first available serial port.

Protocol reference: "MKS SERVO42&57D_CAN User Manual V1.0.6", sections 4-6.
Checksum:  CRC = (can_id + sum(body_bytes)) & 0xFF, appended as last byte.
"""

import json
import os
import time
import threading
from concurrent.futures import Future, TimeoutError as FutureTimeout
from typing import Dict, Optional, Sequence, Tuple

import can
import serial
import serial.tools.list_ports


# ---------------------------------------------------------------------------
# MKS command codes used by this controller
# ---------------------------------------------------------------------------
CMD_READ_ENCODER = 0x31         # int48 addition encoder value
CMD_READ_IO = 0x34              # IO port status: bit0 IN_1(En), bit1 IN_2(Dir)
CMD_SET_HOME_PARAMS = 0x90      # trigger level, direction, speed, end-limit, mode
CMD_GO_HOME = 0x91              # run the firmware homing sequence
CMD_SET_AXIS_ZERO = 0x92        # set current encoder position as zero
CMD_QUERY_STATUS = 0xF1         # uint8 motor state
CMD_SPEED_MODE = 0xF6           # run continuously at a commanded speed
CMD_EMERGENCY_STOP = 0xF7       # uint8 status
CMD_MOVE_RELATIVE = 0xFD        # relative move by pulse count

# FD / FE / F4 / F6 move response status byte
MOVE_FAIL = 0
MOVE_STARTED = 1
MOVE_COMPLETE = 2
MOVE_LIMIT_STOPPED = 3

# 0x91 go-home response status byte
HOME_FAIL = 0
HOME_STARTED = 1
HOME_SUCCESS = 2

# 0x90 home-param direction / trigger-level / mode
HOME_DIR_CW = 0
HOME_DIR_CCW = 1
HOME_TRIG_LOW = 0
HOME_TRIG_HIGH = 1
HOME_MODE_SWITCH = 0     # use limit switch
HOME_MODE_NOSWITCH = 1   # stall-based homing (needs Hm_Ma current set)

# F1 query status byte
STATUS_QUERY_FAIL = 0
STATUS_MOTOR_STOP = 1
STATUS_MOTOR_ACCEL = 2
STATUS_MOTOR_DECEL = 3
STATUS_MOTOR_FULL = 4
STATUS_MOTOR_HOMING = 5
STATUS_MOTOR_CALIB = 6


# ---------------------------------------------------------------------------
# Errors
# ---------------------------------------------------------------------------
class ArctosError(Exception):
    """Base class for ARCTOS arm errors."""


class MoveFailed(ArctosError):
    """Motor firmware reported move failure (status=0)."""


class LimitHit(ArctosError):
    """Motor stopped on an end-limit switch (status=3)."""


class SoftLimitExceeded(ArctosError):
    """
    Move refused: it would drive a joint past its software travel limit.

    Distinct from LimitHit, which is the firmware reporting that a physical
    end-limit switch stopped a move already in progress. This one is a
    software guard that stops the command from being sent at all.
    """


class CommandTimeout(ArctosError):
    """No response received within the expected window."""


# ---------------------------------------------------------------------------
# Port resolution (no source-file rewriting)
# ---------------------------------------------------------------------------
# Valve USB vendor ID. The Index controllers / base stations enumerate a
# "Watchman" wireless-receiver serial device that auto-detect must never
# grab (opening it does nothing useful and steals the wrong tty when the
# SLCAN adapter isn't pinned via ARCTOS_COM_PORT).
_VALVE_VID = 0x28DE
_PORT_DENY_SUBSTRINGS = ("watchman", "valve", "vr radio")

# The CAN adapter on this rig is a CANable2 (VID:PID 16D0:117E). Other
# USB-serial devices share the bus now (e.g. the AmazingHand servo
# adapter, a QinHeng 1A86 chip), so auto-detect must deterministically
# PREFER the CANable rather than grab whatever enumerates first.
_CAN_DEBUG = os.environ.get("ARCTOS_CAN_DEBUG", "").lower() in ("1", "true", "yes")
_CANABLE_VID = 0x16D0
_PORT_PREFER_SUBSTRINGS = ("canable", "slcan", "cantact", "gs_usb")


def _is_denied_port(port_info) -> bool:
    if getattr(port_info, "vid", None) == _VALVE_VID:
        return True
    haystack = " ".join(
        str(getattr(port_info, attr, "") or "")
        for attr in ("description", "manufacturer", "product")
    ).lower()
    return any(s in haystack for s in _PORT_DENY_SUBSTRINGS)


def _is_canable_port(port_info) -> bool:
    if getattr(port_info, "vid", None) == _CANABLE_VID:
        return True
    haystack = " ".join(
        str(getattr(port_info, attr, "") or "")
        for attr in ("description", "manufacturer", "product")
    ).lower()
    return any(s in haystack for s in _PORT_PREFER_SUBSTRINGS)


def resolve_com_port(preferred: Optional[str] = None) -> str:
    """
    Pick a serial port in this order:
      1. `preferred` argument if given and openable
      2. ARCTOS_COM_PORT env var if set and openable
      3. first available serial port, skipping known non-CAN devices
         (Valve "Watchman" VR receiver and friends)
    An explicit preferred/env port is used as-is even if it looks denied;
    only the auto-detect scan applies the denylist.
    Raises ArctosError if no usable serial ports are present.
    """
    candidates = []
    if preferred:
        candidates.append(preferred)
    env_port = os.environ.get("ARCTOS_COM_PORT")
    if env_port and env_port not in candidates:
        candidates.append(env_port)

    for port in candidates:
        try:
            s = serial.Serial(port)
            s.close()
            return port
        except (serial.SerialException, OSError):
            continue

    ports = serial.tools.list_ports.comports()
    # Native motherboard UARTs (/dev/ttyS*) always enumerate with no USB
    # VID and are never the SLCAN adapter. Require a real USB device so we
    # fail loudly instead of grabbing /dev/ttyS31.
    usb_ports = [p for p in ports if getattr(p, "vid", None) is not None]
    if not usb_ports:
        raise ArctosError(
            "No USB serial adapter found (only legacy /dev/ttyS* ports). "
            "Plug in the SLCAN/CAN adapter — it should appear as "
            "/dev/ttyACM* or /dev/ttyUSB* — or set ARCTOS_COM_PORT."
        )

    usable = [p for p in usb_ports if not _is_denied_port(p)]
    for p in usb_ports:
        if _is_denied_port(p):
            print(f"[ARCTOS] Skipping {p.device} ({p.description}) — VR/Valve device")
    if not usable:
        raise ArctosError(
            "Only VR/Valve USB serial devices present. Plug in the SLCAN "
            "adapter, or set ARCTOS_COM_PORT to force a specific port."
        )

    # Deterministically prefer the CANable; other USB-serial devices
    # (AmazingHand servo adapter, etc.) share the bus and enumeration
    # order is not stable.
    canable = [p for p in usable if _is_canable_port(p)]
    if canable:
        chosen = canable[0].device
        if len(usable) > 1:
            others = ", ".join(
                f"{p.device}({p.description})" for p in usable
                if p.device != chosen
            )
            print(f"[ARCTOS] Preferring CANable {chosen} over: {others}")
        else:
            print(f"[ARCTOS] Auto-selected port {chosen}")
        return chosen

    chosen = usable[0].device
    print(f"[ARCTOS] No CANable VID/name match; auto-selected {chosen} "
          f"(set ARCTOS_COM_PORT if this is wrong)")
    return chosen


LIMITS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "joint_limits.json")


def _load_limits_file(
    path: str = LIMITS_PATH,
) -> Tuple[Dict[int, Tuple[float, float]], set]:
    """
    Load joint_limits.json: (measured absolute limits, park_is_origin joints).

    The limits are only meaningful once the joint has been homed in the
    current session -- the encoder resets to 0 on power-on at whatever pose
    the arm happens to be in, which is indistinguishable from sitting at the
    reference end. ArctosArm therefore only enforces them for joints it has
    seen zeroed (or confirmed parked).
    """
    if not os.path.exists(path):
        return {}, set()
    try:
        with open(path) as f:
            doc = json.load(f)
    except (OSError, ValueError) as e:
        print(f"[ARCTOS] could not read {path}: {e}")
        return {}, set()
    limits, park = {}, set()
    for key, lim in doc.get("joints", {}).items():
        try:
            joint = int(key)
        except ValueError:
            continue
        if lim.get("park_is_origin"):
            park.add(joint)
        if "min" not in lim or "max" not in lim:
            continue    # continuous axes carry no bounds
        try:
            limits[joint] = (float(lim["min"]), float(lim["max"]))
        except (TypeError, ValueError):
            continue
    return limits, park


# ---------------------------------------------------------------------------
# Main controller
# ---------------------------------------------------------------------------
class ArctosArm:
    """Controller for the ARCTOS 6-DOF robotic arm."""

    # Joint mechanical configuration
    GEAR_RATIOS: Dict[int, float] = {
        1: 24.6,    # J1 (X)
        2: 75.0,    # J2 (Y)
        3: 150.0,   # J3 (Z)
        4: 48.0,    # J4 (A)
        5: 67.82,   # J5 (B)
        6: 67.82,   # J6 (C)
    }
    INVERT_DIRECTION: Dict[int, bool] = {
        1: True,
        2: False,
        3: False,
        4: False,
        5: False,
        6: False,
    }
    JOINTS = (1, 2, 3, 4, 5, 6)

    # ------------------------------------------------------------------
    # Software travel limits (PHASE 0 STOPGAP)
    #
    # These bound how far each joint may travel from wherever it sat when
    # connect() ran -- NOT absolute joint angles. That is deliberate: the
    # encoders reset on power-on, so until the arm is homed there is no
    # meaningful absolute zero to reference limits against. Bounding travel
    # from the session start needs no zero and still stops a runaway.
    #
    # Values are conservative on purpose. Replace with measured per-joint
    # ranges (see joint_limits.json) once the arm homes reliably.
    TRAVEL_LIMITS: Dict[int, float] = {
        1: 25.0,   # J1 base yaw   - swings the whole arm
        2: 15.0,   # J2 shoulder   - carries arm weight against gravity
        3: 15.0,   # J3 elbow      - the joint that nearly overran
        4: 30.0,   # J4 forearm roll
        5: 30.0,   # J5 wrist B
        6: 30.0,   # J6 wrist C
    }

    PPR = 3200               # microsteps per motor revolution (command)
    ENCODER_CPR = 0x4000     # encoder counts per motor revolution (14-bit)
    BITRATE = 500_000
    ARB_BASE = 0x000

    DEFAULT_RPM = 128
    DEFAULT_ACC = 5
    DEFAULT_MOVE_TIMEOUT = 60.0
    DEFAULT_QUERY_TIMEOUT = 1.0

    def __init__(self, com_port: Optional[str] = None):
        self.com_port = com_port
        self.bus: Optional[can.Bus] = None
        # Allow env override so a dead board (e.g. J1 hardware fault)
        # doesn't make connect() time out on encoder sync.
        env_j = os.environ.get("ARCTOS_JOINTS")
        if env_j:
            self.JOINTS = tuple(int(x) for x in env_j.split(",") if x.strip())
        self.current_angles: Dict[int, float] = {j: 0.0 for j in self.JOINTS}
        # Reference pose for TRAVEL_LIMITS, captured by connect(). Empty until
        # then, which disables the guard (no reference = nothing to bound).
        self._start_angles: Dict[int, float] = {}
        # Measured absolute limits, enforced only for joints zeroed this
        # session (see _load_limits_file).
        self.JOINT_LIMITS: Dict[int, Tuple[float, float]]
        self.JOINT_LIMITS, self._park_is_origin = _load_limits_file()
        self._homed: set = set()
        # Serialises bus writes: an e-stop may be sent from another thread
        # while a worker is mid-command (teleop_gui does exactly this).
        self._tx_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._pending: Dict[Tuple[int, int], Future] = {}
        self._pending_lock = threading.Lock()
        self._rx_thread: Optional[threading.Thread] = None
        self._rx_stop = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def connect(self, sync_encoders: bool = True) -> None:
        """Open the CAN bus, start the RX thread, and sync encoder positions."""
        self.com_port = resolve_com_port(self.com_port)
        if self.bus is not None:
            self.disconnect()
        self.bus = can.Bus(
            interface="slcan",
            channel=self.com_port,
            bitrate=self.BITRATE,
        )
        self._rx_stop = False
        self._rx_thread = threading.Thread(
            target=self._rx_loop, daemon=True, name="arctos-rx"
        )
        self._rx_thread.start()
        # The slcan adapter and the MKS boards need a moment after bus
        # init; the very first frame to a node is otherwise often dropped.
        time.sleep(0.3)
        if sync_encoders:
            self.sync_all_encoders()
            # Anchor the travel guard to the pose we booted into.
            with self._state_lock:
                self._start_angles = dict(self.current_angles)
            self._report_park_candidates()
            print("[ARCTOS] travel limits anchored to start pose; "
                  "per-joint allowance (deg): "
                  + ", ".join(f"J{j}=±{self.TRAVEL_LIMITS[j]:g}"
                              for j in sorted(self.JOINTS)
                              if j in self.TRAVEL_LIMITS))

    PARK_ORIGIN_TOLERANCE = 2.0   # degrees

    def _report_park_candidates(self) -> None:
        """
        Report whether the encoder zero *might* be a valid origin -- but do NOT
        act on it.

        An encoder reading of 0 at power-on proves nothing: the encoder resets
        to 0 wherever the arm happens to be. Treating that as "parked at the
        origin" anchors the measured limits to the wrong physical place, and
        the guard then permits travel straight into a mechanical stop. That is
        strictly worse than having no absolute limits at all.

        So the origin is UNKNOWN until something establishes it: zero_here()
        against a physical reference, or an explicit confirm_parked() from an
        operator who has actually looked at the arm. Until then every joint
        stays on the conservative travel-from-start guard.
        """
        candidates = [j for j in sorted(self._park_is_origin)
                      if abs(self.current_angles.get(j, 999.0))
                      <= self.PARK_ORIGIN_TOLERANCE]
        if candidates:
            names = ", ".join(f"J{j}" for j in candidates)
            print(f"[ARCTOS] {names} read ~0°, which is CONSISTENT with being "
                  f"parked at the origin but does not prove it.")
            print("[ARCTOS] Measured limits are NOT enforced. Using the "
                  "conservative travel guard. If the arm really is parked, "
                  "call confirm_parked() (teleop_gui.py has a button for it).")

    def confirm_parked(self, joints: Optional[Sequence[int]] = None) -> None:
        """
        Assert that the arm is physically at its park pose, so the encoder
        zero really is the origin and the measured limits can be enforced.

        Only call this having actually looked at the arm. Getting it wrong
        puts every limit in the wrong physical place.
        """
        if joints is None:
            joints = sorted(self._park_is_origin)
        for joint in joints:
            angle = self.current_angles.get(joint)
            if angle is None:
                continue
            if abs(angle) > self.PARK_ORIGIN_TOLERANCE:
                print(f"[ARCTOS] refusing to confirm J{joint}: reads "
                      f"{angle:+.2f}°, not its park origin.")
                continue
            self._homed.add(joint)
            lo, hi = self.JOINT_LIMITS.get(joint, (0.0, 0.0))
            print(f"[ARCTOS] J{joint} origin confirmed — measured limits "
                  f"enforced: [{lo:+.1f}, {hi:+.1f}]")

    def disconnect(self, stop_motors: bool = True) -> None:
        """
        Stop the RX thread, fail any pending futures, close the bus.

        If stop_motors is True and the bus is live, fires a best-effort
        emergency-stop to every joint first so a Ctrl+C / exception during
        a move doesn't leave the arm crashing into something.
        """
        if stop_motors and self.bus is not None:
            for joint in self.JOINTS:
                try:
                    self._send_frame(joint, bytes([CMD_EMERGENCY_STOP]))
                except Exception as e:
                    print(f"[ARCTOS] stop on disconnect joint {joint}: {e}")

        self._rx_stop = True
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None

        with self._pending_lock:
            pending = list(self._pending.values())
            self._pending.clear()
        for fut in pending:
            if not fut.done():
                fut.set_exception(ArctosError("Bus disconnected before response"))

        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
        return False

    # ------------------------------------------------------------------
    # Low-level frame helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _hex(data: bytes) -> str:
        return " ".join(f"{b:02X}" for b in data)

    @staticmethod
    def _checksum(arb_id: int, body: bytes) -> int:
        # Per MKS manual: CRC = (ID + sum(bytes)) & 0xFF
        # Valid for our IDs 1-6 (low byte == full 11-bit ID).
        return (arb_id + sum(body)) & 0xFF

    def _validate_joint(self, joint: int) -> None:
        if joint not in self.GEAR_RATIOS:
            raise ValueError(
                f"Invalid joint {joint}; expected one of {self.JOINTS}"
            )

    def _require_bus(self) -> can.Bus:
        if self.bus is None:
            raise ArctosError("Not connected. Call connect() first.")
        return self.bus

    def _send_frame(self, joint: int, body: bytes) -> None:
        bus = self._require_bus()
        arb_id = self.ARB_BASE + joint
        payload = body + bytes([self._checksum(arb_id, body)])
        msg = can.Message(
            arbitration_id=arb_id, data=payload, is_extended_id=False
        )
        if _CAN_DEBUG:
            print(f"[TX] ID=0x{arb_id:03X} DATA=[{self._hex(payload)}]")
        with self._tx_lock:
            bus.send(msg)

    # ------------------------------------------------------------------
    # Pending-future bookkeeping
    # ------------------------------------------------------------------
    def _register_pending(self, joint: int, code: int) -> Future:
        key = (joint, code)
        fut: Future = Future()
        with self._pending_lock:
            old = self._pending.get(key)
            if old is not None and not old.done():
                print(
                    f"[ARCTOS] superseding pending 0x{code:02X} on joint {joint}"
                )
                old.set_exception(ArctosError("superseded by new command"))
            self._pending[key] = fut
        return fut

    def _pop_pending(self, joint: int, code: int) -> Optional[Future]:
        with self._pending_lock:
            return self._pending.pop((joint, code), None)

    def _fail_pending(self, joint: int, code: int, exc: Exception) -> None:
        fut = self._pop_pending(joint, code)
        if fut is not None and not fut.done():
            fut.set_exception(exc)

    def _complete_pending(self, joint: int, code: int, result) -> None:
        fut = self._pop_pending(joint, code)
        if fut is not None and not fut.done():
            fut.set_result(result)

    # ------------------------------------------------------------------
    # RX
    # ------------------------------------------------------------------
    def _rx_loop(self) -> None:
        while not self._rx_stop:
            if self.bus is None:
                time.sleep(0.05)
                continue
            try:
                msg = self.bus.recv(timeout=0.5)
            except Exception as e:
                print(f"[RX] bus error: {e}")
                continue
            if msg is None:
                continue

            data = bytes(msg.data)
            arb_id = msg.arbitration_id
            if _CAN_DEBUG:
                print(f"[RX] ID=0x{arb_id:03X} DATA=[{self._hex(data)}]")

            joint = arb_id - self.ARB_BASE
            if joint not in self.GEAR_RATIOS or len(data) < 2:
                continue

            expected = self._checksum(arb_id, data[:-1])
            if expected != data[-1]:
                print(
                    f"[RX] bad CRC on joint {joint} "
                    f"(got 0x{data[-1]:02X}, want 0x{expected:02X})"
                )
                continue

            code = data[0]
            body = data[1:-1]
            try:
                self._dispatch(joint, code, body)
            except Exception as e:
                print(f"[RX] dispatch error on joint {joint} code 0x{code:02X}: {e}")

    def _dispatch(self, joint: int, code: int, body: bytes) -> None:
        if code == CMD_READ_ENCODER:
            self._handle_encoder_response(joint, body)
        elif code == CMD_MOVE_RELATIVE:
            self._handle_move_response(joint, code, body)
        elif code == CMD_GO_HOME:
            self._handle_home_response(joint, body)
        elif code in (CMD_SET_HOME_PARAMS, CMD_SET_AXIS_ZERO):
            self._handle_simple_ack(joint, code, body)
        elif code == CMD_SPEED_MODE:
            self._handle_speed_response(joint, body)
        elif code == CMD_QUERY_STATUS:
            self._handle_status_response(joint, body)
        elif code == CMD_READ_IO:
            self._handle_io_response(joint, body)
        elif code == CMD_EMERGENCY_STOP:
            self._handle_stop_response(joint, body)
        # Unsolicited frames fall through silently (logged by RX loop).

    def _handle_encoder_response(self, joint: int, body: bytes) -> None:
        # 0x31 response body: int48 big-endian signed (6 bytes).
        #
        # Sign convention: in move_joint, positive degrees produce a
        # dir=0 (CCW) motor command on a non-inverted joint. Per the MKS
        # manual, CCW motion DECREASES the encoder value. So to keep the
        # reported joint angle moving in the same direction as the
        # commanded angle, we must negate the raw encoder-derived angle
        # on non-inverted joints (and leave it alone on inverted ones).
        if len(body) != 6:
            print(f"[RX] 0x31 wrong body length on joint {joint}: {len(body)}")
            return
        total_counts = int.from_bytes(body, byteorder="big", signed=True)
        motor_revs = total_counts / self.ENCODER_CPR
        joint_deg = motor_revs * 360.0 / self.GEAR_RATIOS[joint]
        if not self.INVERT_DIRECTION[joint]:
            joint_deg = -joint_deg
        with self._state_lock:
            self.current_angles[joint] = joint_deg
        self._complete_pending(joint, CMD_READ_ENCODER, joint_deg)

    def _handle_move_response(self, joint: int, code: int, body: bytes) -> None:
        # 0xFD response body: [status] (1 byte)
        if len(body) != 1:
            return
        status = body[0]
        if status == MOVE_STARTED:
            # Informational ack. Do not resolve; wait for status=2/3/0.
            return
        if status == MOVE_COMPLETE:
            self._complete_pending(joint, code, "complete")
        elif status == MOVE_LIMIT_STOPPED:
            self._fail_pending(joint, code, LimitHit(f"joint {joint} hit end limit"))
        else:
            self._fail_pending(
                joint, code, MoveFailed(f"joint {joint} move failed (status={status})")
            )

    def _handle_speed_response(self, joint: int, body: bytes) -> None:
        # 0xF6 response body: [status] (1 byte). Speed mode has no
        # "complete": status 1 = run accepted, 2 = at-speed/decel done,
        # 0 = no-op (e.g. speed-0 commanded while already stopped). None
        # of these is an error we act on — the deadman clutch and E-stop
        # are the real safety — so resolve on any reply with the status.
        if len(body) != 1:
            return
        self._complete_pending(joint, CMD_SPEED_MODE, body[0])

    def _handle_status_response(self, joint: int, body: bytes) -> None:
        if len(body) != 1:
            return
        self._complete_pending(joint, CMD_QUERY_STATUS, body[0])

    def _handle_io_response(self, joint: int, body: bytes) -> None:
        # 0x34 response body: [status] -- bit0 IN_1, bit1 IN_2, bit2/3 outputs.
        if len(body) != 1:
            return
        self._complete_pending(joint, CMD_READ_IO, body[0])

    def _handle_stop_response(self, joint: int, body: bytes) -> None:
        if len(body) != 1:
            return
        self._complete_pending(joint, CMD_EMERGENCY_STOP, bool(body[0]))

    def _handle_home_response(self, joint: int, body: bytes) -> None:
        # 0x91 response body: [status] (1 byte). Two-stage: 1=started, 2=success.
        if len(body) != 1:
            return
        status = body[0]
        if status == HOME_STARTED:
            return  # informational ack, wait for completion
        if status == HOME_SUCCESS:
            self._complete_pending(joint, CMD_GO_HOME, "homed")
        else:
            self._fail_pending(
                joint,
                CMD_GO_HOME,
                MoveFailed(f"joint {joint} home failed (status={status})"),
            )

    def _handle_simple_ack(self, joint: int, code: int, body: bytes) -> None:
        # One-byte status: 1 = success, 0 = fail.
        if len(body) != 1:
            return
        if body[0] == 1:
            self._complete_pending(joint, code, True)
        else:
            self._fail_pending(
                joint,
                code,
                MoveFailed(f"joint {joint} command 0x{code:02X} failed"),
            )

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------
    def _degrees_to_pulses(self, degrees: float, joint: int) -> int:
        gear = self.GEAR_RATIOS[joint]
        return int(abs(degrees) / 360.0 * gear * self.PPR)

    @staticmethod
    def _build_move_body(pulses: int, reverse: bool, rpm: int, acc: int) -> bytes:
        # 0xFD relative-move body (7 bytes, excluding CRC):
        #   [FD, dir|speed_hi(4b), speed_lo(8b), acc, pulse_hi, pulse_mid, pulse_lo]
        pulses = abs(pulses) & 0xFFFFFF
        speed_hi = (rpm >> 8) & 0x0F
        dir_byte = (0x80 if reverse else 0x00) | speed_hi
        return bytes([
            CMD_MOVE_RELATIVE,
            dir_byte,
            rpm & 0xFF,
            acc & 0xFF,
            (pulses >> 16) & 0xFF,
            (pulses >> 8) & 0xFF,
            pulses & 0xFF,
        ])

    def _limit_window(self, joint: int):
        """
        The bounds in force for a joint, as (low, high, description).

        A joint zeroed this session is held to its measured absolute limits.
        Everything else falls back to bounded travel from the connect pose,
        which needs no origin and still stops a runaway.
        """
        if joint in self._homed and joint in self.JOINT_LIMITS:
            lo, hi = self.JOINT_LIMITS[joint]
            return lo, hi, "measured"
        allowance = self.TRAVEL_LIMITS.get(joint)
        if allowance is None or joint not in self._start_angles:
            return None
        start = self._start_angles[joint]
        return start - allowance, start + allowance, "travel-from-start"

    def _apply_travel_limit(
        self,
        joint: int,
        degrees: float,
        on_limit: str,
        current: Optional[float],
    ) -> float:
        """
        Bound a relative move so the joint stays within TRAVEL_LIMITS of the
        pose captured at connect(). Returns the (possibly shortened) delta.

        Checks the PREDICTED end position, not the current one -- catching the
        overrun before the frame goes out is the whole point.

        A joint already outside its window (e.g. confirmed parked at -1.5°
        against a 0° minimum) may move back toward it but never further out.
        Clamping against the raw window would instead yank it into range --
        turning a small negative jog into a larger positive one.
        """
        if on_limit not in ("clamp", "raise"):
            raise ValueError(f"on_limit must be 'clamp' or 'raise', got {on_limit!r}")

        window = self._limit_window(joint)
        if window is None:
            return degrees  # no reference pose or no limit configured
        low, high, source = window

        if current is None:
            # Cached angles are only refreshed by encoder reads, so a stale
            # value here would defeat the guard. Pay for a fresh one.
            try:
                current = self.read_encoder(joint)
            except ArctosError:
                with self._state_lock:
                    current = self.current_angles[joint]

        predicted = current + degrees
        # Widen the window to include where the joint already is, so the
        # clamp can only shorten a move, never reverse it.
        lo, hi = min(low, current), max(high, current)
        if lo <= predicted <= hi:
            return degrees

        if on_limit == "raise":
            raise SoftLimitExceeded(
                f"joint {joint}: move to {predicted:+.2f}° exceeds "
                f"{source} limit [{low:+.2f}, {high:+.2f}]"
            )

        clamped = min(max(predicted, lo), hi)
        allowed = clamped - current
        print(
            f"[ARCTOS] joint {joint} move clamped: {degrees:+.2f}° -> "
            f"{allowed:+.2f}° (would have reached {predicted:+.2f}°, "
            f"limit [{low:+.2f}, {high:+.2f}])"
        )
        return allowed

    def move_joint(
        self,
        joint: int,
        degrees: float,
        rpm: int = DEFAULT_RPM,
        acc: int = DEFAULT_ACC,
        *,
        wait: bool = True,
        timeout: float = DEFAULT_MOVE_TIMEOUT,
        on_limit: str = "clamp",
        current: Optional[float] = None,
    ) -> Future:
        """
        Move a joint by `degrees` relative to its current position.

        The move is checked against TRAVEL_LIMITS before anything is sent.
        `on_limit` is "clamp" (shorten the move to the limit, warn) or "raise"
        (refuse with SoftLimitExceeded). Scripted sequences should pass
        "raise" -- a silently shortened move corrupts a taught trajectory.

        `current` supplies an already-known joint angle so the guard does not
        have to spend a round trip re-reading the encoder; callers that just
        synced (e.g. set_joint_angles) should pass it.

        Returns the Future that resolves on move completion.
        If `wait` is True (default), blocks until the move finishes or times out.
        Raises LimitHit / MoveFailed / CommandTimeout on failure.
        """
        self._validate_joint(joint)
        if not 0 <= rpm <= 3000:
            raise ValueError(f"rpm must be 0..3000, got {rpm}")
        if not 0 <= acc <= 255:
            raise ValueError(f"acc must be 0..255, got {acc}")
        degrees = self._apply_travel_limit(joint, degrees, on_limit, current)
        pulses = self._degrees_to_pulses(degrees, joint)
        if pulses == 0:
            # No-op; do not emit a stop frame (FD with pulses=0 is "stop slowly").
            fut: Future = Future()
            fut.set_result("skipped")
            return fut

        reverse = (degrees < 0) ^ self.INVERT_DIRECTION[joint]
        body = self._build_move_body(pulses, reverse, rpm, acc)
        fut = self._register_pending(joint, CMD_MOVE_RELATIVE)
        self._send_frame(joint, body)

        if wait:
            try:
                fut.result(timeout=timeout)
            except FutureTimeout:
                self._fail_pending(
                    joint,
                    CMD_MOVE_RELATIVE,
                    CommandTimeout(f"joint {joint} move timeout"),
                )
                raise CommandTimeout(
                    f"joint {joint} move did not complete within {timeout}s"
                )
        return fut

    def check_pose(
        self, targets: Dict[int, float]
    ) -> Dict[int, Tuple[float, float, float, str]]:
        """
        Find every joint whose target lies outside its enforced limits.

        Returns {joint: (target, low, high, source)} for the violations, empty
        if the pose is reachable. Checking the whole pose up front lets a
        caller refuse to move at all rather than discovering the third joint is
        out of range after two have already moved.
        """
        bad = {}
        for joint, target in targets.items():
            window = self._limit_window(joint)
            if window is None:
                continue
            low, high, source = window
            if not low <= target <= high:
                bad[joint] = (target, low, high, source)
        return bad

    def _sync_rpms(
        self, deltas: Dict[int, float], rpm: int
    ) -> Dict[int, int]:
        """
        Per-joint speeds that make every joint finish at the same moment.

        Gear ratios span 24.6:1 to 150:1, so at one shared rpm a 10 deg move
        takes 1.0s on J1 and 6.2s on J3 -- the arm does not travel a
        coordinated path, joints just arrive whenever they arrive. Scaling each
        joint's rpm by the motor revolutions it must turn fixes that: the
        longest move runs at the requested rpm and everything else is slowed to
        match.

        The MKS speed field is an integer, so a joint needing under 1 rpm is
        pinned there and arrives early; the caller is told which.
        """
        revs = {j: abs(d) * self.GEAR_RATIOS[j] / 360.0 for j, d in deltas.items()}
        slowest = max(revs.values(), default=0.0)
        if slowest <= 0:
            return {j: rpm for j in deltas}

        out, early = {}, []
        for joint, r in revs.items():
            scaled = rpm * (r / slowest)
            if scaled < 1.0:
                early.append(joint)
            out[joint] = max(1, min(3000, int(round(scaled))))
        if early:
            print(f"[ARCTOS] joints {early} move too little to slow further "
                  f"(under 1 rpm); they will arrive early")

        # The rpm field is an integer, so a joint that wants 3.4 rpm gets 3 --
        # a 12% timing error at the low end that no amount of arithmetic here
        # can remove. Report it rather than claim exact synchronisation.
        durations = {j: revs[j] / out[j] * 60.0 for j in revs if revs[j] > 0}
        if len(durations) > 1:
            longest, shortest = max(durations.values()), min(durations.values())
            spread = (longest - shortest) / longest if longest else 0.0
            if spread > 0.10:
                worst = min(durations, key=durations.get)
                print(f"[ARCTOS] sync within {spread * 100:.0f}% "
                      f"(J{worst} rounds to {out[worst]} rpm); "
                      f"raise rpm or shorten the move to tighten it")
        return out

    def set_joint_angles(
        self,
        j1: float, j2: float, j3: float, j4: float, j5: float, j6: float,
        rpm: int = DEFAULT_RPM,
        acc: int = DEFAULT_ACC,
        *,
        wait: bool = True,
        timeout: float = DEFAULT_MOVE_TIMEOUT,
        on_limit: str = "clamp",
        sync: bool = True,
    ) -> Dict[int, Future]:
        """
        Move all joints to absolute target angles, in parallel.

        Refreshes encoder positions first (so deltas are computed from
        reality, not a stale commanded value), then dispatches all six moves
        concurrently. Blocks on the whole batch if wait=True.

        `sync` (default) scales each joint's rpm so they all finish together;
        `rpm` then sets the pace of the longest-travelling joint. Pass
        sync=False for the old behaviour of one shared rpm.

        `on_limit` is forwarded to move_joint; pass "raise" for taught
        trajectories where a clamped joint would desynchronise the pose. With
        "raise" the whole pose is validated before anything moves, so a
        violation costs no motion at all.
        """
        targets = {1: j1, 2: j2, 3: j3, 4: j4, 5: j5, 6: j6}
        self.sync_all_encoders()

        with self._state_lock:
            current = dict(self.current_angles)

        if on_limit == "raise":
            bad = self.check_pose(targets)
            if bad:
                detail = "; ".join(
                    f"J{j} -> {t:+.2f}° outside {src} [{lo:+.2f}, {hi:+.2f}]"
                    for j, (t, lo, hi, src) in sorted(bad.items())
                )
                raise SoftLimitExceeded(
                    f"pose rejected, nothing moved: {detail}"
                )

        deltas = {j: targets[j] - current[j] for j in self.JOINTS
                  if abs(targets[j] - current[j]) > 0.01}
        rpms = self._sync_rpms(deltas, rpm) if sync else {j: rpm for j in deltas}

        futures: Dict[int, Future] = {}
        for joint, delta in deltas.items():
            futures[joint] = self.move_joint(
                joint, delta, rpms[joint], acc, wait=False,
                on_limit=on_limit, current=current[joint],
            )

        if wait:
            for joint, fut in futures.items():
                try:
                    fut.result(timeout=timeout)
                except FutureTimeout:
                    self._fail_pending(
                        joint,
                        CMD_MOVE_RELATIVE,
                        CommandTimeout(f"joint {joint} move timeout"),
                    )
                    raise CommandTimeout(
                        f"joint {joint} move did not complete within {timeout}s"
                    )
        return futures

    # ------------------------------------------------------------------
    # Speed (velocity) mode
    # ------------------------------------------------------------------
    @staticmethod
    def _build_speed_body(rpm: int, reverse: bool, acc: int) -> bytes:
        # 0xF6 speed-mode body (3 bytes, excluding CRC):
        #   [F6, dir|speed_hi(4b), speed_lo(8b), acc]
        # Same dir/speed packing as the 0xFD relative-move body, minus the
        # pulse count. rpm=0 with a non-zero acc is a graceful decel-to-stop.
        rpm = abs(rpm) & 0x0FFF
        speed_hi = (rpm >> 8) & 0x0F
        dir_byte = (0x80 if reverse else 0x00) | speed_hi
        return bytes([CMD_SPEED_MODE, dir_byte, rpm & 0xFF, acc & 0xFF])

    def set_joint_speed(
        self,
        joint: int,
        rpm: int,
        acc: int = DEFAULT_ACC,
        *,
        wait: bool = True,
        timeout: float = DEFAULT_QUERY_TIMEOUT,
    ) -> Future:
        """
        Run a joint continuously in speed mode (0xF6).

        `rpm` is signed motor RPM; its sign sets direction (then XORed with
        INVERT_DIRECTION[joint]). `rpm=0` issues a graceful stop — the
        firmware decelerates at `acc`. Speed mode never "completes": the
        motor acks immediately and keeps spinning until the next
        set_joint_speed call (or an emergency stop).

        Designed for teleop: call with wait=False at loop rate. Note this
        does NOT update current_angles — call read_encoder for truth.
        """
        self._validate_joint(joint)
        if abs(rpm) > 3000:
            raise ValueError(f"rpm must be -3000..3000, got {rpm}")
        if not 0 <= acc <= 255:
            raise ValueError(f"acc must be 0..255, got {acc}")
        reverse = (rpm < 0) ^ self.INVERT_DIRECTION[joint]
        body = self._build_speed_body(int(rpm), reverse, acc)
        fut = self._register_pending(joint, CMD_SPEED_MODE)
        self._send_frame(joint, body)
        if wait:
            try:
                fut.result(timeout=timeout)
            except FutureTimeout:
                self._fail_pending(
                    joint, CMD_SPEED_MODE, CommandTimeout("speed mode")
                )
                raise CommandTimeout(
                    f"joint {joint} speed-mode command timeout"
                )
        return fut

    def stop_joint(
        self,
        joint: int,
        acc: int = DEFAULT_ACC,
        *,
        wait: bool = True,
        timeout: float = DEFAULT_QUERY_TIMEOUT,
    ) -> Future:
        """Graceful speed-mode stop: decelerate this joint to rest at `acc`."""
        return self.set_joint_speed(
            joint, 0, acc, wait=wait, timeout=timeout
        )

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------
    def read_encoder(
        self, joint: int, timeout: float = DEFAULT_QUERY_TIMEOUT
    ) -> float:
        """Request and return the joint angle in degrees. Blocks."""
        self._validate_joint(joint)
        fut = self._register_pending(joint, CMD_READ_ENCODER)
        self._send_frame(joint, bytes([CMD_READ_ENCODER]))
        try:
            return fut.result(timeout=timeout)
        except FutureTimeout:
            self._fail_pending(
                joint, CMD_READ_ENCODER, CommandTimeout("encoder read")
            )
            raise CommandTimeout(f"joint {joint} encoder read timeout")

    def sync_all_encoders(
        self,
        timeout: float = DEFAULT_QUERY_TIMEOUT,
        retries: int = 3,
    ) -> Dict[int, float]:
        """
        Read all six joints concurrently and update current_angles.

        A node sometimes drops the first encoder request after bus init,
        so timed-out joints are retried up to `retries` times before
        giving up. Returns the fresh angle dict.
        """
        results: Dict[int, float] = {}
        pending_joints = list(self.JOINTS)

        for attempt in range(1, retries + 1):
            futures: Dict[int, Future] = {}
            for joint in pending_joints:
                futures[joint] = self._register_pending(joint, CMD_READ_ENCODER)
                self._send_frame(joint, bytes([CMD_READ_ENCODER]))

            still_pending = []
            for joint, fut in futures.items():
                try:
                    results[joint] = fut.result(timeout=timeout)
                except FutureTimeout:
                    self._fail_pending(
                        joint, CMD_READ_ENCODER, CommandTimeout("encoder read")
                    )
                    still_pending.append(joint)

            if not still_pending:
                return results

            pending_joints = still_pending
            if attempt < retries:
                print(
                    f"[ARCTOS] encoder read retry {attempt} for joints "
                    f"{still_pending}"
                )

        raise CommandTimeout(
            f"joints {pending_joints} did not respond to encoder read "
            f"after {retries} attempts"
        )

    def query_status(
        self, joint: int, timeout: float = DEFAULT_QUERY_TIMEOUT
    ) -> int:
        """
        Query motor state. Returns one of STATUS_MOTOR_* (stop / accel /
        decel / full / homing / calibrating) or STATUS_QUERY_FAIL.
        """
        self._validate_joint(joint)
        fut = self._register_pending(joint, CMD_QUERY_STATUS)
        self._send_frame(joint, bytes([CMD_QUERY_STATUS]))
        try:
            return fut.result(timeout=timeout)
        except FutureTimeout:
            self._fail_pending(
                joint, CMD_QUERY_STATUS, CommandTimeout("status query")
            )
            raise CommandTimeout(f"joint {joint} status query timeout")

    def emergency_stop(
        self, joint: int, timeout: float = DEFAULT_QUERY_TIMEOUT
    ) -> bool:
        """Emergency-stop a single joint. Returns True on firmware ack."""
        self._validate_joint(joint)
        fut = self._register_pending(joint, CMD_EMERGENCY_STOP)
        self._send_frame(joint, bytes([CMD_EMERGENCY_STOP]))
        try:
            return fut.result(timeout=timeout)
        except FutureTimeout:
            self._fail_pending(
                joint, CMD_EMERGENCY_STOP, CommandTimeout("emergency stop")
            )
            raise CommandTimeout(f"joint {joint} emergency stop timeout")

    def emergency_stop_all(
        self, timeout: float = DEFAULT_QUERY_TIMEOUT
    ) -> None:
        """
        Send emergency-stop to every joint, best-effort.

        Also fails any pending move: a stopped move never reports completion,
        so whoever is blocked waiting on it is released now rather than at the
        move timeout. Safe to call from another thread.
        """
        for joint in self.JOINTS:
            try:
                self.emergency_stop(joint, timeout=timeout)
            except ArctosError as e:
                print(f"[ARCTOS] emergency stop joint {joint}: {e}")
            self._fail_pending(joint, CMD_MOVE_RELATIVE,
                               MoveFailed(f"joint {joint}: emergency stop"))

    # ------------------------------------------------------------------
    # Homing
    # ------------------------------------------------------------------
    def set_home_params(
        self,
        joint: int,
        home_dir: int,
        home_speed: int,
        end_limit: bool = False,
        trigger_level: int = HOME_TRIG_LOW,
        mode: int = HOME_MODE_SWITCH,
        *,
        timeout: float = DEFAULT_QUERY_TIMEOUT,
    ) -> bool:
        """
        Configure firmware homing for a joint (0x90).

        home_dir: HOME_DIR_CW (0) or HOME_DIR_CCW (1).
        home_speed: RPM, 0..3000.
        end_limit: enable EndLimit-style homing (stop on switch).
        trigger_level: HOME_TRIG_LOW / HOME_TRIG_HIGH.
        mode: HOME_MODE_SWITCH (default) or HOME_MODE_NOSWITCH.
        """
        self._validate_joint(joint)
        if home_dir not in (HOME_DIR_CW, HOME_DIR_CCW):
            raise ValueError(f"home_dir must be 0 or 1, got {home_dir}")
        if not 0 <= home_speed <= 3000:
            raise ValueError(f"home_speed must be 0..3000 RPM, got {home_speed}")
        if trigger_level not in (HOME_TRIG_LOW, HOME_TRIG_HIGH):
            raise ValueError(f"trigger_level must be 0 or 1, got {trigger_level}")
        if mode not in (HOME_MODE_SWITCH, HOME_MODE_NOSWITCH):
            raise ValueError(f"mode must be 0 or 1, got {mode}")
        body = bytes([
            CMD_SET_HOME_PARAMS,
            trigger_level & 0xFF,
            home_dir & 0xFF,
            (home_speed >> 8) & 0xFF,
            home_speed & 0xFF,
            1 if end_limit else 0,
            mode & 0xFF,
        ])
        fut = self._register_pending(joint, CMD_SET_HOME_PARAMS)
        self._send_frame(joint, body)
        try:
            return fut.result(timeout=timeout)
        except FutureTimeout:
            self._fail_pending(
                joint, CMD_SET_HOME_PARAMS, CommandTimeout("set home params")
            )
            raise CommandTimeout(f"joint {joint} set-home-params timeout")

    def home_joint(self, joint: int, *, timeout: float = 120.0) -> None:
        """
        Run the firmware homing sequence (0x91) for one joint.

        Blocks until the motor reports HOME_SUCCESS, then pins
        current_angles[joint] = 0.0. Raises MoveFailed on HOME_FAIL
        or CommandTimeout if no completion arrives.
        """
        self._validate_joint(joint)
        fut = self._register_pending(joint, CMD_GO_HOME)
        self._send_frame(joint, bytes([CMD_GO_HOME]))
        try:
            fut.result(timeout=timeout)
        except FutureTimeout:
            self._fail_pending(joint, CMD_GO_HOME, CommandTimeout("home"))
            raise CommandTimeout(
                f"joint {joint} did not finish homing within {timeout}s"
            )
        with self._state_lock:
            self.current_angles[joint] = 0.0

    def read_io(self, joint: int, *, timeout: float = DEFAULT_QUERY_TIMEOUT) -> int:
        """
        Read the IO port status byte (0x34).

        bit0 = IN_1, bit1 = IN_2, bit2 = OUT_1, bit3 = OUT_2. With the limit
        port remap enabled, IN_1 is the En pin and IN_2 is the Dir pin -- the
        two limit sensors on a two-sensor joint.

        Reading these directly lets us home against a sensor in software,
        without the firmware's EndLimit function (which only takes effect
        after a firmware homing cycle, and unlocks the shaft when it trips).
        """
        self._validate_joint(joint)
        fut = self._register_pending(joint, CMD_READ_IO)
        self._send_frame(joint, bytes([CMD_READ_IO]))
        try:
            return fut.result(timeout=timeout)
        except FutureTimeout:
            self._fail_pending(joint, CMD_READ_IO, CommandTimeout("read io"))
            raise CommandTimeout(f"joint {joint} IO read timeout")

    def zero_here(
        self, joint: int, *, timeout: float = DEFAULT_QUERY_TIMEOUT
    ) -> bool:
        """
        Mark the current encoder position as zero (0x92).

        No motion. Useful for manual zeroing; prefer home_joint() on boot
        so the zero is tied to a physical reference.
        """
        self._validate_joint(joint)
        fut = self._register_pending(joint, CMD_SET_AXIS_ZERO)
        self._send_frame(joint, bytes([CMD_SET_AXIS_ZERO]))
        try:
            ok = fut.result(timeout=timeout)
        except FutureTimeout:
            self._fail_pending(
                joint, CMD_SET_AXIS_ZERO, CommandTimeout("zero here")
            )
            raise CommandTimeout(f"joint {joint} zero-here timeout")
        if ok:
            with self._state_lock:
                self.current_angles[joint] = 0.0
            # The joint now has a real reference, so its measured absolute
            # limits become meaningful for the rest of this session.
            self._start_angles[joint] = 0.0
            self._homed.add(joint)
            if joint in self.JOINT_LIMITS:
                lo, hi = self.JOINT_LIMITS[joint]
                print(f"[ARCTOS] joint {joint} homed — measured limits now "
                      f"enforced: [{lo:+.1f}, {hi:+.1f}]")
        return ok

    def home_all(
        self,
        order: Optional[Sequence[int]] = None,
        *,
        timeout: float = 120.0,
    ) -> None:
        """
        Home joints sequentially (one at a time, never parallel) so the
        arm cannot collide with itself while chasing limit switches.
        """
        seq = tuple(order) if order is not None else self.JOINTS
        for joint in seq:
            self._validate_joint(joint)
        for joint in seq:
            print(f"[ARCTOS] homing joint {joint}")
            self.home_joint(joint, timeout=timeout)


# Example usage
if __name__ == "__main__":
    with ArctosArm() as arm:
        print("Synced positions:", arm.current_angles)
        arm.move_joint(1, -15)
        print("J1 now at:", arm.read_encoder(1))
