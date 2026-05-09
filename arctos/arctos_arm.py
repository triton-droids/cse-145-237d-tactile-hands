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
CMD_SET_HOME_PARAMS = 0x90      # trigger level, direction, speed, end-limit, mode
CMD_GO_HOME = 0x91              # run the firmware homing sequence
CMD_SET_AXIS_ZERO = 0x92        # set current encoder position as zero
CMD_QUERY_STATUS = 0xF1         # uint8 motor state
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


class CommandTimeout(ArctosError):
    """No response received within the expected window."""


# ---------------------------------------------------------------------------
# Port resolution (no source-file rewriting)
# ---------------------------------------------------------------------------
def resolve_com_port(preferred: Optional[str] = None) -> str:
    """
    Pick a serial port in this order:
      1. `preferred` argument if given and openable
      2. ARCTOS_COM_PORT env var if set and openable
      3. first available serial port on the system
    Raises ArctosError if no serial ports are present.
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
    if not ports:
        raise ArctosError("No serial ports found. Plug in the SLCAN adapter.")
    chosen = ports[0].device
    print(f"[ARCTOS] Auto-selected port {chosen}")
    return chosen


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
        self.current_angles: Dict[int, float] = {j: 0.0 for j in self.JOINTS}
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
        if sync_encoders:
            self.sync_all_encoders()

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
        print(f"[TX] ID=0x{arb_id:03X} DATA=[{self._hex(payload)}]")
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
        elif code == CMD_QUERY_STATUS:
            self._handle_status_response(joint, body)
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

    def _handle_status_response(self, joint: int, body: bytes) -> None:
        if len(body) != 1:
            return
        self._complete_pending(joint, CMD_QUERY_STATUS, body[0])

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

    def move_joint(
        self,
        joint: int,
        degrees: float,
        rpm: int = DEFAULT_RPM,
        acc: int = DEFAULT_ACC,
        *,
        wait: bool = True,
        timeout: float = DEFAULT_MOVE_TIMEOUT,
    ) -> Future:
        """
        Move a joint by `degrees` relative to its current position.

        Returns the Future that resolves on move completion.
        If `wait` is True (default), blocks until the move finishes or times out.
        Raises LimitHit / MoveFailed / CommandTimeout on failure.
        """
        self._validate_joint(joint)
        if not 0 <= rpm <= 3000:
            raise ValueError(f"rpm must be 0..3000, got {rpm}")
        if not 0 <= acc <= 255:
            raise ValueError(f"acc must be 0..255, got {acc}")
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

    def set_joint_angles(
        self,
        j1: float, j2: float, j3: float, j4: float, j5: float, j6: float,
        rpm: int = DEFAULT_RPM,
        acc: int = DEFAULT_ACC,
        *,
        wait: bool = True,
        timeout: float = DEFAULT_MOVE_TIMEOUT,
    ) -> Dict[int, Future]:
        """
        Move all joints to absolute target angles, in parallel.

        Refreshes encoder positions first (so deltas are computed from
        reality, not a stale commanded value), then dispatches all six moves
        concurrently. Blocks on the whole batch if wait=True.
        """
        targets = {1: j1, 2: j2, 3: j3, 4: j4, 5: j5, 6: j6}
        self.sync_all_encoders()

        with self._state_lock:
            current = dict(self.current_angles)

        futures: Dict[int, Future] = {}
        for joint in self.JOINTS:
            delta = targets[joint] - current[joint]
            if abs(delta) > 0.01:
                futures[joint] = self.move_joint(
                    joint, delta, rpm, acc, wait=False
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
        self, timeout: float = DEFAULT_QUERY_TIMEOUT
    ) -> Dict[int, float]:
        """
        Read all six joints concurrently and update current_angles.
        Returns the fresh angle dict.
        """
        futures: Dict[int, Future] = {}
        for joint in self.JOINTS:
            futures[joint] = self._register_pending(joint, CMD_READ_ENCODER)
            self._send_frame(joint, bytes([CMD_READ_ENCODER]))

        results: Dict[int, float] = {}
        for joint, fut in futures.items():
            try:
                results[joint] = fut.result(timeout=timeout)
            except FutureTimeout:
                self._fail_pending(
                    joint, CMD_READ_ENCODER, CommandTimeout("encoder read")
                )
                raise CommandTimeout(
                    f"joint {joint} encoder read timeout"
                )
        return results

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
        """Send emergency-stop to every joint, best-effort."""
        for joint in self.JOINTS:
            try:
                self.emergency_stop(joint, timeout=timeout)
            except ArctosError as e:
                print(f"[ARCTOS] emergency stop joint {joint}: {e}")

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
