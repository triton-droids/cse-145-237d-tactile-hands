#!/usr/bin/env python3
"""
Offline self-test for the ARCTOS arm tools. Needs no hardware.

Run it on a new machine (setup.sh does), and before committing changes to
the arm tools:

  python3 selftest.py
  python3 selftest.py -v        # list every check

Checks that every tool imports (so all dependencies are installed), that no
script uses a name it never defined (the bug class that crashed jog.py
mid-session), and that the safety logic behaves: limit clamping, the
limits-file merge, e-stop releasing a blocked move.
"""
import builtins
import glob
import importlib
import json
import os
import shutil
import symtable
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from arctos_arm import (  # noqa: E402
    ArctosArm, MoveFailed, SoftLimitExceeded, CMD_MOVE_RELATIVE,
    CMD_EMERGENCY_STOP, LIMITS_PATH, _load_limits_file,
)

# The arm tools. vr_* need ROS 2 / openvr, so they are only checked
# statically (TestNames), not imported.
TOOLS = ["arctos_arm", "jog", "home", "park", "poses", "demo_move",
         "teleop_gui", "motion_debug", "check_sensors", "read_pos",
         "read_params", "set_params"]


def undefined_names(source, filename="<src>"):
    """Global names a module reads but never binds (imports, defs, assigns)."""
    top = symtable.symtable(source, filename, "exec")
    known = set(dir(builtins)) | {"__file__", "__name__", "__doc__"}
    known |= {s.get_name() for s in top.get_symbols()
              if s.is_assigned() or s.is_imported() or s.is_namespace()}
    missing = set()

    def walk(table):
        for sym in table.get_symbols():
            if (sym.is_referenced() and sym.get_name() not in known
                    and (table.get_type() == "module" or sym.is_global())):
                missing.add(sym.get_name())
        for child in table.get_children():
            walk(child)

    walk(top)
    return missing


def offline_arm():
    """An ArctosArm that never touches a bus unless a test gives it one."""
    arm = ArctosArm(com_port="/dev/null")
    arm.JOINT_LIMITS = {}
    arm._park_is_origin = set()
    arm._homed = set()
    arm._start_angles = {}
    return arm


class TestEnvironment(unittest.TestCase):
    def test_python_can_is_v4(self):
        import can
        self.assertGreaterEqual(int(can.__version__.split(".")[0]), 4,
                                f"python-can {can.__version__} is too old")

    def test_every_tool_imports(self):
        for name in TOOLS:
            with self.subTest(tool=name):
                importlib.import_module(name)


class TestNames(unittest.TestCase):
    def test_no_script_uses_an_undefined_name(self):
        for path in sorted(glob.glob(os.path.join(HERE, "*.py"))):
            with self.subTest(file=os.path.basename(path)):
                with open(path) as f:
                    self.assertEqual(undefined_names(f.read(), path), set())

    def test_checker_catches_a_missing_import(self):
        src = "import os\ndef f():\n    return save_pose(os.sep)\n"
        self.assertEqual(undefined_names(src), {"save_pose"})


class TestTravelLimit(unittest.TestCase):
    def setUp(self):
        self.arm = offline_arm()
        self.arm.JOINT_LIMITS = {2: (0.0, 100.0)}
        self.arm._homed = {2}

    def limit(self, current, degrees, on_limit="clamp", joint=2):
        return self.arm._apply_travel_limit(joint, degrees, on_limit, current)

    def test_inside_window_passes_unchanged(self):
        self.assertAlmostEqual(self.limit(50.0, 10.0), 10.0)

    def test_overrun_is_clamped_to_the_bound(self):
        self.assertAlmostEqual(self.limit(95.0, 10.0), 5.0)

    def test_overrun_raises_in_raise_mode(self):
        with self.assertRaises(SoftLimitExceeded):
            self.limit(95.0, 10.0, "raise")

    def test_outside_window_is_never_reversed(self):
        # Confirmed parked at -1.5 against a 0 minimum, asked for -0.1:
        # used to come back as +1.5.
        self.assertAlmostEqual(self.limit(-1.5, -0.1), 0.0)
        with self.assertRaises(SoftLimitExceeded):
            self.limit(-1.5, -0.1, "raise")

    def test_outside_window_may_move_back_toward_it(self):
        self.assertAlmostEqual(self.limit(-1.5, 0.5), 0.5)
        self.assertAlmostEqual(self.limit(-1.5, 200.0), 101.5)

    def test_unhomed_joint_uses_travel_from_start(self):
        self.arm._start_angles = {3: 10.0}
        allowance = self.arm.TRAVEL_LIMITS[3]
        self.assertAlmostEqual(self.limit(10.0, allowance + 5.0, joint=3),
                               allowance)

    def test_no_reference_means_no_bound(self):
        self.assertAlmostEqual(self.limit(0.0, 500.0, joint=3), 500.0)


class TestCoordination(unittest.TestCase):
    def test_longest_move_sets_the_pace(self):
        arm = offline_arm()
        rpms = arm._sync_rpms({1: 10.0, 3: 10.0}, 60)
        ratio = arm.GEAR_RATIOS[1] / arm.GEAR_RATIOS[3]
        self.assertEqual(rpms[3], 60)
        self.assertEqual(rpms[1], max(1, round(60 * ratio)))


class FakeBus:
    def __init__(self):
        self.sent = []

    def send(self, msg):
        self.sent.append(msg)


class TestEmergencyStop(unittest.TestCase):
    def test_stop_releases_a_blocked_move(self):
        arm = offline_arm()
        arm.bus = FakeBus()
        move = arm._register_pending(3, CMD_MOVE_RELATIVE)
        arm.emergency_stop_all(timeout=0.01)   # no board answers offline
        self.assertTrue(move.done())
        self.assertIsInstance(move.exception(), MoveFailed)
        stops = [m for m in arm.bus.sent if m.data[0] == CMD_EMERGENCY_STOP]
        self.assertEqual(sorted(m.arbitration_id for m in stops),
                         sorted(arm.JOINTS))


class TestLimitsFile(unittest.TestCase):
    def setUp(self):
        import jog
        self.jog = jog
        self.tmp = tempfile.mkdtemp()
        self.path = os.path.join(self.tmp, "joint_limits.json")
        shutil.copy(LIMITS_PATH, self.path)
        with open(self.path) as f:
            self.original = json.load(f)
        self.margin = float(self.original.get("margin_deg", 2.0))

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def load(self):
        with open(self.path) as f:
            return json.load(f)

    def test_repo_file_is_sane(self):
        limits, park = _load_limits_file(LIMITS_PATH)
        self.assertTrue(limits, "joint_limits.json has no bounded joints")
        for joint, (lo, hi) in limits.items():
            with self.subTest(joint=joint):
                self.assertLess(lo, hi)
                if joint in park:   # park.py drives these to 0
                    self.assertTrue(lo <= 0.0 <= hi,
                                    f"J{joint} park pose 0 is outside "
                                    f"[{lo}, {hi}]")

    def test_save_without_changes_is_lossless(self):
        recorded = self.jog.load_limits(self.path)
        self.jog.save_limits(recorded, self.margin, self.path)
        self.assertEqual(self.load(), self.original)

    def test_new_end_gets_margin_and_keeps_hand_set_fields(self):
        joint = next(k for k, v in self.original["joints"].items()
                     if v.get("park_is_origin") and "measured_max" in v)
        recorded = self.jog.load_limits(self.path)
        new_max = recorded[int(joint)]["max"] + 5.0
        recorded[int(joint)]["max"] = new_max
        self.jog.save_limits(recorded, self.margin, self.path)

        doc = self.load()
        entry, old = doc["joints"][joint], self.original["joints"][joint]
        self.assertAlmostEqual(entry["max"], round(new_max - self.margin, 3))
        self.assertEqual(entry["min"], old["min"])
        self.assertTrue(entry["park_is_origin"])
        self.assertEqual(entry.get("note"), old.get("note"))
        for key, other in self.original["joints"].items():
            if key != joint:
                self.assertEqual(doc["joints"][key], other)
        for key in self.original:
            if key not in ("joints", "measured_at"):
                self.assertEqual(doc[key], self.original[key])

    def test_park_joint_keeps_zero_in_window(self):
        self.original["joints"]["9"] = {"park_is_origin": True}
        with open(self.path, "w") as f:
            json.dump(self.original, f)
        self.jog.save_limits({9: {"min": -0.2, "max": 50.0}}, 3.0, self.path)
        entry = self.load()["joints"]["9"]
        self.assertEqual(entry["min"], 0.0)       # not -0.2 + 3.0
        self.assertEqual(entry["max"], 47.0)

    def test_continuous_drops_bounds_only(self):
        self.jog.save_limits({4: {"continuous": True}}, self.margin, self.path)
        entry = self.load()["joints"]["4"]
        self.assertTrue(entry["continuous"])
        self.assertNotIn("min", entry)
        self.assertEqual(entry.get("note"),
                         self.original["joints"]["4"].get("note"))

    def test_unreadable_file_is_not_replaced(self):
        with open(self.path, "w") as f:
            f.write("{ not json")
        with self.assertRaises(ValueError):
            self.jog.save_limits({3: {"min": 0.0, "max": 1.0}}, 1.0, self.path)
        with open(self.path) as f:
            self.assertEqual(f.read(), "{ not json")


class TestPoses(unittest.TestCase):
    def test_round_trip_and_replay_guard(self):
        import poses
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "poses.json")
            poses.save_pose("above_pick", {1: 1.5, 2: 20.0}, homed=[2],
                            path=path)
            pose = poses.load_poses(path)["above_pick"]
            self.assertEqual(pose["joints"], {1: 1.5, 2: 20.0})
            self.assertIsNotNone(poses.check_replayable(pose, homed_now=[]))
            self.assertIsNone(poses.check_replayable(pose, homed_now=[2]))


if __name__ == "__main__":
    # buffer: the arm's [ARCTOS] chatter is shown only for a failing test.
    unittest.main(buffer=True)
