#!/usr/bin/env python3
"""
Named arm poses, stored in poses.json.

A pose is six absolute joint angles in the same frame as joint_limits.json,
plus a record of which joints had a confirmed origin when it was saved.

That last part is the point. Joint angles are only meaningful relative to an
origin, and the encoders reset to 0 on power-on wherever the arm happens to be.
A pose taught against a confirmed origin and replayed against an unknown one
sends the arm somewhere entirely different -- so replay refuses unless the
joints are homed now and were homed then.

  python3 poses.py list
  python3 poses.py show above_pick
  python3 poses.py delete above_pick
"""
import json
import os
import sys
import time
from typing import Dict, Optional

POSES_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "poses.json")

_WARNING = (
    "Joint angles are relative to the origin in force when they were taught. "
    "Encoders reset to 0 on power-on, so a pose is only safe to replay once "
    "the joints listed in 'homed' have a confirmed origin again."
)


def load_poses(path: str = POSES_PATH) -> Dict[str, dict]:
    """All stored poses, keyed by name. Empty if the file is missing."""
    if not os.path.exists(path):
        return {}
    try:
        with open(path) as f:
            doc = json.load(f)
    except (OSError, ValueError) as e:
        print(f"[poses] could not read {path}: {e}")
        return {}
    out = {}
    for name, entry in doc.get("poses", {}).items():
        joints = entry.get("joints", {})
        try:
            angles = {int(k): float(v) for k, v in joints.items()}
        except (TypeError, ValueError):
            print(f"[poses] skipping malformed pose {name!r}")
            continue
        out[name] = {
            "joints": angles,
            "homed": sorted(int(j) for j in entry.get("homed", [])),
            "saved_at": entry.get("saved_at", "unknown"),
        }
    return out


def save_pose(name: str, angles: Dict[int, float], homed=(),
              path: str = POSES_PATH) -> None:
    """Add or replace a pose, preserving everything else in the file."""
    poses = load_poses(path)
    poses[name] = {
        "joints": dict(angles),
        "homed": sorted(int(j) for j in homed),
        "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    _write(poses, path)


def delete_pose(name: str, path: str = POSES_PATH) -> bool:
    poses = load_poses(path)
    if name not in poses:
        return False
    del poses[name]
    _write(poses, path)
    return True


def _write(poses: Dict[str, dict], path: str) -> None:
    doc = {
        "_warning": _WARNING,
        "poses": {
            name: {
                "joints": {str(j): round(a, 4)
                           for j, a in sorted(p["joints"].items())},
                "homed": p["homed"],
                "saved_at": p["saved_at"],
            }
            for name, p in sorted(poses.items())
        },
    }
    with open(path, "w") as f:
        json.dump(doc, f, indent=2)
        f.write("\n")


def check_replayable(pose: dict, homed_now) -> Optional[str]:
    """
    Why this pose must not be replayed right now, or None if it is safe.

    A joint taught against a confirmed origin needs that origin back before its
    angle means the same thing.
    """
    need = set(pose.get("homed", ()))
    missing = sorted(need - set(homed_now))
    if missing:
        return (f"taught with J{', J'.join(str(j) for j in missing)} homed, "
                f"but they have no confirmed origin now — re-home or re-teach")
    return None


def describe(name: str, pose: dict) -> str:
    angles = " ".join(f"J{j}={pose['joints'][j]:+7.2f}"
                      for j in sorted(pose["joints"]))
    homed = ",".join(f"J{j}" for j in pose["homed"]) or "none"
    return f"{name:<18} {angles}   [homed: {homed}]"


def main():
    args = sys.argv[1:]
    poses = load_poses()
    if not args or args[0] == "list":
        if not poses:
            print(f"no poses stored ({POSES_PATH})")
            return
        for name, p in sorted(poses.items()):
            print(describe(name, p))
    elif args[0] == "show" and len(args) > 1:
        p = poses.get(args[1])
        if not p:
            sys.exit(f"no such pose: {args[1]}")
        print(json.dumps(p, indent=2, default=str))
    elif args[0] == "delete" and len(args) > 1:
        print(f"deleted {args[1]}" if delete_pose(args[1])
              else f"no such pose: {args[1]}")
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main()
