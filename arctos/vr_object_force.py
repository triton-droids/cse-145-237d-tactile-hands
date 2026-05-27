"""
Object detection -> per-object grasp force threshold (ROS publisher).

Runs the fine-tuned YOLO26 detector on the object-facing USB webcam and
publishes the force threshold the AmazingHand should stop closing at for
whatever object is in view. vr_hand_control.py subscribes and clamps each
finger so the grasp halts once that finger's FSR exceeds the active limit.

This is the ONLY consumer of the object camera + YOLO — same "one publisher
owns the device, N consumers read ROS" design as vr_controller_publisher.py.

Run under SYSTEM python with ROS sourced (see run_vr_teleop.sh). ultralytics
+ torch must be pip-installed into the system python3 on this rig (rclpy is
not importable from conda), and the chosen .pt copied next to this script (or
point --model-path / $YOLO_MODEL_DIR at it):

    source /opt/ros/jazzy/setup.bash
    /usr/bin/python3 vr_object_force.py                # yolo26s, detect-once-lock
    /usr/bin/python3 vr_object_force.py --model l      # yolo26l (more accurate)
    /usr/bin/python3 vr_object_force.py --continuous    # re-detect every frame
    /usr/bin/python3 vr_object_force.py --show          # annotated preview window

Publishes:
    /hand/force_limit   std_msgs/Float32   active threshold (raw FSR ADC)
    /hand/object_label  std_msgs/String    detected class name, or ""

Modes:
    detect-once-lock (default) — once a class is seen confidently for
        LOCK_STREAK consecutive frames, lock that threshold for the grasp and
        keep publishing it. Restart the node (or press 'r' in --show) for the
        next object. Avoids the threshold flickering mid-grasp.
    --continuous — publish the current top detection's threshold every frame;
        falls back to DEFAULT_FORCE_LIMIT when nothing is detected.
"""

import argparse
import os
import time

import cv2
import rclpy
from std_msgs.msg import Float32, String
from ultralytics import YOLO

from vr_ros_io import (
    DEFAULT_FORCE_LIMIT,
    OBJECT_FORCE_LIMITS,
    TOPIC_FORCE_LIMIT,
    TOPIC_OBJECT_LABEL,
    ros_init_no_signals,
)

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.environ.get("YOLO_MODEL_DIR", HERE)

# The two fine-tuned variants. PyTorch .pt on this (Linux) rig — the CoreML
# .mlpackage variants are Apple-only. Selected with --model {s,l}.
MODELS = {
    "s": "yolo26s_finetuned_TH_objects.pt",
    "l": "yolo26l_finetuned_TH_objects.pt",   # add this .pt to use --model l
}

CAMERA_INDEX = 1          # object-facing USB webcam (v4l index)
CONF = 0.6                # min detection confidence
PUBLISH_HZ = 10.0         # detection cadence; the hand loop runs faster
LOCK_STREAK = 5           # consecutive consistent frames before locking
IMGSZ = 640


def resolve_model_path(args):
    if args.model_path:
        return args.model_path
    return os.path.join(MODEL_DIR, MODELS[args.model])


def top_label(result, conf):
    """Highest-confidence detection's class name >= conf, else None."""
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return None
    best_i, best_c = -1, conf
    for i in range(len(boxes)):
        c = float(boxes.conf[i])
        if c >= best_c:
            best_i, best_c = i, c
    if best_i < 0:
        return None
    return result.names[int(boxes.cls[best_i])]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", choices=("s", "l"), default="s",
                    help="yolo26 variant: s=fast (default), l=accurate")
    ap.add_argument("--model-path", default=None,
                    help="explicit .pt path (overrides --model)")
    ap.add_argument("--camera", type=int, default=CAMERA_INDEX,
                    help=f"object webcam v4l index (default {CAMERA_INDEX})")
    ap.add_argument("--conf", type=float, default=CONF)
    ap.add_argument("--continuous", action="store_true",
                    help="re-detect every frame instead of detect-once-lock")
    ap.add_argument("--show", action="store_true",
                    help="show annotated preview window (q quits, r relocks)")
    args = ap.parse_args()

    model_path = resolve_model_path(args)
    if not os.path.exists(model_path):
        raise SystemExit(
            f"[obj] model not found: {model_path}\n"
            f"      copy the .pt here or set --model-path / $YOLO_MODEL_DIR."
        )

    print(f"[obj] loading {os.path.basename(model_path)} …")
    model = YOLO(model_path)

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise SystemExit(f"[obj] could not open camera index {args.camera}")

    ros_init_no_signals()
    node = rclpy.create_node("vr_object_force")
    limit_pub = node.create_publisher(Float32, TOPIC_FORCE_LIMIT, 10)
    label_pub = node.create_publisher(String, TOPIC_OBJECT_LABEL, 10)

    mode = "continuous" if args.continuous else "detect-once-lock"
    print(f"[obj] camera={args.camera} mode={mode} conf={args.conf:.2f}. "
          f"publishing {TOPIC_FORCE_LIMIT} + {TOPIC_OBJECT_LABEL}. Ctrl+C to quit.")
    print(f"[obj] known objects: "
          + ", ".join(f"{k}={v:g}" for k, v in OBJECT_FORCE_LIMITS.items())
          + f"  (default={DEFAULT_FORCE_LIMIT:g})")

    locked_label = None       # detect-once-lock: the committed object
    streak_label, streak_n = None, 0
    period = 1.0 / PUBLISH_HZ
    n = 0
    try:
        while rclpy.ok():
            t0 = time.time()
            ok, frame = cap.read()
            if not ok:
                print("[obj] failed to read frame")
                time.sleep(period)
                continue

            result = model(frame, imgsz=IMGSZ, conf=args.conf, verbose=False)[0]
            seen = top_label(result, args.conf)

            if args.continuous:
                active = seen
            else:
                # Require LOCK_STREAK consistent frames before committing, so a
                # single spurious detection can't lock the wrong threshold.
                if locked_label is None:
                    if seen is not None and seen == streak_label:
                        streak_n += 1
                    else:
                        streak_label, streak_n = seen, (1 if seen else 0)
                    if streak_n >= LOCK_STREAK:
                        locked_label = streak_label
                        print(f"[obj] locked: {locked_label} -> "
                              f"{OBJECT_FORCE_LIMITS.get(locked_label, DEFAULT_FORCE_LIMIT):g}")
                active = locked_label

            limit = OBJECT_FORCE_LIMITS.get(active, DEFAULT_FORCE_LIMIT)

            lmsg = Float32()
            lmsg.data = float(limit)
            limit_pub.publish(lmsg)
            smsg = String()
            smsg.data = active or ""
            label_pub.publish(smsg)

            if args.show:
                annotated = result.plot()
                tag = f"{active or 'none'}  limit={limit:g}  [{mode}]"
                cv2.putText(annotated, tag, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.imshow("vr_object_force", annotated)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("r"):
                    locked_label, streak_label, streak_n = None, None, 0
                    print("[obj] relock requested")

            n += 1
            if n % int(PUBLISH_HZ * 3) == 0:
                print(f"[obj] seen={seen or '-'} active={active or '-'} "
                      f"limit={limit:g}")

            dt = period - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)
    except KeyboardInterrupt:
        print("\n[obj] bye")
    finally:
        cap.release()
        if args.show:
            cv2.destroyAllWindows()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
