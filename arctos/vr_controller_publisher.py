"""
OpenVR -> ROS2 publisher. The single SteamVR client for the teleop stack.

Reads the right Index controller via teleop_actions.json (same manifest
the teleop used directly) and publishes:

  /vr/right_controller/pose  geometry_msgs/PoseStamped  (room frame)
  /vr/clutch                 std_msgs/Bool              (B held)

Pose is only published on ticks where SteamVR reports a valid pose;
clutch is published every tick. Consumers detect a dropped controller by
pose staleness (see vr_ros_io.ControllerSubscriber).

Run under SYSTEM python with ROS sourced (see run_vr_teleop.sh):
    source /opt/ros/jazzy/setup.bash
    /usr/bin/python3 vr_controller_publisher.py
"""

import os
import time

import numpy as np
import openvr
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32, Float32MultiArray

from vr_ros_io import (
    CALIB_PATH,
    FRAME_ID,
    TOPIC_CLUTCH,
    TOPIC_FINGERS,
    TOPIC_GRIP,
    TOPIC_POSE,
    TOPIC_TRIGGER,
    apply_calibration,
    load_calibration,
    mat_to_quat,
    ros_init_no_signals,
)

HERE = os.path.dirname(os.path.abspath(__file__))
MANIFEST = os.path.join(HERE, "teleop_actions.json")
PUBLISH_HZ = 90.0


def main():
    print("[pub] initializing SteamVR…")
    openvr.init(openvr.VRApplication_Other)
    vri = openvr.VRInput()
    vri.setActionManifestPath(MANIFEST)
    a_pose = vri.getActionHandle("/actions/arctos/in/hand_pose")
    a_clutch = vri.getActionHandle("/actions/arctos/in/clutch")
    a_skel = vri.getActionHandle("/actions/arctos/in/skeleton_right")
    a_trigger = vri.getActionHandle("/actions/arctos/in/trigger")
    a_grip = vri.getActionHandle("/actions/arctos/in/grip")
    set_main = vri.getActionSetHandle("/actions/arctos")

    active = (openvr.VRActiveActionSet_t * 1)()
    active[0].ulActionSet = set_main
    active[0].ulRestrictedToDevice = openvr.k_ulInvalidInputValueHandle
    active[0].ulSecondaryActionSet = 0
    active[0].nPriority = 0

    ros_init_no_signals()
    node = rclpy.create_node("vr_controller_publisher")
    pose_pub = node.create_publisher(PoseStamped, TOPIC_POSE, 10)
    clutch_pub = node.create_publisher(Bool, TOPIC_CLUTCH, 10)
    finger_pub = node.create_publisher(Float32MultiArray, TOPIC_FINGERS, 10)
    trig_pub = node.create_publisher(Float32, TOPIC_TRIGGER, 10)
    grip_pub = node.create_publisher(Float32, TOPIC_GRIP, 10)
    print(f"[pub] publishing pose/clutch + {TOPIC_FINGERS}/"
          f"{TOPIC_TRIGGER}/{TOPIC_GRIP} at ~{PUBLISH_HZ:.0f} Hz. "
          f"Ctrl+C to quit.")

    # Manual 6-DOF calibration from vr_calibration.json (set with
    # vr_calibration_gui.py). Hot-reloaded when the file's mtime
    # changes, so slider tweaks take effect live without a restart.
    def _cal_mtime():
        try:
            return os.path.getmtime(CALIB_PATH)
        except OSError:
            return 0.0

    R_cal, t_cal = load_calibration()
    cal_mtime = _cal_mtime()
    print(f"[pub] applying 6-DOF calibration from {CALIB_PATH} "
          f"(live-reloaded on change).")

    period = 1.0 / PUBLISH_HZ
    n = 0
    try:
        while rclpy.ok():
            t0 = time.time()
            vri.updateActionState(active)

            clutch = bool(
                vri.getDigitalActionData(
                    a_clutch, openvr.k_ulInvalidInputValueHandle
                ).bState
            )
            cmsg = Bool()
            cmsg.data = clutch
            clutch_pub.publish(cmsg)

            # --- Hand: capacitive finger curls + trigger + grip ------
            trig = float(vri.getAnalogActionData(
                a_trigger, openvr.k_ulInvalidInputValueHandle).x)
            grip = float(vri.getAnalogActionData(
                a_grip, openvr.k_ulInvalidInputValueHandle).x)
            try:
                skel = vri.getSkeletalSummaryData(
                    a_skel, openvr.VRSummaryType_FromAnimation)
                # flFingerCurl order = thumb,index,middle,ring,pinky
                curls = [float(np.clip(skel.flFingerCurl[i], 0.0, 1.0))
                         for i in range(5)]
            except Exception:
                # No skeletal data (controller not reporting it): fall
                # back to trigger+grip blend, same as the MuJoCo teleop.
                close = float(np.clip(trig + 0.5 * grip, 0.0, 1.0))
                curls = [close] * 5

            fmsg = Float32MultiArray()
            fmsg.data = curls
            finger_pub.publish(fmsg)
            tmsg = Float32()
            tmsg.data = trig
            trig_pub.publish(tmsg)
            gmsg = Float32()
            gmsg.data = grip
            grip_pub.publish(gmsg)

            # Hot-reload the manual calibration if the file changed
            # (the slider GUI rewrites it on every tweak).
            mt = _cal_mtime()
            if mt != cal_mtime:
                cal_mtime = mt
                R_cal, t_cal = load_calibration()
                print("[pub] calibration reloaded from file.")

            pd = vri.getPoseActionDataRelativeToNow(
                a_pose,
                openvr.TrackingUniverseStanding,
                0.0,
                openvr.k_ulInvalidInputValueHandle,
            )

            if pd.pose.bPoseIsValid:
                m = pd.pose.mDeviceToAbsoluteTracking
                p = np.array([m.m[0][3], m.m[1][3], m.m[2][3]])
                R = np.array([
                    [m.m[0][0], m.m[0][1], m.m[0][2]],
                    [m.m[1][0], m.m[1][1], m.m[1][2]],
                    [m.m[2][0], m.m[2][1], m.m[2][2]],
                ])

                # Publish the pose re-expressed in the calibrated frame
                # (all-zero calibration = unchanged room frame).
                pa, Ra = apply_calibration(p, R, R_cal, t_cal)
                w, x, y, z = mat_to_quat(Ra)

                pmsg = PoseStamped()
                pmsg.header.stamp = node.get_clock().now().to_msg()
                pmsg.header.frame_id = FRAME_ID
                pmsg.pose.position.x = float(pa[0])
                pmsg.pose.position.y = float(pa[1])
                pmsg.pose.position.z = float(pa[2])
                pmsg.pose.orientation.w = float(w)
                pmsg.pose.orientation.x = float(x)
                pmsg.pose.orientation.y = float(y)
                pmsg.pose.orientation.z = float(z)
                pose_pub.publish(pmsg)

            n += 1
            if n % int(PUBLISH_HZ * 3) == 0:
                cs = " ".join(f"{c:.2f}" for c in curls)
                print(f"[pub] alive — clutch={'B' if clutch else '-'} "
                      f"pose={'ok' if pd.pose.bPoseIsValid else 'INVALID'} "
                      f"trig={trig:.2f} grip={grip:.2f} curls=[{cs}]")

            dt = period - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)
    except KeyboardInterrupt:
        print("\n[pub] bye")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        openvr.shutdown()


if __name__ == "__main__":
    main()
