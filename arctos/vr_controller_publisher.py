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
from std_msgs.msg import Bool

from vr_ros_io import FRAME_ID, TOPIC_CLUTCH, TOPIC_POSE, mat_to_quat

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
    set_main = vri.getActionSetHandle("/actions/arctos")

    active = (openvr.VRActiveActionSet_t * 1)()
    active[0].ulActionSet = set_main
    active[0].ulRestrictedToDevice = openvr.k_ulInvalidInputValueHandle
    active[0].ulSecondaryActionSet = 0
    active[0].nPriority = 0

    rclpy.init()
    node = rclpy.create_node("vr_controller_publisher")
    pose_pub = node.create_publisher(PoseStamped, TOPIC_POSE, 10)
    clutch_pub = node.create_publisher(Bool, TOPIC_CLUTCH, 10)
    print(f"[pub] publishing {TOPIC_POSE} + {TOPIC_CLUTCH} "
          f"at ~{PUBLISH_HZ:.0f} Hz. Ctrl+C to quit.")

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
                w, x, y, z = mat_to_quat(R)

                pmsg = PoseStamped()
                pmsg.header.stamp = node.get_clock().now().to_msg()
                pmsg.header.frame_id = FRAME_ID
                pmsg.pose.position.x = float(p[0])
                pmsg.pose.position.y = float(p[1])
                pmsg.pose.position.z = float(p[2])
                pmsg.pose.orientation.w = float(w)
                pmsg.pose.orientation.x = float(x)
                pmsg.pose.orientation.y = float(y)
                pmsg.pose.orientation.z = float(z)
                pose_pub.publish(pmsg)

            n += 1
            if n % int(PUBLISH_HZ * 3) == 0:
                print(f"[pub] alive — clutch={'B' if clutch else '-'} "
                      f"pose={'ok' if pd.pose.bPoseIsValid else 'INVALID'}")

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
