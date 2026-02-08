#!/usr/bin/env python3
import argparse
import math
import sys

import rospy
import tf
from sensor_msgs.msg import JointState


UR10_JOINTS = [
    "ursa_shoulder_pan_joint",
    "ursa_shoulder_lift_joint",
    "ursa_elbow_joint",
    "ursa_wrist_1_joint",
    "ursa_wrist_2_joint",
    "ursa_wrist_3_joint",
]


def rad_to_deg(v):
    return v * 180.0 / math.pi


def get_joint_positions(topic, timeout):
    msg = rospy.wait_for_message(topic, JointState, timeout=timeout)
    pos_map = dict(zip(msg.name, msg.position))
    missing = [j for j in UR10_JOINTS if j not in pos_map]
    if missing:
        rospy.logwarn("Missing joints in %s: %s", topic, ", ".join(missing))
    return [pos_map.get(j, 0.0) for j in UR10_JOINTS]


def get_transform(listener, base, ee, timeout):
    try:
        listener.waitForTransform(base, ee, rospy.Time(0), rospy.Duration(timeout))
        trans, rot = listener.lookupTransform(base, ee, rospy.Time(0))
        return trans, rot
    except Exception as exc:
        rospy.logerr("TF lookup failed (%s -> %s): %s", base, ee, exc)
        return None, None


def main():
    parser = argparse.ArgumentParser(description="Teach pose helper for peg-in-hole tuning.")
    parser.add_argument("--base", default="ursa_base_link", help="Base frame")
    parser.add_argument("--ee", default="ursa_ee_link", help="End-effector frame")
    parser.add_argument("--joint-topic", default="/ursa_joint_states", help="JointState topic for UR10")
    parser.add_argument("--timeout", type=float, default=2.0, help="Seconds to wait for data")
    parser.add_argument("--mode", choices=["home", "peg", "hole", "pose"], default="pose",
                        help="Output mode")
    parser.add_argument("--yaml", action="store_true", help="Print YAML snippet")

    args = parser.parse_args()
    rospy.init_node("teach_pose", anonymous=True)

    listener = tf.TransformListener()
    trans, rot = get_transform(listener, args.base, args.ee, args.timeout)
    if trans is None:
        return 1

    roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
    x, y, z = trans

    print("base -> ee position [m]: {:.6f} {:.6f} {:.6f}".format(x, y, z))
    print("base -> ee rpy [deg]: {:.3f} {:.3f} {:.3f}".format(
        rad_to_deg(roll), rad_to_deg(pitch), rad_to_deg(yaw)
    ))

    if args.mode == "home":
        q = get_joint_positions(args.joint_topic, args.timeout)
        q_deg = [rad_to_deg(v) for v in q]
        if args.yaml:
            print("peg_in_hole_fsm:")
            print("  q_home: [{}]".format(", ".join("{:.2f}".format(v) for v in q_deg)))
        else:
            print("q_home [deg]: {}".format([round(v, 3) for v in q_deg]))
        return 0

    if args.mode in ("peg", "hole"):
        key = "p_peg" if args.mode == "peg" else "p_hole"
        if args.yaml:
            print("peg_in_hole_fsm:")
            print("  {}: [{:.4f}, {:.4f}, {:.4f}]".format(key, x, y, z))
            if args.mode == "peg":
                print("  peg_orientation: [{:.1f}, {:.1f}, {:.1f}]".format(
                    rad_to_deg(roll), rad_to_deg(pitch), rad_to_deg(yaw)
                ))
        else:
            print("{} [m]: {:.6f} {:.6f} {:.6f}".format(key, x, y, z))
        return 0

    # mode == pose
    if args.yaml:
        print("pose:")
        print("  position: [{:.4f}, {:.4f}, {:.4f}]".format(x, y, z))
        print("  rpy_deg: [{:.1f}, {:.1f}, {:.1f}]".format(
            rad_to_deg(roll), rad_to_deg(pitch), rad_to_deg(yaw)
        ))

    return 0


if __name__ == "__main__":
    sys.exit(main())
