#!/usr/bin/env python3
import math
import ast

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion

try:
    import tf.transformations as tft
except Exception:  # pragma: no cover - TF import varies in ROS envs
    tft = None
try:
    import rospkg
except Exception:  # pragma: no cover
    rospkg = None


def quat_from_rpy(roll, pitch, yaw):
    if tft is None:
        return Quaternion(0.0, 0.0, 0.0, 1.0)
    q = tft.quaternion_from_euler(roll, pitch, yaw)
    return Quaternion(q[0], q[1], q[2], q[3])


def get_param_fallback(fsm_ns, key, default):
    # Prefer values from the peg_in_hole_fsm node's namespace if present
    fsm_key = "{}/{}".format(fsm_ns.rstrip("/"), key)
    if rospy.has_param(fsm_key):
        return rospy.get_param(fsm_key)
    if rospy.has_param("~" + key):
        return rospy.get_param("~" + key)
    return default


def parse_list_param(value, default):
    if isinstance(value, (list, tuple)):
        return list(value)
    if isinstance(value, str):
        try:
            parsed = ast.literal_eval(value)
            if isinstance(parsed, (list, tuple)):
                return list(parsed)
        except Exception:
            pass
    return list(default)


def quat_from_rpy_deg(roll_deg, pitch_deg, yaw_deg):
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    return quat_from_rpy(roll, pitch, yaw)


def make_marker(marker_id, frame_id, position, orientation_q, radius, length, color,
                use_mesh=False, mesh_resource="", mesh_scale=1.0):
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = rospy.Time.now()
    m.ns = "peg_hole"
    m.id = marker_id
    if use_mesh and mesh_resource:
        m.type = Marker.MESH_RESOURCE
        m.mesh_resource = mesh_resource
        m.mesh_use_embedded_materials = False
    else:
        m.type = Marker.CYLINDER
    m.action = Marker.ADD
    m.pose.position.x = position[0]
    m.pose.position.y = position[1]
    m.pose.position.z = position[2]
    m.pose.orientation = orientation_q
    if m.type == Marker.MESH_RESOURCE:
        m.scale.x = mesh_scale
        m.scale.y = mesh_scale
        m.scale.z = mesh_scale
    else:
        m.scale.x = radius * 2.0
        m.scale.y = radius * 2.0
        m.scale.z = length
    m.color.r = color[0]
    m.color.g = color[1]
    m.color.b = color[2]
    m.color.a = color[3]
    m.lifetime = rospy.Duration(0)
    return m


def make_sphere_marker(marker_id, frame_id, position, diameter, color):
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = rospy.Time.now()
    m.ns = "peg_hole_targets"
    m.id = marker_id
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.pose.position.x = position[0]
    m.pose.position.y = position[1]
    m.pose.position.z = position[2]
    m.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    m.scale.x = diameter
    m.scale.y = diameter
    m.scale.z = diameter
    m.color.r = color[0]
    m.color.g = color[1]
    m.color.b = color[2]
    m.color.a = color[3]
    m.lifetime = rospy.Duration(0)
    return m


def main():
    rospy.init_node("peg_hole_markers")

    frame_id = rospy.get_param("~frame_id", "base_link")
    rate_hz = rospy.get_param("~rate", 10.0)
    topic = rospy.get_param("~topic", "/peg_hole_markers")
    fsm_ns = rospy.get_param("~fsm_ns", "/peg_in_hole_fsm/peg_in_hole_fsm")

    # Defaults based on SDF visuals
    peg_radius = rospy.get_param("~peg_radius", 0.01)
    peg_length = rospy.get_param("~peg_length", 0.08)
    hole_radius = rospy.get_param("~hole_radius", 0.03)
    hole_length = rospy.get_param("~hole_length", 0.02)
    use_mesh = rospy.get_param("~use_mesh", False)
    mesh_scale = rospy.get_param("~mesh_scale", 0.001)
    peg_mesh_rpy_deg = parse_list_param(
        rospy.get_param("~peg_mesh_rpy_deg", [0.0, 180.0, 0.0]),
        [0.0, 180.0, 0.0],
    )
    hole_mesh_rpy_deg = parse_list_param(
        rospy.get_param("~hole_mesh_rpy_deg", [0.0, 0.0, 0.0]),
        [0.0, 0.0, 0.0],
    )
    peg_mesh_z_offset = rospy.get_param("~peg_mesh_z_offset", 0.0)
    hole_mesh_z_offset = rospy.get_param("~hole_mesh_z_offset", 0.0)
    show_target = rospy.get_param("~show_target", True)
    target_diameter = rospy.get_param("~target_diameter", 0.015)

    peg_mesh = rospy.get_param("~peg_mesh", "")
    hole_mesh = rospy.get_param("~hole_mesh", "")
    if use_mesh and rospkg is not None and (not peg_mesh or not hole_mesh):
        pkg_path = rospkg.RosPack().get_path("tum_ics_ur10_simulation_objects")
        if not peg_mesh:
            peg_mesh = "file://{}/models/peg/meshes/object_cylinder.stl".format(pkg_path)
        if not hole_mesh:
            hole_mesh = "file://{}/models/hole/meshes/base_slot_cylinder.stl".format(pkg_path)

    pub = rospy.Publisher(topic, MarkerArray, queue_size=1, latch=True)
    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        p_peg = parse_list_param(
            get_param_fallback(fsm_ns, "p_peg", [0.5, 0.2, 0.15]),
            [0.5, 0.2, 0.15],
        )
        p_hole = parse_list_param(
            get_param_fallback(fsm_ns, "p_hole", [0.5, -0.2, 0.15]),
            [0.5, -0.2, 0.15],
        )
        rpy_deg = parse_list_param(
            get_param_fallback(fsm_ns, "peg_orientation", [180.0, 0.0, 0.0]),
            [180.0, 0.0, 0.0],
        )

        roll = math.radians(rpy_deg[0])
        pitch = math.radians(rpy_deg[1])
        yaw = math.radians(rpy_deg[2])
        peg_q = quat_from_rpy(roll, pitch, yaw)
        hole_q = Quaternion(0.0, 0.0, 0.0, 1.0)

        arr = MarkerArray()
        peg_q_mesh = quat_from_rpy_deg(peg_mesh_rpy_deg[0], peg_mesh_rpy_deg[1], peg_mesh_rpy_deg[2])
        hole_q_mesh = quat_from_rpy_deg(hole_mesh_rpy_deg[0], hole_mesh_rpy_deg[1], hole_mesh_rpy_deg[2])

        peg_pos = list(p_peg)
        hole_pos = list(p_hole)
        peg_pos[2] += peg_mesh_z_offset
        hole_pos[2] += hole_mesh_z_offset

        arr.markers.append(
            make_marker(
                0, frame_id, peg_pos,
                peg_q_mesh if use_mesh else peg_q,
                peg_radius, peg_length,
                (0.85, 0.2, 0.2, 0.9),
                use_mesh=use_mesh, mesh_resource=peg_mesh, mesh_scale=mesh_scale,
            )
        )
        arr.markers.append(
            make_marker(
                1, frame_id, hole_pos,
                hole_q_mesh if use_mesh else hole_q,
                hole_radius, hole_length,
                (0.2, 0.4, 0.85, 0.6),
                use_mesh=use_mesh, mesh_resource=hole_mesh, mesh_scale=mesh_scale,
            )
        )
        if show_target:
            arr.markers.append(
                make_sphere_marker(
                    10, frame_id, p_peg, target_diameter,
                    (0.9, 0.3, 0.3, 0.9),
                )
            )
            arr.markers.append(
                make_sphere_marker(
                    11, frame_id, p_hole, target_diameter,
                    (0.3, 0.5, 0.9, 0.9),
                )
            )

        pub.publish(arr)
        rate.sleep()


if __name__ == "__main__":
    main()
