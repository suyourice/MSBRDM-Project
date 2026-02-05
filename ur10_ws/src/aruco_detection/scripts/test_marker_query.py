#!/usr/bin/env python3
"""
Test script for querying ArUco marker positions.
Demonstrates how to use the GetMarkerPosition service.
"""

import rospy
from aruco_detection.srv import GetMarkerPosition


def query_marker(marker_id):
    """Query position of a specific marker ID."""
    rospy.wait_for_service('/aruco_detector_node/get_marker_position')

    try:
        get_marker = rospy.ServiceProxy('/aruco_detector_node/get_marker_position', GetMarkerPosition)
        response = get_marker(marker_id=marker_id)

        if response.success:
            pos = response.position
            rospy.loginfo(f"Marker {marker_id}: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}")
            rospy.loginfo(f"Message: {response.message}")
            return pos
        else:
            rospy.logwarn(f"Failed to get marker {marker_id}: {response.message}")
            return None

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None


def main():
    rospy.init_node('test_marker_query', anonymous=True)

    rate = rospy.Rate(1)  # 1 Hz

    # Query multiple markers in sequence
    markers_to_query = [0, 1, 2, 3, 4, 5]

    rospy.loginfo("Starting marker query test...")
    rospy.loginfo(f"Will query markers: {markers_to_query}")

    while not rospy.is_shutdown():
        rospy.loginfo("=" * 60)
        for marker_id in markers_to_query:
            query_marker(marker_id)
            rospy.sleep(0.2)  # Small delay between queries

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
