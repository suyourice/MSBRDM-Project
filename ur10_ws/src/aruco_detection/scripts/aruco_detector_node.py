#!/usr/bin/env python3
"""
ArUco Marker Detector Node for UR10 Manipulation

Subscribes to DroidCam image stream, detects ArUco markers, and provides
3D marker positions in robot base frame for manipulation targeting.

Inspired by VerticalChess board_detection_node.py
"""

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

import tf2_ros
import tf2_geometry_msgs

from aruco_detection.srv import GetMarkerPosition, GetMarkerPositionResponse


class ArucoDetectorNode:
    """Detects ArUco markers and provides positions in robot base frame."""

    def __init__(self):
        rospy.init_node('aruco_detector_node', anonymous=False)

        # Parameters
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_optical_frame')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.aruco_dict_name = rospy.get_param('~aruco_dict', 'DICT_4X4_50')
        self.marker_size = rospy.get_param('~marker_size', 0.05)  # meters
        self.use_compressed = rospy.get_param('~use_compressed', True)
        self.publish_viz = rospy.get_param('~publish_visualization', True)
        self.detect_rate = rospy.get_param('~detect_rate', 10.0)  # Hz

        # OpenCV/ArUco setup
        self.bridge = CvBridge()
        aruco_dict_id = getattr(cv2.aruco, self.aruco_dict_name)
        self.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_id)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Camera calibration
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False

        # TF
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Detection state
        self.detected_markers = {}  # marker_id -> (position, timestamp)
        self.last_frame = None
        self.last_detection_time = rospy.Time.now()

        # Subscribers
        rospy.Subscriber('camera/camera_info', CameraInfo, self.camera_info_callback)

        if self.use_compressed:
            rospy.Subscriber('camera/image/compressed', CompressedImage, self.image_callback_compressed)
        else:
            rospy.Subscriber('camera/image_raw', Image, self.image_callback)

        # Publishers
        self.marker_viz_pub = rospy.Publisher('~marker_visualization', MarkerArray, queue_size=10)
        self.marker_poses_pub = rospy.Publisher('~marker_poses', PoseStamped, queue_size=10)

        # Services
        self.get_marker_service = rospy.Service('~get_marker_position', GetMarkerPosition, self.get_marker_position_callback)

        # Detection timer
        self.detection_timer = rospy.Timer(rospy.Duration(1.0 / self.detect_rate), self.detection_timer_callback)

        rospy.loginfo(f"ArUco Detector initialized: dict={self.aruco_dict_name}, size={self.marker_size}m")
        rospy.loginfo(f"Waiting for camera calibration on 'camera/camera_info'...")

    def camera_info_callback(self, msg):
        """Store camera calibration parameters."""
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D)
            if len(self.dist_coeffs) < 5:
                self.dist_coeffs = np.zeros(5)
            self.camera_info_received = True
            rospy.loginfo(f"Camera calibration received: fx={self.camera_matrix[0,0]:.2f}, fy={self.camera_matrix[1,1]:.2f}")

    def image_callback(self, msg):
        """Callback for raw Image messages."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_frame = frame
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

    def image_callback_compressed(self, msg):
        """Callback for CompressedImage messages (DroidCam default)."""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.last_frame = frame
        except Exception as e:
            rospy.logerr(f"Compressed image decode error: {e}")

    def detection_timer_callback(self, event):
        """Periodic detection processing."""
        if self.last_frame is None or not self.camera_info_received:
            return

        self.detect_markers(self.last_frame)

    def detect_markers(self, frame):
        """Detect ArUco markers and update detection state."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is None or len(ids) == 0:
            self.detected_markers.clear()
            return

        # Estimate pose for each marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self.camera_matrix, self.dist_coeffs
        )

        current_time = rospy.Time.now()
        self.detected_markers.clear()

        for i, marker_id in enumerate(ids.flatten()):
            rvec = rvecs[i][0]
            tvec = tvecs[i][0]

            # Transform to robot base frame
            try:
                position_base = self.transform_to_base_frame(tvec, current_time)
                if position_base is not None:
                    self.detected_markers[marker_id] = (position_base, current_time)

                    # Publish pose
                    if self.marker_poses_pub.get_num_connections() > 0:
                        pose_msg = PoseStamped()
                        pose_msg.header.stamp = current_time
                        pose_msg.header.frame_id = self.base_frame
                        pose_msg.pose.position = position_base
                        self.marker_poses_pub.publish(pose_msg)
            except Exception as e:
                rospy.logwarn_throttle(5.0, f"Failed to transform marker {marker_id}: {e}")

        # Visualization
        if self.publish_viz and len(self.detected_markers) > 0:
            self.publish_visualization()

        self.last_detection_time = current_time

    def transform_to_base_frame(self, tvec, timestamp):
        """Transform marker position from camera frame to robot base frame."""
        try:
            # Create point in camera frame
            point_camera = PoseStamped()
            point_camera.header.stamp = timestamp
            point_camera.header.frame_id = self.camera_frame
            point_camera.pose.position.x = float(tvec[0])
            point_camera.pose.position.y = float(tvec[1])
            point_camera.pose.position.z = float(tvec[2])
            point_camera.pose.orientation.w = 1.0

            # Transform to base frame
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                timestamp,
                rospy.Duration(0.1)
            )

            point_base = tf2_geometry_msgs.do_transform_pose(point_camera, transform)
            return point_base.pose.position

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, f"TF transform failed: {e}")
            return None

    def get_marker_position_callback(self, req):
        """Service callback to get 3D position of a specific marker ID."""
        response = GetMarkerPositionResponse()

        marker_id = req.marker_id

        if marker_id not in self.detected_markers:
            response.success = False
            response.message = f"Marker ID {marker_id} not currently detected"
            return response

        position, timestamp = self.detected_markers[marker_id]
        age = (rospy.Time.now() - timestamp).to_sec()

        if age > 1.0:  # Stale detection
            response.success = False
            response.message = f"Marker {marker_id} detection is stale ({age:.2f}s old)"
            return response

        response.success = True
        response.position = position
        response.message = f"Marker {marker_id} at ({position.x:.3f}, {position.y:.3f}, {position.z:.3f}) in {self.base_frame}"

        return response

    def publish_visualization(self):
        """Publish RViz markers for detected ArUco markers."""
        marker_array = MarkerArray()

        for marker_id, (position, timestamp) in self.detected_markers.items():
            # Sphere marker
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = self.base_frame
            marker.ns = "aruco_markers"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = position
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.03
            marker.scale.y = 0.03
            marker.scale.z = 0.03
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
            marker.lifetime = rospy.Duration(0.5)
            marker_array.markers.append(marker)

            # Text label
            text = Marker()
            text.header = marker.header
            text.ns = "aruco_labels"
            text.id = marker_id + 1000
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = position.x
            text.pose.position.y = position.y
            text.pose.position.z = position.z + 0.05
            text.pose.orientation.w = 1.0
            text.scale.z = 0.03
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = f"ID:{marker_id}"
            text.lifetime = rospy.Duration(0.5)
            marker_array.markers.append(text)

        self.marker_viz_pub.publish(marker_array)

    def run(self):
        """Main loop."""
        rospy.spin()


if __name__ == '__main__':
    try:
        node = ArucoDetectorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
