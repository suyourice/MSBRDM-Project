# ArUco Marker Detection for UR10

ROS Noetic package for detecting ArUco markers from DroidCam video stream and providing 3D positions in UR10 robot base frame.

## Features

- **Real-time ArUco marker detection** from DroidCam image stream
- **3D pose estimation** in camera frame
- **TF transformation** to robot base frame
- **ROS service interface** for querying marker positions
- **RViz visualization** of detected markers

## Dependencies

```bash
sudo apt-get install ros-noetic-cv-bridge ros-noetic-tf2-geometry-msgs
pip3 install opencv-contrib-python
```

## Build

```bash
cd ~/MSBRDM-Project/ur10_ws
source /opt/ros/noetic/setup.bash
catkin build aruco_detection
source devel/setup.bash
```

## Camera Calibration

Before using, calibrate your DroidCam:

```bash
# 1. Start DroidCam and ROS bridge
rosrun droidcam_ros droidcam_node

# 2. Run calibration (8x6 checkerboard, 24mm squares)
rosrun camera_calibration cameracalibrator.py \
  --size 8x6 --square 0.024 \
  image:=/droidcam/image \
  camera:=/droidcam

# 3. Click 'Calibrate', then 'Save' to get calibration file
# 4. Update config/camera_calibration.yaml with results
```

## Usage

### Launch Detection

```bash
roslaunch aruco_detection aruco_detection.launch
```

### Launch with Custom Parameters

```bash
roslaunch aruco_detection aruco_detection.launch \
  camera_topic:=/droidcam_obs/image/compressed \
  marker_size:=0.08 \
  aruco_dict:=DICT_6X6_250
```

### Query Marker Position (Python)

```python
import rospy
from aruco_detection.srv import GetMarkerPosition

rospy.init_node('test_marker_query')
rospy.wait_for_service('/aruco_detector_node/get_marker_position')

get_marker = rospy.ServiceProxy('/aruco_detector_node/get_marker_position', GetMarkerPosition)
response = get_marker(marker_id=5)

if response.success:
    print(f"Marker 5 position: {response.position}")
else:
    print(f"Error: {response.message}")
```

### Query Marker Position (Command Line)

```bash
rosservice call /aruco_detector_node/get_marker_position "marker_id: 5"
```

## Topics

### Subscribed
- `camera/image/compressed` (sensor_msgs/CompressedImage) - Input image from DroidCam
- `camera/camera_info` (sensor_msgs/CameraInfo) - Camera calibration

### Published
- `~marker_poses` (geometry_msgs/PoseStamped) - Detected marker poses in base frame
- `~marker_visualization` (visualization_msgs/MarkerArray) - RViz markers

## Services

- `~get_marker_position` (aruco_detection/GetMarkerPosition) - Query 3D position of specific marker ID

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `~camera_frame` | string | `camera_optical_frame` | Camera TF frame |
| `~base_frame` | string | `base_link` | Robot base TF frame |
| `~aruco_dict` | string | `DICT_4X4_50` | ArUco dictionary name |
| `~marker_size` | float | `0.05` | Physical marker size (meters) |
| `~use_compressed` | bool | `true` | Use compressed images |
| `~publish_visualization` | bool | `true` | Publish RViz markers |
| `~detect_rate` | float | `10.0` | Detection rate (Hz) |

## TF Requirements

The node requires TF transform from `camera_optical_frame` to `base_link`. Set this up using:

1. **Static TF Publisher** (if camera is fixed):
```bash
rosrun tf static_transform_publisher x y z qx qy qz qw base_link camera_optical_frame 100
```

2. **URDF/Xacro** (recommended for fixed camera mount):
```xml
<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_optical_frame"/>
  <origin xyz="0.5 0.0 0.5" rpy="0 0 0"/>
</joint>
```

## Integration with UR10 Manipulation

Example pick-and-place workflow:

```python
import rospy
from aruco_detection.srv import GetMarkerPosition
# ... UR10 control imports ...

# Get marker position
rospy.wait_for_service('/aruco_detector_node/get_marker_position')
get_marker = rospy.ServiceProxy('/aruco_detector_node/get_marker_position', GetMarkerPosition)

response = get_marker(marker_id=10)
if response.success:
    target_pos = response.position
    # Move UR10 end-effector to target_pos
    ur10_move_to(target_pos.x, target_pos.y, target_pos.z)
```

## Troubleshooting

### No markers detected
- Check marker size parameter matches physical markers
- Ensure ArUco dictionary matches printed markers
- Verify camera calibration

### TF transform errors
- Check TF tree: `rosrun tf view_frames`
- Verify camera_optical_frame exists
- Add static transform if needed

### Poor detection accuracy
- Recalibrate camera
- Increase marker size
- Improve lighting conditions
- Reduce motion blur (lower camera exposure)

## References

- Based on [VerticalChess](https://github.com/yourusername/VerticalChess) board detection
- OpenCV ArUco documentation: https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
- DroidCam: https://www.dev47apps.com/droidcam/linux/
