# MSBRDM-Project

Control and motion planning experiments using a UR10 industrial robotic manipulator  
for the **MSBRDM (Multi-Sensory Based Robot Dynamic Manipulation)** course.

This repository uses the **TUM ICS UR10 framework** with ROS Noetic on Ubuntu 20.04.

---

## Environment

- OS: Ubuntu 20.04
- ROS: Noetic
- Workspace: `ur10_ws`
- Robot: UR10
- End-effector: Lacquey gripper

---

## Build Instructions

Make sure all required `.deb` packages are installed before building  
(see course-provided installation instructions).

Build the catkin workspace:

```bash
cd ~/MSBRDM/MSBRDM-Project/ur10_ws
source /opt/ros/noetic/setup.bash
catkin build -DTUM_ICS_USE_QT5=1
```

After a successful build, source the workspace:

```bash
source devel/setup.bash
```

## Run UR10 Bringup (with Gripper)

To start the UR10 robot with the Lacquey gripper:

```bash
roslaunch tum_ics_ur10_bringup bringUR10-lacquey.launch
```


This launch file starts:

- UR10 robot model

- Lacquey gripper

- Required controllers

- RViz with a predefined configuration


## Other Available Bringup Options

UR10 without gripper:

```bash
roslaunch tum_ics_ur10_bringup bringUR10.launch
```

UR10 with force-torque sensor:

```bash
roslaunch tum_ics_ur10_bringup bringUR10-FT.launch
```

UR10 with force-torque sensor and Lacquey gripper:

```bash
roslaunch tum_ics_ur10_bringup bringUR10-FT-lacquey.launch
```

---

## Notes

- Always source both ROS and the workspace before running any launch files.

- Do not mix catkin_make and catkin build.

- It is recommended to run ROS commands as a non-root user.
