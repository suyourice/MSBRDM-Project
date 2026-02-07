# UR10 Peg-in-Hole Controller

A modular, extensible controller framework for precision assembly tasks using the Universal Robots UR10.

## Overview

This package implements a complete control system for the peg-in-hole assembly task, combining:
- **Joint space control** for home positioning
- **Cartesian space control (PDG)** for trajectory tracking
- **Admittance control** for compliant insertion with force feedback
- **State machine** for task sequencing
- **Gripper control** for object manipulation

## Architecture

### Modular Design

The controller is organized into specialized modules:

```
tum_ics_ur10_controller_tutorial/
├── common/          # Base classes and utilities
├── joint/           # Joint space controllers
├── cartesian/       # Operational space controllers
├── gripper/         # Gripper interface
├── sensors/         # F/T sensor interface
└── fsm/             # Finite state machine
```

### Controller Hierarchy

All controllers inherit from `ControllerBase`, which provides:
- Unified lifecycle: `init()` → `start()` → `update()` → `stop()`
- Access to robot state via LLI (Low-Level Interface)
- Integration with ROS parameter server

## Controllers

### 1. Joint PD Controller
- **Purpose**: Move to specific joint configurations (home, park)
- **Control law**: τ = -Kp(q - q_d) - Kd·q̇
- **Config**: `config/controllers/joint_pd.yaml`

### 2. Cartesian PDG Controller
- **Purpose**: Track Cartesian trajectories with position/orientation control
- **Control law**:
  ```
  q̇_r = J⁺(ẋ_d + Kp·(x_d - x))
  τ = -Kd·(q̇ - q̇_r)
  ```
- **Features**:
  - Position error: Euclidean distance
  - Orientation error: Angle-axis representation
- **Config**: `config/controllers/cartesian_pdg.yaml`

### 3. Admittance Controller
- **Purpose**: Compliant motion in response to external forces
- **Control law**: M·Δẍ + D·Δẋ = F_ext
- **Application**: XY-plane compliance during insertion
- **Config**: `config/controllers/admittance.yaml`

### 4. Hybrid Insertion Controller
- **Purpose**: Combined position/compliance control for peg insertion
- **Strategy**:
  - Z-axis: Position control (constant descent)
  - XY-plane: Admittance control (compliant search)
  - XY-plane: Circular trajectory (spiral search)
- **Config**: `config/controllers/hybrid_insertion.yaml`

### 5. Lacquey Gripper Controller
- **Purpose**: Open/close gripper for grasping
- **Interface**: ROS topics/services
- **Config**: `config/gripper.yaml`

## Peg-in-Hole State Machine

### States

1. **INIT** - Initialize controllers
2. **HOME** - Move to home position (joint control)
3. **OPEN_GRIPPER** - Open gripper
4. **MOVE_TO_PEG** - Approach peg location (Cartesian PDG)
5. **DESCEND_TO_PEG** - Descend to grasp height
6. **CLOSE_GRIPPER** - Grasp peg
7. **LIFT_PEG** - Lift peg up
8. **MOVE_ABOVE_HOLE** - Move above hole (ArUco vision)
9. **ALIGN_ORIENTATION** - Align peg with hole
10. **CIRCULAR_INSERTION** - Insert with spiral search + compliance
11. **SUCCESS** - Insertion complete
12. **RETRY** - Retry if insertion fails
13. **DONE** - Task complete

### Success Detection

Insertion success is detected when **both** conditions are met:
- **Z-depth**: Peg inserted to target depth (e.g., 25mm)
- **Force drop**: Insertion force drops below threshold (indicates full insertion)

### Failure Handling

If insertion times out:
1. Lift peg back to `MOVE_ABOVE_HOLE`
2. Retry up to N times (configurable)
3. Abort if max retries exceeded

## Configuration

All parameters are stored in YAML files under `config/`:

```yaml
# config/peg_in_hole_fsm.yaml
states:
  CIRCULAR_INSERTION:
    radius: 0.003              # 3mm circular search
    omega: 0.5                 # rad/s angular velocity
    v_insert: 0.002            # m/s descent velocity
    success_criteria:
      z_depth: 0.025           # 25mm insertion depth
      force_drop_threshold: 3.0  # 3N force reduction
```

## Building

```bash
cd ur10_ws
catkin build tum_ics_ur10_controller_tutorial
source devel/setup.bash
```

## Usage

Launch the peg-in-hole controller:

```bash
roslaunch tum_ics_ur10_controller_tutorial peg_in_hole.launch
```

## Math Utilities

### Orientation Error (Angle-Axis)

The orientation error is computed using the angle-axis representation:

```
R_error = R_desired * R_current^T
e_o = angle * axis
```

This provides a minimal 3D representation on SO(3).

### Pseudo-Inverse

Jacobian pseudo-inverse is computed using SVD with damping for singularity robustness:

```
J⁺ = V * Σ⁺ * U^T
```

where singular values are damped: σ_inv = σ / (σ² + λ²)

## Dependencies

- ROS Noetic
- tum_ics_ur_robot_lli (Low-level interface)
- tum_ics_ur_robot_msgs (Custom messages)
- Eigen3
- Qt5

## Development Status

### Phase 1: Infrastructure ✅
- [x] ControllerBase interface
- [x] Math utilities (angle-axis, pseudo-inverse)
- [x] Type definitions
- [x] Directory structure

### Phase 2: Joint Controller ✅
- [x] Joint PD controller implementation
- [x] Configuration file (joint_pd.yaml)
- [x] Test node (joint_pd_test_node)

### Phase 3: Cartesian PDG Controller ✅
- [x] CartesianPDGController implementation
- [x] Cartesian error computation (position + angle-axis orientation)
- [x] Configuration file (cartesian_pdg.yaml)
- [x] Test node (cartesian_pdg_test_node)

### Phase 4: Gripper Controller ✅
- [x] LacqueyGripperController implementation
- [x] ROS topic interface (open/close/grasp)
- [x] Configuration file (gripper.yaml)

### Phase 5: F/T Sensor Interface ✅
- [x] FTSensorInterface implementation
- [x] Subscribe to WrenchStamped topic
- [x] Low-pass filtering and bias removal
- [x] Configuration file (sensors.yaml)

### Phase 6-8: Advanced Controllers ⏳
- [ ] Admittance controller
- [ ] Hybrid insertion controller
- [ ] Peg-in-hole FSM

## Contributing

When adding new controllers:
1. Inherit from `ControllerBase`
2. Implement all lifecycle methods
3. Add configuration file to `config/`
4. Update CMakeLists.txt
5. Update this README

## License

BSD

## Authors

- Emmanuel Dean (original framework)
- You-Ri Su <you-ri.su@tum.de> (peg-in-hole implementation)
