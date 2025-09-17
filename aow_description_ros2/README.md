# AOW Description ROS2

This package contains the URDF description and ROS2 launch files for the AOW (Anymal on Wheels) quadruped robot.

## Features

- Complete URDF model with 12 actuated leg joints (3 per leg) + 4 wheel joints
- ROS2 Control integration with Gazebo
- Effort control for leg joints
- Velocity control for wheel joints
- Launch files for Gazebo simulation and RViz visualization

## Joint Layout

### Leg Joints (Effort Controlled)
- **HAA** (Hip Abduction/Adduction): `LF_HAA`, `RF_HAA`, `LH_HAA`, `RH_HAA`
- **HFE** (Hip Flexion/Extension): `LF_HFE`, `RF_HFE`, `LH_HFE`, `RH_HFE`
- **KFE** (Knee Flexion/Extension): `LF_KFE`, `RF_KFE`, `LH_KFE`, `RH_KFE`

### Wheel Joints (Velocity Controlled)
- **Wheels**: `LF_WHEEL`, `RF_WHEEL`, `LH_WHEEL`, `RH_WHEEL`

## Usage

### 1. Build the Package

```bash
cd ~/aowd_ws
colcon build --packages-select aow_description_ros2
source install/setup.bash
```

### 2. Launch in Gazebo

```bash
ros2 launch aow_description_ros2 aow_gazebo.launch.py
```

### 3. Launch in RViz (for visualization only)

```bash
ros2 launch aow_description_ros2 aow_rviz.launch.py
```

## ROS2 Control

The robot uses ROS2 Control with two controller groups:

### Leg Effort Controller
- **Type**: `effort_controllers/JointGroupEffortController`
- **Joints**: All 12 leg joints (HAA, HFE, KFE for each leg)
- **Command Interface**: effort (torque commands)
- **State Interfaces**: position, velocity

### Wheel Velocity Controller
- **Type**: `velocity_controllers/JointGroupVelocityController`
- **Joints**: All 4 wheel joints
- **Command Interface**: velocity (rad/s)
- **State Interfaces**: position, velocity

## Controller Commands

### Send Effort Commands to Legs
```bash
ros2 topic pub /leg_effort_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

### Send Velocity Commands to Wheels
```bash
ros2 topic pub /wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [1.0, 1.0, 1.0, 1.0]"
```

## Configuration Files

- `config/aow_controllers.yaml`: ROS2 Control configuration
- `config/aow.rviz`: RViz visualization configuration
- `urdf/aow.urdf`: Main robot description file

## Integration with Controller Executor

This package is designed to work with the `controller_executor` from the `aow_controllers` package. The controller executor will:

1. Subscribe to the joint states published by this package
2. Run neural network inference for combined navigation + locomotion
3. Publish joint commands to the controllers defined here

## Dependencies

- `robot_state_publisher`
- `joint_state_publisher`
- `gazebo_ros`
- `ros2_control`
- `ros2_controllers`
- `gazebo_ros2_control`

## File Structure

```
aow_description_ros2/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── aow_controllers.yaml    # ROS2 Control configuration
│   └── aow.rviz               # RViz configuration
├── env_hooks/
│   └── aow_description_ros2.dsv.in
├── launch/
│   ├── aow_gazebo.launch.py   # Gazebo simulation launch
│   └── aow_rviz.launch.py     # RViz visualization launch
├── meshes/                    # Robot mesh files
├── urdf/
│   └── aow.urdf              # Main robot description
└── worlds/                   # Gazebo world files
```

## Notes

- The mesh files in this package use the original ANYmal mesh assets
- Some mount and wheel mesh files may need to be added if visual accuracy is required
- The robot is configured for both standing locomotion and wheeled navigation
- Joint limits and dynamics parameters may need tuning for specific applications
