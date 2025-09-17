# AOW Gazebo ROS2

This package provides Gazebo simulation for the AOW (All-terrain Outdoor robot with Wheels) robot.

## Package Contents

- **launch/**: Launch files for Gazebo simulation
- **config/**: Configuration files including ros_gz_bridge setup
- **worlds/**: SDF world files for different environments
- **rviz/**: RViz configuration files

## Launch Files

### Main Simulation Launch

```bash
# Launch AOW in Gazebo with controller
ros2 launch aow_gazebo_ros2 aow_gazebo.launch.py

# Launch with different world
ros2 launch aow_gazebo_ros2 aow_gazebo.launch.py world_file:=playground.world

# Launch at specific position
ros2 launch aow_gazebo_ros2 aow_gazebo.launch.py x:=1.0 y:=2.0 z:=1.0 yaw:=1.57
```

### Core Gazebo Launch (without controller)

```bash
# Launch just Gazebo with AOW robot
ros2 launch aow_gazebo_ros2 gazebo.launch.py

# Launch with specific world
ros2 launch aow_gazebo_ros2 gazebo.launch.py world_file:=playground.world
```

### RViz Visualization

```bash
# Launch RViz with AOW robot model (no simulation)
ros2 launch aow_gazebo_ros2 aow_rviz_only.launch.py

# Or include from aow_description_ros2
ros2 launch aow_gazebo_ros2 aow_rviz.launch.py
```

## Available Worlds

- **empty.world**: Basic flat ground with lighting
- **playground.world**: Environment with colored box obstacles for navigation testing

## Controller Integration

The main launch file (`aow_gazebo.launch.py`) automatically starts the `controller_executor` node from the `aow_controllers` package, which:

- Subscribes to robot sensor data (joint states, odometry, IMU)
- Runs neural network inference for combined navigation and locomotion
- Publishes joint commands to actuate the robot

## Topics

### Joint Commands (ROS to Gazebo)
Position control for leg joints:
- `/LF_HAA_position_cmd`, `/RF_HAA_position_cmd`, `/LH_HAA_position_cmd`, `/RH_HAA_position_cmd`
- `/LF_HFE_position_cmd`, `/RF_HFE_position_cmd`, `/LH_HFE_position_cmd`, `/RH_HFE_position_cmd`
- `/LF_KFE_position_cmd`, `/RF_KFE_position_cmd`, `/LH_KFE_position_cmd`, `/RH_KFE_position_cmd`

Velocity control for wheel joints:
- `/LF_WHEEL_velocity_cmd`, `/RF_WHEEL_velocity_cmd`, `/LH_WHEEL_velocity_cmd`, `/RH_WHEEL_velocity_cmd`

### Sensor Data (Gazebo to ROS)
- `/joint_states`: Joint positions and velocities
- `/odom`: Robot odometry
- `/imu`: IMU sensor data
- `/tf`: Transform tree

## Dependencies

- `ros_gz_sim`: Gazebo simulation interface
- `ros_gz_bridge`: Communication bridge between ROS2 and Gazebo
- `aow_description_ros2`: Robot URDF and meshes
- `aow_controllers`: Robot control and neural network inference

## Robot Model

The simulation uses the AOW URDF from `aow_description_ros2` which includes:
- Quadrupedal leg structure with HAA/HFE/KFE joints
- Four wheels for hybrid locomotion
- Direct Gazebo joint plugins (no ROS2 Control)
- IMU sensor for state estimation
- Proper inertial and collision properties

## Notes

- All joint naming follows AOW convention: `{LF|RF|LH|RH}_{HAA|HFE|KFE|WHEEL}`
- Uses modern Gazebo Garden with ros_gz packages
- Controller runs at 50Hz inference with 200Hz joint command publishing
- Supports both position control (legs) and velocity control (wheels)
