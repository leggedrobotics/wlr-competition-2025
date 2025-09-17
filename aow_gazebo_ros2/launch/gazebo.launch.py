from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare all launch arguments
    
    world_file_path = PathJoinSubstitution([
        FindPackageShare("aow_gazebo_ros2"),
        "worlds",
        LaunchConfiguration("world_file"),
    ])
    
    declared_arguments = [
        DeclareLaunchArgument(
            "world_file",
            default_value="empty.world",
            description="Path to the world file",
        ),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.7"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("paused", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("debug", default_value="false"),
        DeclareLaunchArgument("verbose", default_value="false"),
        DeclareLaunchArgument(
            "seed", 
            default_value="42", 
            description="Random seed for Gazebo simulation (0 = random, any other number = deterministic)"
        ),
        DeclareLaunchArgument("run_gui", default_value="true"),
    ]

    # Include the robot description from aow_description_ros2
    load_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("aow_description_ros2"),
                "launch",
                "load.launch.py"
            ])
        ])
    )
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": [
                IfElseSubstitution(LaunchConfiguration("paused"), if_value="", else_value="-r "),
                IfElseSubstitution(LaunchConfiguration("verbose"), if_value="-v4 ", else_value=""),
                "--seed ", LaunchConfiguration("seed"), " ",
                world_file_path,
            ],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Spawn robot model
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",  # Use the topic from load.launch.py
            "-name", "aow",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
            "-Y", LaunchConfiguration("yaw"),
        ],
        output="screen",
    )

    # ros_gz_bridge config
    ros_gz_bridge_config = PathJoinSubstitution([
        FindPackageShare("aow_gazebo_ros2"), "config", "aow_gz_bridge.yaml"
    ])

    # Use Node instead of IncludeLaunchDescription for ros_gz_bridge
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": ros_gz_bridge_config}],
        output="screen",
    )

    world_name = LaunchConfiguration("world_name", default="default")

    # Service bridge for world control (reset to spawn)
    world_control_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="world_control_service_bridge",
        arguments=[
            # ROS srv  ↔︎  Gazebo request/response types
            # (use ignition.msgs.* if you’re on older Ignition builds)
            [TextSubstitution(text="/world/"), world_name,
            TextSubstitution(text="/control@ros_gz_interfaces/srv/ControlWorld@gz.msgs.WorldControl@gz.msgs.Boolean")]
        ],
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )


    # Service bridge for set_pose service
    service_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="set_pose_service_bridge",
        arguments=["/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose"],
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # Service bridge for world create (spawn entity) service
    world_create_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="world_create_service_bridge",
        arguments=[
            "/world/default/create@ros_gz_interfaces/srv/SpawnEntity@gz.msgs.EntityFactory@gz.msgs.Boolean"
        ],
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription(
        declared_arguments
        + [
            load_launch,
            gz_sim,
            spawn_robot,
            ros_gz_bridge,
            service_bridge,
            world_control_bridge,
            world_create_bridge,
        ]
    )
