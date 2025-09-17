#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "world_file",
            default_value="empty.world",
            description="Gazebo world file to load (empty.world, playground.world, etc.)",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RViz for visualization",
        ),
        DeclareLaunchArgument(
            "joystick",
            default_value="true",
            description="Enable joystick control for velocity commands",
        ),
        DeclareLaunchArgument(
            "x",
            default_value="0.0",
            description="Initial x position of the robot",
        ),
        DeclareLaunchArgument(
            "y",
            default_value="0.0",
            description="Initial y position of the robot",
        ),
        DeclareLaunchArgument(
            "z",
            default_value="0.7",
            description="Initial z position of the robot",
        ),
        DeclareLaunchArgument(
            "yaw",
            default_value="0.0",
            description="Initial yaw orientation of the robot",
        ),
        DeclareLaunchArgument(
            "paused",
            default_value="true",
            description="Start Gazebo in paused state",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Gazebo with GUI",
        ),
    ]

    # Include Gazebo with AOW robot
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("aow_gazebo_ros2"),
                "launch",
                "gazebo.launch.py"
            ])
        ),
        launch_arguments={
            "world_file": LaunchConfiguration("world_file"),
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "z": LaunchConfiguration("z"),
            "yaw": LaunchConfiguration("yaw"),
            "paused": LaunchConfiguration("paused"),
            "run_gui": LaunchConfiguration("gui"),
            "use_sim_time": "true",
        }.items(),
    )

    # Static transform from map to odom (needed for navigation)
    static_tf_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="log",
        parameters=[{"use_sim_time": True}],
    )

    # Unified controller executor (pure pursuit mode)
    general_config_file = PathJoinSubstitution([
        FindPackageShare("aow_controllers"),
        "config",
        "controller_executor_config.yaml"
    ])
    
    specific_config_file = PathJoinSubstitution([
        FindPackageShare("aow_controllers"),
        "config",
        "pure_pursuit_controller_config.yaml"
    ])
    
    controller_executor = Node(
        package="aow_controllers",
        executable="aow_controller_executor",  # Use unified controller executor
        name="controller_executor",
        output="screen",
        parameters=[
            general_config_file,
            specific_config_file,
        ],
    )

    # Publish mesh node for b2w simulation worlds
    publish_mesh_node = Node(
        package="aow_sim_worlds",
        executable="publish_mesh_node",
        name="publish_mesh_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # RViz for visualization (optional)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d", PathJoinSubstitution([
                FindPackageShare("aow_controllers"),
                "rviz",
                "pure_pursuit.rviz"
            ])
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )


    return LaunchDescription(
        declared_arguments + [
            gazebo_launch,
            static_tf_map_to_odom,
            controller_executor,
            publish_mesh_node,
            rviz_node,
        ]
    )
