from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Path to the URDF file
    description_file = PathJoinSubstitution(
        [FindPackageShare("aow_description_ros2"), "urdf", "aow.urdf"]
    )

    # Read the file content directly
    robot_description = ParameterValue(
        Command(['cat ', description_file]),
        value_type=str
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],    
    )
    
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Define the launch description
    return LaunchDescription([
        robot_state_publisher_node,
        # joint_state_publisher_node,
    ])