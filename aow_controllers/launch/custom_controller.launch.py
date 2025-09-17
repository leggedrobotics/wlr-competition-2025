from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    aow_controllers_share = get_package_share_directory('aow_controllers')
    aow_description_share = get_package_share_directory('aow_description_ros2')
    
    template_config = os.path.join(aow_controllers_share, 'config', 'template_controller_config.yaml')
    custom_config = os.path.join(aow_controllers_share, 'config', 'custom_controller_config.yaml')
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([aow_description_share, '/launch/gazebo.launch.py'])
    )
    
    controller_node = Node(
        package='aow_controllers',
        executable='aow_controller_executor',
        name='custom_controller',
        parameters=[template_config, custom_config, {'controller_type': 'custom'}],
        output='screen'
    )
    
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),
        gazebo_launch,
        controller_node
    ])
