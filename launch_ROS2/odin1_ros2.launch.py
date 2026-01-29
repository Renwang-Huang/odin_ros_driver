# USAGE: ros2 launch odin_ros_driver odin1_ros2.launch.py
#        ros2 launch odin_ros_driver odin1_ros2.launch.py use_rviz:=true
#        ros2 launch odin_ros_driver odin1_ros2.launch.py use_rviz:=true rviz_config:=/path/to/your.rviz

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    package_dir = get_package_share_directory('odin_ros_driver')

    # Declare configuration parameter
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(package_dir, 'config', 'control_command.yaml'),
        description='Path to the control config YAML file'
    )

    # Whether to launch RViz2 (default: false)
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Set to true to launch RViz2'
    )

    # RViz2 config file (used when use_rviz is true)
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(package_dir, 'config', 'odin_ros2.rviz'),
        description='Path to RViz2 config file'
    )

    # Create main node
    host_sdk_node = Node(
        package='odin_ros_driver',
        executable='host_sdk_sample',
        name='host_sdk_sample',
        output='screen',
       # arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[{
            'config_file': LaunchConfiguration('config_file')
        }]
    )

    pcd2depth_config_path = os.path.join(package_dir, 'config', 'control_command.yaml')
    with open(pcd2depth_config_path, 'r') as f:
        pcd2depth_params = yaml.safe_load(f) 
    pcd2depth_calib_path = os.path.join(package_dir, 'config', 'calib.yaml')
    pcd2depth_params['calib_file_path'] = pcd2depth_calib_path 
    pcd2depth_node = Node(
        package='odin_ros_driver',
        executable='pcd2depth_ros2_node',  
        name='pcd2depth_ros2_node',
        output='screen',
        parameters=[pcd2depth_params]
    )

    # RViz2 node (only launched when use_rviz is true)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    # Create launch description
    ld = LaunchDescription()
    ld.add_action(config_file_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(host_sdk_node)
    ld.add_action(pcd2depth_node)
    ld.add_action(GroupAction(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        actions=[rviz_node]
    ))

    return ld
