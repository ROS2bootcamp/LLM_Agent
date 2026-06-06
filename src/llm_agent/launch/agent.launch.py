import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('llm_agent')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value=os.path.join(share, 'config', 'agent.yaml'),
        ),
        DeclareLaunchArgument(
            'scan_waypoints_path',
            default_value=os.path.join(share, 'config', 'scan_waypoints.yaml'),
        ),
        DeclareLaunchArgument(
            'targets_path',
            default_value=os.path.join(share, 'config', 'targets.yaml'),
        ),
        DeclareLaunchArgument(
            'objects_path',
            default_value=os.path.join(share, 'config', 'objects.yaml'),
        ),
        Node(
            package='llm_agent',
            executable='agent_node',
            name='llm_agent',
            output='screen',
            parameters=[{
                'config_path':          LaunchConfiguration('config_path'),
                'scan_waypoints_path':  LaunchConfiguration('scan_waypoints_path'),
                'targets_path':         LaunchConfiguration('targets_path'),
                'objects_path':         LaunchConfiguration('objects_path'),
            }],
        ),
    ])
