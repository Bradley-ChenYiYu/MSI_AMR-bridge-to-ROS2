import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('player_bridge')
    rviz_config_file = os.path.join(pkg_share, 'config', 'msi.rviz')

    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Whether to start Rviz2'
        ),
        Node(
            package='player_bridge',
            executable='laser_merger_node',
            name='laser_merger',
            output='screen',
            parameters=[{
                'input_topic1': '/laser0',
                'input_topic2': '/laser1',
                'frame_id': 'base_link',
            }]
        ),
        Node(
            package='player_bridge',
            executable='player_bridge',
            name='player_bridge_node',
            output='screen',
            parameters=[{
                'player_host': '192.168.0.2',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'map_frame': 'map',
                'enable_lp0': True,
                'enable_lp1': True,
                'enable_cam0': True,
                'enable_cam1': False,
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            condition=IfCondition(use_rviz),
            output='screen'
        )
    ])
