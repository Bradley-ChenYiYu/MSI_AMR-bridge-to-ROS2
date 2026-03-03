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

    player_host = LaunchConfiguration('player_host')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    map_frame = LaunchConfiguration('map_frame')
    use_rviz = LaunchConfiguration('use_rviz')
    enable_cam0 = LaunchConfiguration('enable_cam0')
    enable_cam1 = LaunchConfiguration('enable_cam1')
    enable_cam2 = LaunchConfiguration('enable_cam2')

    return LaunchDescription([
        DeclareLaunchArgument(
            'player_host',
            default_value='192.168.88.8', # '192.168.88.8' '192.168.0.2'
            description='Amr IP address'
        ),
        DeclareLaunchArgument(
            'odom_frame',
            default_value='odom',
            description='odom_frame'
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='base_frame'
        ),DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='map_frame'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to start Rviz2'
        ),
        DeclareLaunchArgument(
            'enable_cam0',
            default_value='true',
            description='Whether to enable camera 0'
        ),
        DeclareLaunchArgument(
            'enable_cam1',
            default_value='true',
            description='Whether to enable camera 1'
        ),
        DeclareLaunchArgument(
            'enable_cam2',
            default_value='true',
            description='Whether to enable camera 2'
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
                'player_host': player_host,
                'odom_frame': odom_frame,
                'base_frame': base_frame,
                'map_frame': map_frame,
                'enable_lp0': True,
                'enable_lp1': True,
                'enable_cam0': False,
                'enable_cam1': False,
                'enable_cam2': False,
            }]
        ),
        Node(
            package='player_bridge',
            executable='player_rgb_bridge',
            name='player_rgb_bridge_node',
            output='screen',
            parameters=[{
                'player_host': player_host,
                'odom_frame': odom_frame,
                'base_frame': base_frame,
                'map_frame': map_frame,
                'enable_lp0': True,
                'enable_lp1': True,
                'enable_cam0': enable_cam0,
                'enable_cam1': enable_cam1,
                'enable_cam2': enable_cam2,
            }]
        ),
        Node(
            package='player_bridge',
            executable='player_battery_bridge',
            name='player_battery_bridge_node',
            output='screen',
            parameters=[{
                'player_host': player_host,
                'base_frame': base_frame,
                'publish_rate': 1.0,    # Hz
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
