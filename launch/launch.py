from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, EmitEvent, LogInfo)
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import launch
import lifecycle_msgs.msg
import os
import yaml
from pathlib import Path




def generate_launch_description():
    #Path the parameters file
    parameters_file = os.path.join(
        get_package_share_directory("player_bridge"),
        'config', 
        'params.yaml')




    params_file = LaunchConfiguration('params_file')
    params_file_arg = DeclareLaunchArgument('params_file',
                                            default_value=str(
                                                parameters_file),
                                            description='name or path to the parameters file to use.')

    ouster_ns = LaunchConfiguration('ouster_ns')
    ouster_ns_arg = DeclareLaunchArgument(
        'ouster_ns', default_value='ouster')


    os_driver_name = LaunchConfiguration('os_driver_name')
    os_driver_name_arg = DeclareLaunchArgument(
        'os_driver_name', default_value='os_driver')

    os_driver = LifecycleNode(
        package='ouster_ros',
        executable='os_driver',
        name=os_driver_name,
        namespace=ouster_ns,
        parameters=[params_file],
        output='screen',
    )

    sensor_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(os_driver),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    sensor_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=os_driver, goal_state='inactive',
            entities=[
                LogInfo(msg="os_driver activating..."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(os_driver),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
            handle_once=True
        )
    )

    sensor_finalized_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=os_driver, goal_state='finalized',
            entities=[
                LogInfo(
                    msg="Failed to communicate with the sensor in a timely manner."),
                EmitEvent(event=launch.events.Shutdown(
                    reason="Couldn't communicate with sensor"))
            ],
        )
    )
    
    




    
    # Load the parameters specific to your ComposableNode
    slam_3d_configuration=''
    with open(parameters_file, 'r') as file:
        params=yaml.safe_load(file)
        slam_3d_configuration = params['slam_3d']['ros__parameters']




    """Generate launch description with multiple components."""
    slam_node = Node(name='slam_3d',
                     package='slam_3d',
                     executable='slam', #setup.py 'slam = slam_3d.slam:main'
                     output='screen',
                     parameters=[slam_3d_configuration],
                     #extra_arguments=[{'use_intra_process_comms':True}], #not support node implement by python
                     prefix=[''],
                     #prefix=['gdb -ex run bt q y --args'],
                     arguments=['--ros-args', '--log-level', 'INFO'],
                     )
    
    
    return LaunchDescription([
      slam_node,
      params_file_arg,
      ouster_ns_arg,
      os_driver_name_arg,
      os_driver,
      sensor_configure_event,
      sensor_activate_event,
      sensor_finalized_event
    ])
