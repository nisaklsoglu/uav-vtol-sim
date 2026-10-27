import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    world_path = '/home/uav/ardupilot_gazebo/worlds/quadplane_runway.sdf'

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', world_path],
        output='screen'
    )

    sitl_vtol_1 = ExecuteProcess(
        cmd=[
            'sim_vehicle.py',
            '-v', 'ArduPlane',
            '-I0',
            '-f', 'quadplane',
            '--model', 'JSON:127.0.0.1:9002',
            '--no-rebuild'
        ],
        output='screen'
    )

    sitl_vtol_2 = ExecuteProcess(
        cmd=[
            'sim_vehicle.py',
            '-v', 'ArduPlane',
            '-I1',
            '-f', 'quadplane',
            '--model', 'JSON:127.0.0.1:9012',
            '--no-rebuild'
        ],
        output='screen'
    )

    delayed_mavros = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='mavros',
                executable='mavros_node',
                namespace='vtol_1',
                parameters=[{
                    'fcu_url': 'tcp://127.0.0.1:5762',
                    'target_system_id': 1,
                    'target_component_id': 1,
                }],
                output='screen'
            ),
            Node(
                package='mavros',
                executable='mavros_node',
                namespace='vtol_2',
                parameters=[{
                    'fcu_url': 'tcp://127.0.0.1:5773',
                    'target_system_id': 1,
                    'target_component_id': 1,
                }],
                output='screen'
            ),
        ]
    )

    return LaunchDescription([
        gz_sim,
        sitl_vtol_1,
        sitl_vtol_2,
        delayed_mavros
    ])
