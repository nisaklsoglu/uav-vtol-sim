import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('vtol_bringup')
    fastdds_xml = os.path.join(pkg_share, 'config', 'fastdds_udp_loopback.xml')
    mavros_params = os.path.join(pkg_share, 'config', 'mavros_lite.yaml')
    sitl_parm = os.path.join(pkg_share, 'config', 'sitl_no_prearm.parm')
    world_path = os.path.join(pkg_share, 'worlds', 'quadplane_dual_runway.sdf')
    prepare_script = os.path.join(pkg_share, 'scripts', 'prepare_dual_vtol_models.sh')

    models_dir = '/tmp/vtol_gz_models'
    alti_parm = '/home/uav/SITL_Models/Gazebo/config/alti_transition_quad.param'
    # Prepared per-vehicle models must win over stock alti_transition_quad (port 9002).
    gz_path_parts = [
        models_dir,
        '/home/uav/SITL_Models/Gazebo/models',
        '/home/uav/SITL_Models/Gazebo/worlds',
        '/home/uav/ardupilot_gazebo/models',
        '/home/uav/ardupilot_gazebo/worlds',
    ]
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing_gz_path:
        gz_path_parts.append(existing_gz_path)
    gz_resource_path = ':'.join(gz_path_parts)

    # Force Fast DDS over local UDPv4 (no SHM) for all launched ROS 2 processes.
    env_actions = [
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', fastdds_xml),
        SetEnvironmentVariable('FASTDDS_DEFAULT_PROFILES_FILE', fastdds_xml),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
    ]

    # Mapping (must stay 1:1):
    #   vtol_1 -> -I0 -> JSON:9002 -> TCP SERIAL1 5762 -> /vtol_1
    #   vtol_2 -> -I1 -> JSON:9012 -> TCP SERIAL1 5772 -> /vtol_2
    # Note: 5773 is SERIAL2 of instance 1; MAVROS must use SERIAL1 (+2), i.e. 5772.
    prepare_models = ExecuteProcess(
        cmd=['bash', prepare_script, models_dir],
        output='screen',
    )

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', world_path],
        output='screen',
    )

    sitl_common = [
        'sim_vehicle.py',
        '-v', 'ArduPlane',
        '-f', 'quadplane',
        '-w',  # wipe EEPROM so parm overrides always apply
        '--add-param-file', alti_parm,
        '--add-param-file', sitl_parm,  # overrides: ARMING_CHECK=0, FS_GCS_ENABLE=0
        '--no-rebuild',
    ]

    sitl_vtol_1 = ExecuteProcess(
        cmd=[
            *sitl_common,
            '-I0',
            '--model', 'JSON:127.0.0.1:9002',
        ],
        output='screen',
    )

    sitl_vtol_2 = ExecuteProcess(
        cmd=[
            *sitl_common,
            '-I1',
            '--model', 'JSON:127.0.0.1:9012',
        ],
        output='screen',
    )

    mavros_common = {
        'target_system_id': 1,
        'target_component_id': 1,
        'plugin_denylist': [
            'param',
            'ftp',
            'mission',
            'actuator_control',
            'hil',
            'image_pub',
            'px4flow',
            'vibration',
            'wheel_odometry',
            'vision_speed_estimate',
            'debug_value',
            'altitude',
        ],
    }

    delayed_mavros = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='mavros',
                executable='mavros_node',
                namespace='vtol_1',
                parameters=[
                    mavros_params,
                    {
                        **mavros_common,
                        'fcu_url': 'tcp://127.0.0.1:5762',
                    },
                ],
                output='screen',
            ),
            Node(
                package='mavros',
                executable='mavros_node',
                namespace='vtol_2',
                parameters=[
                    mavros_params,
                    {
                        **mavros_common,
                        'fcu_url': 'tcp://127.0.0.1:5772',
                    },
                ],
                output='screen',
            ),
        ],
    )

    delayed_commander = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='vtol_bringup',
                executable='multi_vtol_commander',
                name='multi_vtol_commander',
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        *env_actions,
        prepare_models,
        gz_sim,
        sitl_vtol_1,
        sitl_vtol_2,
        delayed_mavros,
        delayed_commander,
    ])
