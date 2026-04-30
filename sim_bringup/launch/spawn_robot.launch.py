from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _spawn(context, *args, **kwargs):
    pkg_share = Path(get_package_share_directory('sim_bringup'))
    robot = LaunchConfiguration('robot').perform(context)
    sdf_path = pkg_share / 'robots' / robot / 'model.sdf'
    bridge_config = pkg_share / 'config' / f'{robot}_bridge.yaml'
    static_tfs = pkg_share / 'robots' / robot / 'static_tfs.launch.py'

    actions = []

    actions.append(Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_' + robot,
        output='screen',
        arguments=[
            '-world', LaunchConfiguration('world'),
            '-file', str(sdf_path),
            '-name', robot,
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
    ))

    actions.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=robot + '_bridge',
        output='screen',
        parameters=[{'config_file': str(bridge_config)}],
    ))

    if static_tfs.is_file():
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(static_tfs)),
        ))

    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='default',
            description='Name of the Gazebo world to spawn into.',
        ),
        DeclareLaunchArgument(
            'robot', default_value='turtlebot3_burger',
            description='Robot name (directory under robots/).',
        ),
        DeclareLaunchArgument('x', default_value='-3.5'),
        DeclareLaunchArgument('y', default_value='-4.5'),
        DeclareLaunchArgument('z', default_value='0.01'),
        DeclareLaunchArgument('yaw', default_value='1.58'),
        OpaqueFunction(function=_spawn),
    ])
