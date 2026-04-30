from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def _resolve_rviz(context, *args, **kwargs):
    if LaunchConfiguration('rviz').perform(context).lower() != 'true':
        return []

    pkg_share = Path(get_package_share_directory('sim_bringup'))
    config_dir = pkg_share / 'config'
    robot = LaunchConfiguration('robot').perform(context)
    override = LaunchConfiguration('rviz_config').perform(context)

    if override:
        config_path = override
    elif robot and (config_dir / f'{robot}.rviz').is_file():
        config_path = str(config_dir / f'{robot}.rviz')
    elif (config_dir / 'default.rviz').is_file():
        config_path = str(config_dir / 'default.rviz')
    else:
        config_path = ''

    rviz_args = ['-d', config_path] if config_path else []
    return [Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=rviz_args,
    )]

def generate_launch_description():
    pkg_share = Path(get_package_share_directory('sim_bringup'))
    launch_dir = pkg_share / 'launch'

    world = LaunchConfiguration('world')
    robot = LaunchConfiguration('robot')

    declare_world = DeclareLaunchArgument(
        'world', default_value='small_house',
        description='World name (directory under worlds/).',
    )
    declare_robot = DeclareLaunchArgument(
        'robot', default_value='turtlebot3_burger',
        description="Robot name (directory under robots/). Empty string spawns nothing.",
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz.',
    )
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config', default_value='',
        description=(
            'Explicit path to an RViz config file. '
            'If empty, falls back to config/<robot>.rviz, then config/default.rviz, '
            'then an empty workspace.'
        ),
    )
    declare_gui = DeclareLaunchArgument('gui', default_value='true')
    declare_paused = DeclareLaunchArgument('paused', default_value='false')
    declare_verbose = DeclareLaunchArgument('verbose', default_value='false')
    declare_x = DeclareLaunchArgument('x', default_value='-3.5')
    declare_y = DeclareLaunchArgument('y', default_value='-4.5')
    declare_z = DeclareLaunchArgument('z', default_value='0.01')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='1.58')

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'world.launch.py')),
        launch_arguments={
            'world': world,
            'gui': LaunchConfiguration('gui'),
            'paused': LaunchConfiguration('paused'),
            'verbose': LaunchConfiguration('verbose'),
        }.items(),
    )

    spawn_included = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'spawn_robot.launch.py')),
        launch_arguments={
            'robot': robot,
            'world': world,
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )
    spawn = TimerAction(
        period=5.0,
        actions=[spawn_included],
        condition=UnlessCondition(PythonExpression(["'", robot, "' == ''"])),
    )

    return LaunchDescription([
        declare_world,
        declare_robot,
        declare_rviz,
        declare_rviz_config,
        declare_gui, declare_paused, declare_verbose,
        declare_x, declare_y, declare_z, declare_yaw,
        world_launch,
        spawn,
        OpaqueFunction(function=_resolve_rviz),
    ])
