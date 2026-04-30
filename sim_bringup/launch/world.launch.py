from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def _world_paths(context, *args, **kwargs):
    pkg_share = Path(get_package_share_directory('sim_bringup'))
    world_name = LaunchConfiguration('world').perform(context)
    world_file = pkg_share / 'worlds' / world_name / f'{world_name}.world'
    world_models = pkg_share / 'worlds' / world_name / 'models'
    robots_root = pkg_share / 'robots'

    actions = [
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=str(world_models),
        ),
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=str(robots_root),
        ),
    ]

    gui = LaunchConfiguration('gui').perform(context)
    paused = LaunchConfiguration('paused').perform(context)
    verbose = LaunchConfiguration('verbose').perform(context)
    gz_args = str(world_file)
    if gui == 'false':
        gz_args += ' -s'
    if paused == 'false':
        gz_args += ' -r'
    if verbose == 'true':
        gz_args += ' -v 4'

    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(ros_gz_sim_share) / 'launch' / 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    ))

    bridge_config = pkg_share / 'config' / 'world_bridge.yaml'
    actions.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='world_bridge',
        output='screen',
        parameters=[{'config_file': str(bridge_config)}],
    ))

    return actions

def generate_launch_description():
    declare_world = DeclareLaunchArgument(
        'world', default_value='small_house',
        description='World name (directory under worlds/).',
    )
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Run Gazebo with GUI (set false for headless).',
    )
    declare_paused = DeclareLaunchArgument(
        'paused', default_value='false',
        description='Start the simulation paused.',
    )
    declare_verbose = DeclareLaunchArgument(
        'verbose', default_value='false',
        description='Enable verbose Gazebo logging.',
    )

    actions = []
    nvidia_egl_vendor = Path('/usr/share/glvnd/egl_vendor.d/10_nvidia.json')
    if nvidia_egl_vendor.is_file():
        actions.append(SetEnvironmentVariable(
            name='__EGL_VENDOR_LIBRARY_FILENAMES',
            value=str(nvidia_egl_vendor),
        ))

    actions.extend([
        declare_world,
        declare_gui,
        declare_paused,
        declare_verbose,
        OpaqueFunction(function=_world_paths),
    ])
    return LaunchDescription(actions)
