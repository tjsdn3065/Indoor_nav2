from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration

def generate_launch_description():
    pkg_share = Path(get_package_share_directory('indoor_nav2_bringup'))
    launch_dir = pkg_share / 'launch'

    default_map = str(pkg_share / 'maps' / 'small_house.yaml')
    default_params = str(pkg_share / 'config' / 'nav2_params.yaml')
    default_rviz = str(pkg_share / 'rviz' / 'localization.rviz')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    launch_rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Full path to the map YAML to load (only used when slam:=amcl).',
    )
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Nav2 parameters YAML (all servers).',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use /clock from the simulator.',
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically bring lifecycle nodes to ACTIVE.',
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch a dedicated RViz alongside Nav2.',
    )
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config', default_value=default_rviz,
        description='RViz config used when rviz:=true.',
    )
    declare_slam = DeclareLaunchArgument(
        'slam', default_value='amcl',
        choices=['amcl', 'rtabmap'],
        description='Localization source. amcl: bundle map_server+AMCL '
                    '(default, TB3 workflow). rtabmap: skip — expects '
                    'rtabmap_localization.launch.py to run separately.',
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'localization.launch.py')),
        launch_arguments={
            'map': map_yaml,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'rviz': launch_rviz,
            'rviz_config': rviz_config,
        }.items(),
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('slam'), 'amcl')),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'navigation.launch.py')),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
        }.items(),
    )

    return LaunchDescription([
        declare_map,
        declare_params,
        declare_use_sim_time,
        declare_autostart,
        declare_rviz,
        declare_rviz_config,
        declare_slam,
        localization,
        navigation,
    ])
