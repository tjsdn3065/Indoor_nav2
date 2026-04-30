from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg_share = Path(get_package_share_directory('indoor_nav2_bringup'))

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
        description='Full path to the map YAML to load.',
    )
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Nav2 parameters YAML (map_server + AMCL + lifecycle_manager).',
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
        description='Launch a dedicated RViz for localization (sim_bringup already opens one).',
    )
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config', default_value=default_rviz,
        description='RViz config used when rviz:=true.',
    )

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={
            'yaml_filename': map_yaml,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
        },
        convert_types=True,
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_params],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[configured_params],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_localization',
        output='log',
        arguments=['-d', rviz_config],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
        declare_map,
        declare_params,
        declare_use_sim_time,
        declare_autostart,
        declare_rviz,
        declare_rviz_config,
        map_server,
        amcl,
        lifecycle_manager,
        rviz,
    ])
