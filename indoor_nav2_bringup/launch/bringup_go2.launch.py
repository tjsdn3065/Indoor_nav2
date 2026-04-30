from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = Path(get_package_share_directory('indoor_nav2_bringup'))
    rtabmap_share = Path(get_package_share_directory('indoor_nav2_rtabmap'))

    default_db = str(rtabmap_share / 'databases' / 'small_house_go2.db')
    default_params = str(pkg_share / 'config' / 'nav2_params_go2.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    database_path = LaunchConfiguration('database_path')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    launch_rviz = LaunchConfiguration('rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use /clock from the simulator.',
    )
    declare_database_path = DeclareLaunchArgument(
        'database_path', default_value=default_db,
        description='RTAB-Map database file to load for localization. '
                    'Defaults to small_house_go2.db inside indoor_nav2_rtabmap.',
    )
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Nav2 parameters YAML (Go2 profile).',
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically bring lifecycle nodes to ACTIVE.',
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch a dedicated RViz alongside Nav2 '
                    '(go2.launch.py already opens one).',
    )

    rtabmap_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(rtabmap_share / 'launch' / 'rtabmap_localization.launch.py')),
        launch_arguments={
            'database_path': database_path,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(pkg_share / 'launch' / 'bringup.launch.py')),
        launch_arguments={
            'slam': 'rtabmap',
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'rviz': launch_rviz,
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_database_path,
        declare_params,
        declare_autostart,
        declare_rviz,
        rtabmap_localization,
        nav2,
    ])
