from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = Path(get_package_share_directory('indoor_nav2_bringup'))
    rtabmap_share = Path(get_package_share_directory('indoor_nav2_rtabmap'))

    default_map = str(pkg_share / 'maps' / 'small_house.yaml')
    default_params = str(pkg_share / 'config' / 'nav2_params_go2_amcl.yaml')
    p2l_yaml = str(rtabmap_share / 'config' / 'pointcloud_to_laserscan.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    launch_rviz = LaunchConfiguration('rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use /clock from the simulator.',
    )
    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Static map YAML for AMCL. Defaults to small_house.yaml.',
    )
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Nav2 parameters YAML (Go2 + AMCL profile).',
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

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[p2l_yaml, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cloud_in', '/velodyne_points/points'),
            ('scan', '/scan'),
        ],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(pkg_share / 'launch' / 'bringup.launch.py')),
        launch_arguments={
            'slam': 'amcl',
            'map': map_yaml,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'rviz': launch_rviz,
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_params,
        declare_autostart,
        declare_rviz,
        pointcloud_to_laserscan,
        nav2,
    ])
