from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = Path(get_package_share_directory('indoor_nav2_rtabmap'))
    common_yaml = str(pkg_share / 'config' / 'rtabmap_common.yaml')
    localization_yaml = str(pkg_share / 'config' / 'rtabmap_localization.yaml')
    p2l_yaml = str(pkg_share / 'config' / 'pointcloud_to_laserscan.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    database_path = LaunchConfiguration('database_path')
    launch_viz = LaunchConfiguration('rtabmap_viz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use /clock from the simulator.',
    )
    declare_database_path = DeclareLaunchArgument(
        'database_path',
        description='Path to the RTAB-Map DB produced by rtabmap_mapping.launch.py.',
    )
    declare_viz = DeclareLaunchArgument(
        'rtabmap_viz', default_value='false',
        description='Launch the rtabmap_viz GUI (heavy; off by default).',
    )

    common_overrides = {'use_sim_time': use_sim_time}
    rgbd_remaps = [
        ('rgb/image', '/d455/image'),
        ('depth/image', '/d455/depth_image'),
        ('rgb/camera_info', '/d455/camera_info'),
    ]
    rtabmap_remaps = [
        ('scan_cloud', '/velodyne_points/points'),
        ('odom', '/odom'),
        ('imu', '/imu/data'),
    ]

    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        namespace='rtabmap',
        output='screen',
        parameters=[common_yaml, localization_yaml, common_overrides],
        remappings=rgbd_remaps,
    )

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[p2l_yaml, common_overrides],
        remappings=[
            ('cloud_in', '/velodyne_points/points'),
            ('scan', '/scan'),
        ],
    )

    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        output='screen',
        parameters=[
            common_yaml,
            localization_yaml,
            common_overrides,
            {'database_path': database_path},
        ],
        remappings=rtabmap_remaps,
    )

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        namespace='rtabmap',
        output='log',
        parameters=[common_yaml, common_overrides],
        remappings=rtabmap_remaps,
        condition=IfCondition(launch_viz),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_database_path,
        declare_viz,
        rgbd_sync,
        pointcloud_to_laserscan,
        rtabmap,
        rtabmap_viz,
    ])
