from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='tf_base_footprint_to_base_link',
            arguments=['--z', '0.010',
                       '--frame-id', 'base_footprint',
                       '--child-frame-id', 'base_link'],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='tf_base_link_to_imu_link',
            arguments=['--x', '-0.032', '--z', '0.068',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'imu_link'],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='tf_base_link_to_base_scan',
            arguments=['--x', '-0.032', '--z', '0.171',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'base_scan'],
        ),
    ])
