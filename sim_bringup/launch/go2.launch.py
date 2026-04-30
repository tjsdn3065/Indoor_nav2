from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def _bringup(context, *args, **kwargs):
    sim_share = Path(get_package_share_directory('sim_bringup'))
    go2_desc = Path(get_package_share_directory('unitree_go2_description'))
    go2_sim = Path(get_package_share_directory('unitree_go2_sim'))

    xacro_path = go2_desc / 'urdf' / 'unitree_go2_robot.xacro'
    ros_control_cfg = go2_sim / 'config' / 'ros_control' / 'ros_control.yaml'
    joints_cfg = go2_sim / 'config' / 'joints' / 'joints.yaml'
    links_cfg = go2_sim / 'config' / 'links' / 'links.yaml'
    gait_cfg = go2_sim / 'config' / 'gait' / 'gait.yaml'
    bridge_cfg = sim_share / 'config' / 'go2_bridge.yaml'

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')

    robot_description = {
        'robot_description': Command([
            'xacro ', str(xacro_path),
            ' robot_controllers:=', str(ros_control_cfg),
        ]),
        'use_sim_time': use_sim_time,
    }

    actions = []

    actions.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    ))

    actions.append(Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_' + context.launch_configurations['robot_name'],
        output='screen',
        arguments=[
            '-world', LaunchConfiguration('world'),
            '-topic', 'robot_description',
            '-name', robot_name,
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
    ))

    actions.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='go2_bridge',
        output='screen',
        parameters=[{'config_file': str(bridge_cfg)}],
    ))

    actions.append(Node(
        package='champ_base',
        executable='quadruped_controller_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'gazebo': False},
            {'publish_joint_states': False},
            {'publish_joint_control': True},
            {'publish_foot_contacts': True},
            {'joint_controller_topic':
                'joint_group_effort_controller/joint_trajectory'},
            {'urdf': Command(['xacro ', str(xacro_path)])},
            str(joints_cfg),
            str(links_cfg),
            str(gait_cfg),
            {'hardware_connected': False},
            {'close_loop_odom': True},
        ],
        remappings=[('/cmd_vel/smooth', '/cmd_vel')],
    ))

    actions.append(Node(
        package='champ_base',
        executable='state_estimation_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'orientation_from_imu': True},
            {'urdf': Command(['xacro ', str(xacro_path)])},
            str(joints_cfg),
            str(links_cfg),
            str(gait_cfg),
        ],
    ))

    # z=0.225 is the Go2 trunk standing height above ground (base_footprint).
    actions.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        output='log',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.225',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'base_link',
        ],
    ))

    actions.append(Node(
        package='robot_localization',
        executable='ekf_node',
        name='footprint_to_odom_ekf',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'base_link_frame': 'base_footprint'},
            {'odom_frame': 'odom'},
            {'world_frame': 'odom'},
            {'publish_tf': True},
            {'frequency': 50.0},
            {'two_d_mode': True},
            {'odom0': 'odom/raw'},
            {'odom0_config': [False, False, False,
                              False, False, False,
                              True, True, False,
                              False, False, True,
                              False, False, False]},
            {'imu0': 'imu/data'},
            {'imu0_config': [False, False, False,
                             False, False, True,
                             False, False, False,
                             False, False, True,
                             False, False, False]},
        ],
        remappings=[('odometry/filtered', 'odom')],
    ))

    actions.append(TimerAction(
        period=3.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=['--controller-manager-timeout', '120',
                       'joint_states_controller'],
            parameters=[{'use_sim_time': use_sim_time}],
        )],
    ))
    actions.append(TimerAction(
        period=5.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=['--controller-manager-timeout', '120',
                       'joint_group_effort_controller'],
            parameters=[{'use_sim_time': use_sim_time}],
        )],
    ))

    rviz_config_path = sim_share / 'config' / 'go2.rviz'
    if not rviz_config_path.is_file():
        rviz_config_path = sim_share / 'config' / 'default.rviz'
    rviz_args = ['-d', str(rviz_config_path)] if rviz_config_path.is_file() else []
    actions.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=rviz_args,
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': use_sim_time}],
    ))

    return actions

def generate_launch_description():
    sim_share = Path(get_package_share_directory('sim_bringup'))
    world_launch = sim_share / 'launch' / 'world.launch.py'

    declare_world = DeclareLaunchArgument(
        'world', default_value='small_house',
        description='World name (directory under worlds/).',
    )
    declare_robot_name = DeclareLaunchArgument(
        'robot_name', default_value='go2',
        description='Entity name inside Gazebo.',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz.',
    )
    declare_gui = DeclareLaunchArgument('gui', default_value='true')
    declare_paused = DeclareLaunchArgument('paused', default_value='false')
    declare_verbose = DeclareLaunchArgument('verbose', default_value='false')
    declare_x = DeclareLaunchArgument('x', default_value='-3.5')
    declare_y = DeclareLaunchArgument('y', default_value='-4.5')
    declare_z = DeclareLaunchArgument('z', default_value='0.4')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='1.58')

    realsense_share = Path(get_package_share_directory('realsense2_description'))
    realsense_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=str(realsense_share.parent),
    )

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(world_launch)),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'paused': LaunchConfiguration('paused'),
            'verbose': LaunchConfiguration('verbose'),
        }.items(),
    )

    return LaunchDescription([
        declare_world,
        declare_robot_name,
        declare_use_sim_time,
        declare_rviz,
        declare_gui, declare_paused, declare_verbose,
        declare_x, declare_y, declare_z, declare_yaw,
        realsense_resource_path,
        world,
        OpaqueFunction(function=_bringup),
    ])
