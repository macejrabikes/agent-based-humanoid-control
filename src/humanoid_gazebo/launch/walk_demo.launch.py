"""Chappie demo: Gazebo (arena + zaves) + politika chodze (C++) + kamera sleduje robota.

  ros2 launch humanoid_gazebo walk_demo.launch.py
  ros2 launch humanoid_gazebo walk_demo.launch.py weld_upper:=false      # volne ruky
  ros2 launch humanoid_gazebo walk_demo.launch.py policy_impl:=py        # Python verzia politiky
  ros2 launch humanoid_gazebo walk_demo.launch.py auto_enable:=false follow:=false

Ovladanie z klavesnice v INOM terminali:  ros2 run humanoid_teleop wasd_teleop
Pozy ruk (ak je weld_upper:=false):
  ros2 topic pub --once /skills/pose std_msgs/msg/String "{data: 'raise_right_arm'}"
"""
import os

from ament_index_python.packages import (PackageNotFoundError,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def gz_service(service, reqtype, req):
    return ExecuteProcess(
        cmd=['gz', 'service', '-s', service, '--reqtype', reqtype,
             '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req],
        output='screen')


def generate_launch_description():
    gz_share = get_package_share_directory('humanoid_gazebo')
    ws = os.path.expanduser('~/agent-based-humanoid-control')

    big = os.path.join(gz_share, 'worlds', 'arena_big_walk.sdf')
    default_world = 'arena_big_walk.sdf' if os.path.exists(big) else 'arena_walk.sdf'

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'walk_mode': 'true',
            'with_hands': LaunchConfiguration('with_hands'),
            'weld_upper': LaunchConfiguration('weld_upper'),
            'world': LaunchConfiguration('world'),
        }.items())

    # politika chodze: C++ (predvolene) alebo Python (PyTorch z virtualneho prostredia)
    policy_cpp = Node(
        package='humanoid_walk', executable='walk_policy_node', output='screen',
        parameters=[{'cmd_timeout': 0.6}],
        condition=LaunchConfigurationEquals('policy_impl', 'cpp'))
    policy_py = ExecuteProcess(
        cmd=[LaunchConfiguration('venv_python'),
             os.path.join(ws, 'tools', 'walk_policy.py')],
        additional_env={'CHAPPIE_CMD_TIMEOUT': '0.6'},
        output='screen',
        condition=LaunchConfigurationEquals('policy_impl', 'py'))

    enable = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/walk_policy/enable',
             'std_msgs/msg/Bool', '{data: true}'],
        output='screen')

    # pozy rúk (balik humanoid_skills), len ak je nainstalovany a ruky nie su zvarene
    arm_actions = []
    try:
        get_package_share_directory('humanoid_skills')
        arm_actions.append(Node(
            package='humanoid_skills', executable='pose_skills', output='screen',
            parameters=[{
                'poses_file': os.path.join(
                    get_package_share_directory('humanoid_walk'), 'config', 'arm_poses.yaml'),
                'body_topic': '/upper_body_controller/joint_trajectory',
            }],
            condition=LaunchConfigurationEquals('weld_upper', 'false')))
    except PackageNotFoundError:
        pass

    skills_actions = []
    try:
        get_package_share_directory('humanoid_interfaces')
        skills_actions.append(Node(
            package='humanoid_skills', executable='motion_skills', output='screen'))
        skills_actions.append(Node(
            package='humanoid_skills', executable='camera_snapshot', output='screen'))
    except PackageNotFoundError:
        pass

    try:
        get_package_share_directory('humanoid_agent')
        skills_actions.append(Node(
            package='humanoid_agent', executable='agent_node', output='screen'))
    except PackageNotFoundError:
        pass

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('weld_upper', default_value='false',
                              description='true = ruky a driek pevne, false = volne ruky'),
        DeclareLaunchArgument('policy_impl', default_value='cpp',
                              description='cpp (C++ uzol) alebo py (Python)'),
        DeclareLaunchArgument(
            'venv_python',
            default_value=os.path.expanduser('~/ros_torch_venv/bin/python')),
        DeclareLaunchArgument('with_hands', default_value='false',
                              description='true = robot ma prsty (Dex3)'),
        DeclareLaunchArgument('follow', default_value='true',
                              description='kamera sleduje robota'),
        DeclareLaunchArgument('auto_enable', default_value='true',
                              description='po starte automaticky zapnut politiku'),
        sim,
        TimerAction(period=12.0, actions=[policy_cpp, policy_py] + arm_actions + skills_actions),
        TimerAction(period=14.0, condition=IfCondition(LaunchConfiguration('follow')),
                    actions=[
                        gz_service('/gui/follow', 'gz.msgs.StringMsg', 'data: "chappie"')]),
        TimerAction(period=15.0, condition=IfCondition(LaunchConfiguration('follow')),
                    actions=[
                        gz_service('/gui/follow/offset', 'gz.msgs.Vector3d',
                                   'x: -2.2 y: 0.0 z: 1.2')]),
        TimerAction(period=20.0, condition=IfCondition(LaunchConfiguration('auto_enable')),
                    actions=[enable]),
    ])
