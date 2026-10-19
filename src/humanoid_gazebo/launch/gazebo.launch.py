import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def spawner(name, *extra, **kwargs):
    return Node(
        package='controller_manager',
        executable='spawner',
        arguments=[name, '--controller-manager-timeout', '30', *extra],
        output='screen',
        **kwargs)


def generate_launch_description():
    desc_share = get_package_share_directory('humanoid_description')
    gz_share = get_package_share_directory('humanoid_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(desc_share, 'urdf', 'humanoid.urdf.xacro')
    world = os.path.join(gz_share, 'worlds', 'arena.sdf')

    fixed_base = LaunchConfiguration('fixed_base')
    with_hands = LaunchConfiguration('with_hands')
    walk_mode = LaunchConfiguration('walk_mode')
    weld_upper = LaunchConfiguration('weld_upper')
    spawn_z = LaunchConfiguration('spawn_z')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' fixed_base:=', fixed_base,
                 ' with_hands:=', with_hands,
                 ' walk_mode:=', walk_mode,
                 ' weld_upper:=', weld_upper]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('fixed_base', default_value='false'),
        DeclareLaunchArgument('with_hands', default_value='false'),
        DeclareLaunchArgument('walk_mode', default_value='false'),
        DeclareLaunchArgument('weld_upper', default_value='false'),
        DeclareLaunchArgument('world', default_value='arena.sdf'),
        DeclareLaunchArgument('spawn_z', default_value='0.80'),

        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                  os.path.dirname(desc_share)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': ['-r --render-engine ogre ', gz_share + '/worlds/', LaunchConfiguration('world')]}.items()),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': True}],
            output='screen'),

        TimerAction(period=4.0, actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-name', 'chappie',
                           '-topic', 'robot_description',
                           '-z', spawn_z],
                output='screen'),
        ]),

        TimerAction(period=8.0, actions=[
            spawner('joint_state_broadcaster'),
            spawner('body_controller', condition=UnlessCondition(walk_mode)),
            spawner('upper_body_controller', condition=IfCondition(PythonExpression(
                ["'", walk_mode, "' == 'true' and '", weld_upper, "' != 'true'"]))),
            spawner('leg_pid_controller', condition=IfCondition(walk_mode)),
            spawner('getup_pid_controller', '--inactive', condition=IfCondition(PythonExpression(
                ["'", walk_mode, "' == 'true' and '", weld_upper, "' != 'true'"]))),
            spawner('hand_controller', condition=IfCondition(with_hands)),
        ]),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/chappie/detach@std_msgs/msg/Empty]gz.msgs.Empty',
                '/imu/pelvis@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/imu/torso@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/chappie/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/chappie/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/chappie/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            ],
            output='screen'),
    ])
