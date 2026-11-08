#!/usr/bin/env python3
"""
Etapa 1 portu chodze do Gazeba: nohy riadene momentom (PD cez pid_controller).

Prida argument walk_mode:
  walk_mode:=false (default)  nic sa nemeni, vsetkych 29 klbov cez body_controller
  walk_mode:=true             12 klbov nohy: effort + leg_pid_controller (kp, kd z unitree_rl_gym),
                              driek a ruky (17 klbov): upper_body_controller,
                              nohy maju damping 0.001 a friction 0.1 (ako v MuJoCo modeli)

Upravi: humanoid.urdf.xacro, humanoid.ros2_control.xacro,
        humanoid_control/config/controllers.yaml, humanoid_gazebo/launch/gazebo.launch.py
Zalohy: ~/backup_urdf/*.before_walk
"""
import os
import re
import shutil
import sys

from lxml import etree

SRC = os.path.expanduser('~/agent-based-humanoid-control/src')
URDF = f'{SRC}/humanoid_description/urdf'
BODY = f'{URDF}/humanoid.urdf.xacro'
HANDS = f'{URDF}/g1.hands.xacro'
ROS2C = f'{URDF}/humanoid.ros2_control.xacro'
CTRL_YAML = f'{SRC}/humanoid_control/config/controllers.yaml'
LAUNCH = f'{SRC}/humanoid_gazebo/launch/gazebo.launch.py'
NS = 'http://www.ros.org/wiki/xacro'

# poradie 12 klbov a zosilnenia presne podla unitree_rl_gym (g1.yaml, g1_12dof.xml)
LEG_ORDER = [f'{side}_{joint}_joint'
             for side in ('left', 'right')
             for joint in ('hip_pitch', 'hip_roll', 'hip_yaw',
                           'knee', 'ankle_pitch', 'ankle_roll')]
KP = [100.0, 100.0, 100.0, 150.0, 40.0, 40.0] * 2
KD = [2.0, 2.0, 2.0, 4.0, 2.0, 2.0] * 2
EFFORT = [88.0, 139.0, 88.0, 139.0, 50.0, 50.0] * 2


def X(tag, **attrs):
    return etree.Element(f'{{{NS}}}{tag}', **attrs)


def revolute_names(root):
    return [j.get('name') for j in root.iter('joint')
            if j.get('type') == 'revolute']


def hand_joint_names(hands_root):
    for el in hands_root:
        if isinstance(el.tag, str) and el.tag == f'{{{NS}}}if':
            return revolute_names(el)
    sys.exit('V g1.hands.xacro som nenasiel <xacro:if>')


# ------------------------------------------------------------ humanoid.urdf.xacro
def edit_body():
    tree = etree.parse(BODY)
    root = tree.getroot()

    if not any(a.get('name') == 'walk_mode' for a in root.findall(f'{{{NS}}}arg')):
        args = root.findall(f'{{{NS}}}arg')
        if not args:
            sys.exit('V humanoid.urdf.xacro som nenasiel ziadny <xacro:arg>')
        idx = root.index(args[-1])
        new = [
            X('arg', name='walk_mode', default='false'),
            X('property', name='walk_mode_str', value='$(arg walk_mode)'),
            X('property', name='is_walk',
              value="${walk_mode_str in ['true', 'True', True, 1]}"),
        ]
        for off, el in enumerate(new, start=1):
            root.insert(idx + off, el)

    n = 0
    for j in root.findall('joint'):
        if j.get('name') not in LEG_ORDER:
            continue
        dyn = j.find('dynamics')
        if dyn is None:
            print(f"VAROVANIE: klb {j.get('name')} nema <dynamics>")
            continue
        old_d, old_f = dyn.get('damping'), dyn.get('friction')
        if old_d.startswith('${'):
            continue                      # uz upravene
        dyn.set('damping', f'${{0.001 if is_walk else {old_d}}}')
        dyn.set('friction', f'${{0.1 if is_walk else {old_f}}}')
        n += 1

    etree.indent(root, space='  ')
    tree.write(BODY, xml_declaration=True, encoding='utf-8')
    return root, n


# ------------------------------------------------------------ ros2_control xacro
def edit_ros2_control():
    tree = etree.parse(ROS2C)
    root = tree.getroot()
    n = 0
    for j in root.iter('joint'):
        if j.get('name') not in LEG_ORDER:
            continue
        if j.find(f'{{{NS}}}if') is not None:
            continue                      # uz upravene
        ci = j.find('command_interface')
        if ci is None:
            continue
        pos = j.index(ci)
        j.remove(ci)
        xif = X('if', value='$(arg walk_mode)')
        etree.SubElement(xif, 'command_interface', name='effort')
        xun = X('unless', value='$(arg walk_mode)')
        etree.SubElement(xun, 'command_interface', name='position')
        j.insert(pos, xif)
        j.insert(pos + 1, xun)
        n += 1
    etree.indent(root, space='  ')
    tree.write(ROS2C, xml_declaration=True, encoding='utf-8')
    return n


# ------------------------------------------------------------ controllers.yaml
def jtc_block(name, joints):
    out = [f'{name}:',
           '  ros__parameters:',
           '    allow_partial_joints_goal: true',
           '    joints:']
    out += [f'      - {j}' for j in joints]
    out += ['    command_interfaces:',
            '      - position',
            '    state_interfaces:',
            '      - position',
            '      - velocity', '']
    return out


def write_controllers(body_joints, upper_joints, hand_joints):
    lines = ['controller_manager:',
             '  ros__parameters:',
             '    update_rate: 500',
             '    use_sim_time: true',
             '    joint_state_broadcaster:',
             '      type: joint_state_broadcaster/JointStateBroadcaster',
             '    body_controller:',
             '      type: joint_trajectory_controller/JointTrajectoryController',
             '    upper_body_controller:',
             '      type: joint_trajectory_controller/JointTrajectoryController',
             '    leg_pid_controller:',
             '      type: pid_controller/PidController',
             '    hand_controller:',
             '      type: joint_trajectory_controller/JointTrajectoryController',
             '']
    lines += jtc_block('body_controller', body_joints)
    lines += jtc_block('upper_body_controller', upper_joints)
    lines += jtc_block('hand_controller', hand_joints)

    lines += ['leg_pid_controller:',
              '  ros__parameters:',
              '    dof_names:']
    lines += [f'      - {n}' for n in LEG_ORDER]
    lines += ['    command_interface: effort',
              '    reference_and_state_interfaces:',
              '      - position',
              '      - velocity',
              '    gains:']
    for n, kp, kd, lim in zip(LEG_ORDER, KP, KD, EFFORT):
        lines += [f'      {n}:',
                  f'        p: {kp}',
                  '        i: 0.0',
                  f'        d: {kd}',
                  f'        u_clamp_max: {lim}',
                  f'        u_clamp_min: {-lim}']
    lines.append('')
    with open(CTRL_YAML, 'w') as f:
        f.write('\n'.join(lines))


# ------------------------------------------------------------ launch
LAUNCH_TEMPLATE = """import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def spawner(name, **kwargs):
    return Node(
        package='controller_manager',
        executable='spawner',
        arguments=[name, '--controller-manager-timeout', '30'],
        output='screen',
        **kwargs)


def generate_launch_description():
    desc_share = get_package_share_directory('humanoid_description')
    gz_share = get_package_share_directory('humanoid_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(desc_share, 'urdf', 'humanoid.urdf.xacro')
    world = os.path.join(gz_share, 'worlds', '__WORLD__')

    fixed_base = LaunchConfiguration('fixed_base')
    with_hands = LaunchConfiguration('with_hands')
    walk_mode = LaunchConfiguration('walk_mode')
    spawn_z = LaunchConfiguration('spawn_z')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' fixed_base:=', fixed_base,
                 ' with_hands:=', with_hands,
                 ' walk_mode:=', walk_mode]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('fixed_base', default_value='false'),
        DeclareLaunchArgument('with_hands', default_value='false'),
        DeclareLaunchArgument('walk_mode', default_value='false'),
        DeclareLaunchArgument('spawn_z', default_value='__SPAWN_Z__'),

        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                  os.path.dirname(desc_share)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': '-r --render-engine ogre ' + world}.items()),

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
            spawner('upper_body_controller', condition=IfCondition(walk_mode)),
            spawner('leg_pid_controller', condition=IfCondition(walk_mode)),
            spawner('hand_controller', condition=IfCondition(with_hands)),
        ]),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/imu/pelvis@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/imu/torso@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            ],
            output='screen'),
    ])
"""


def write_launch():
    spawn_z, world = '0.8', 'empty.sdf'
    if os.path.exists(LAUNCH):
        text = open(LAUNCH).read()
        m = re.search(r"'spawn_z',\s*default_value='([0-9.]+)'", text)
        if m:
            spawn_z = m.group(1)
        m = re.search(r"'worlds',\s*'([^']+\.sdf)'", text)
        if m:
            world = m.group(1)
    with open(LAUNCH, 'w') as f:
        f.write(LAUNCH_TEMPLATE.replace('__SPAWN_Z__', spawn_z)
                .replace('__WORLD__', world))
    return spawn_z, world


# ------------------------------------------------------------ main
def main():
    for p in (BODY, HANDS, ROS2C):
        if not os.path.exists(p):
            sys.exit(f'Nenasiel som {p}')

    backup = os.path.expanduser('~/backup_urdf')
    os.makedirs(backup, exist_ok=True)
    for p in (BODY, ROS2C, CTRL_YAML, LAUNCH):
        if os.path.exists(p):
            shutil.copy(p, os.path.join(backup, os.path.basename(p) + '.before_walk'))

    body_root, n_dyn = edit_body()
    body_joints = revolute_names(body_root)
    missing = [n for n in LEG_ORDER if n not in body_joints]
    if missing:
        sys.exit(f'Tieto klby z politiky v modeli nie su: {missing}')
    upper = [n for n in body_joints if n not in LEG_ORDER]

    hands_root = etree.parse(HANDS).getroot()
    hand_joints = hand_joint_names(hands_root)

    n_ros2c = edit_ros2_control()
    write_controllers(body_joints, upper, hand_joints)
    spawn_z, world = write_launch()

    print(f'klby tela: {len(body_joints)} (nohy {len(LEG_ORDER)} + driek a ruky {len(upper)}), '
          f'ruky: {len(hand_joints)}')
    print(f'dynamics upravene pri {n_dyn} klboch nohy, ros2_control upravene pri {n_ros2c}')
    print(f'launch: world={world}, spawn_z default={spawn_z}')
    print(f'zalohy: {backup}/*.before_walk')


if __name__ == '__main__':
    main()
