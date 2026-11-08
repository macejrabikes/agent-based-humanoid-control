#!/usr/bin/env python3
"""
Vygeneruje pre G1 (Chappie):
  urdf/humanoid.ros2_control.xacro   ros2_control + gz plugin (telo + volitelne ruky)
  urdf/humanoid.sensors.xacro        2x IMU, kamera (D435), lidar (Mid-360)
  urdf/humanoid.gazebo.xacro         trenie chodidiel
  humanoid_control/config/controllers.yaml   body_controller + hand_controller
  humanoid_gazebo/launch/gazebo.launch.py    launch s fixed_base / with_hands
a prida includy do humanoid.urdf.xacro. Vypise odporucane spawn_z.

Pouzitie:  python g1_control.py
"""
import os
import shutil
import sys

import numpy as np
from lxml import etree

SRC = os.path.expanduser('~/agent-based-humanoid-control/src')
URDF = f'{SRC}/humanoid_description/urdf'
BODY = f'{URDF}/humanoid.urdf.xacro'
HANDS = f'{URDF}/g1.hands.xacro'
CTRL_YAML = f'{SRC}/humanoid_control/config/controllers.yaml'
LAUNCH = f'{SRC}/humanoid_gazebo/launch/gazebo.launch.py'
NS = 'http://www.ros.org/wiki/xacro'


# ---------------------------------------------------------------- pomocne
def revolute_names(root):
    return [j.get('name') for j in root.iter('joint')
            if j.get('type') == 'revolute']


def hand_joint_names(hands_root):
    for el in hands_root:
        if isinstance(el.tag, str) and el.tag == f'{{{NS}}}if':
            return revolute_names(el)
    sys.exit('V g1.hands.xacro som nenasiel <xacro:if>')


def rot(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def T(origin):
    M = np.eye(4)
    if origin is not None:
        xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
        rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]
        M[:3, :3] = rot(*rpy)
        M[:3, 3] = xyz
    return M


def chain(name, parent_joint):
    M = np.eye(4)
    while name in parent_joint:
        j = parent_joint[name]
        M = T(j.find('origin')) @ M
        name = j.find('parent').get('link')
    return M


def spawn_height(body):
    """Vyska base_link nad zemou, ked chodidla (kontaktne gule) stoja na zemi."""
    links = {l.get('name'): l for l in body.findall('link')}
    pj = {j.find('child').get('link'): j for j in body.findall('joint')}
    lowest = []
    for name, link in links.items():
        if 'ankle_roll' not in name:
            continue
        B = chain(name, pj)
        for col in link.findall('collision'):
            sph = col.find('geometry/sphere')
            if sph is None:
                continue
            M = B @ T(col.find('origin'))
            lowest.append(M[2, 3] - float(sph.get('radius')))
    return -min(lowest) if lowest else None


def ensure_include(body, filename):
    last = None
    for el in body:
        if not isinstance(el.tag, str) or el.tag != f'{{{NS}}}include':
            continue
        if el.get('filename') == filename:
            return
        last = el
    inc = etree.Element(f'{{{NS}}}include', filename=filename)
    if last is None:
        body.insert(0, inc)
    else:
        inc.tail = last.tail
        body.insert(body.index(last) + 1, inc)


# ---------------------------------------------------------------- generatory
def joint_xml(name):
    return [f'    <joint name="{name}">',
            '      <command_interface name="position"/>',
            '      <state_interface name="position"/>',
            '      <state_interface name="velocity"/>',
            '    </joint>']


def write_ros2_control(body_joints, hand_joints):
    lines = ['<?xml version="1.0"?>',
             '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">',
             '  <ros2_control name="GazeboSimSystem" type="system">',
             '    <hardware>',
             '      <plugin>gz_ros2_control/GazeboSimSystem</plugin>',
             '    </hardware>']
    for n in body_joints:
        lines += joint_xml(n)
    lines.append('    <xacro:if value="$(arg with_hands)">')
    for n in hand_joints:
        lines += ['  ' + l for l in joint_xml(n)]
    lines.append('    </xacro:if>')
    lines += ['  </ros2_control>',
              '',
              '  <gazebo>',
              '    <plugin filename="gz_ros2_control-system" '
              'name="gz_ros2_control::GazeboSimROS2ControlPlugin">',
              '      <parameters>$(find humanoid_control)/config/controllers.yaml</parameters>',
              '    </plugin>',
              '  </gazebo>',
              '</robot>', '']
    with open(f'{URDF}/humanoid.ros2_control.xacro', 'w') as f:
        f.write('\n'.join(lines))


SENSORS = """<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- IMU v panve -->
  <gazebo reference="imu_in_pelvis">
    <sensor name="imu_pelvis" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <topic>imu/pelvis</topic>
      <gz_frame_id>imu_in_pelvis</gz_frame_id>
    </sensor>
  </gazebo>

  <!-- IMU v trupe -->
  <gazebo reference="imu_in_torso">
    <sensor name="imu_torso" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <topic>imu/torso</topic>
      <gz_frame_id>imu_in_torso</gz_frame_id>
    </sensor>
  </gazebo>

  <!-- RGB kamera (RealSense D435) -->
  <gazebo reference="d435_link">
    <sensor name="rgb_camera" type="camera">
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <topic>camera/image</topic>
      <gz_frame_id>d435_link</gz_frame_id>
      <camera>
        <horizontal_fov>1.2</horizontal_fov>
        <image>
          <width>424</width>
          <height>240</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10</far>
        </clip>
      </camera>
    </sensor>
  </gazebo>

  <!-- Lidar (Livox Mid-360) -->
  <gazebo reference="mid360_link">
    <sensor name="mid360" type="gpu_lidar">
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <topic>lidar</topic>
      <gz_frame_id>mid360_link</gz_frame_id>
      <lidar>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
          <vertical>
            <samples>16</samples>
            <resolution>1</resolution>
            <min_angle>-0.122</min_angle>
            <max_angle>0.907</max_angle>
          </vertical>
        </scan>
        <range>
          <min>0.1</min>
          <max>40</max>
          <resolution>0.01</resolution>
        </range>
      </lidar>
    </sensor>
  </gazebo>

</robot>
"""


def write_gazebo_friction(body):
    feet = [l.get('name') for l in body.findall('link')
            if 'ankle_roll' in l.get('name', '')]
    lines = ['<?xml version="1.0"?>',
             '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">', '']
    for name in feet:
        lines += [f'  <gazebo reference="{name}">',
                  '    <mu1>1.0</mu1>',
                  '    <mu2>1.0</mu2>',
                  '  </gazebo>', '']
    lines += ['</robot>', '']
    with open(f'{URDF}/humanoid.gazebo.xacro', 'w') as f:
        f.write('\n'.join(lines))
    return feet


def write_controllers(body_joints, hand_joints):
    def block(name, joints):
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

    lines = ['controller_manager:',
             '  ros__parameters:',
             '    update_rate: 200',
             '    use_sim_time: true',
             '    joint_state_broadcaster:',
             '      type: joint_state_broadcaster/JointStateBroadcaster',
             '    body_controller:',
             '      type: joint_trajectory_controller/JointTrajectoryController',
             '    hand_controller:',
             '      type: joint_trajectory_controller/JointTrajectoryController',
             '']
    lines += block('body_controller', body_joints)
    lines += block('hand_controller', hand_joints)
    os.makedirs(os.path.dirname(CTRL_YAML), exist_ok=True)
    with open(CTRL_YAML, 'w') as f:
        f.write('\n'.join(lines))


LAUNCH_TEMPLATE = """import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory('humanoid_description')
    gz_share = get_package_share_directory('humanoid_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(desc_share, 'urdf', 'humanoid.urdf.xacro')
    world = os.path.join(gz_share, 'worlds', 'empty.sdf')

    fixed_base = LaunchConfiguration('fixed_base')
    with_hands = LaunchConfiguration('with_hands')
    spawn_z = LaunchConfiguration('spawn_z')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' fixed_base:=', fixed_base,
                 ' with_hands:=', with_hands]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('fixed_base', default_value='false'),
        DeclareLaunchArgument('with_hands', default_value='false'),
        DeclareLaunchArgument('spawn_z', default_value='__SPAWN_Z__'),

        # aby Gazebo nasiel package://humanoid_description/meshes/...
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
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster',
                           '--controller-manager-timeout', '30'],
                output='screen'),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['body_controller',
                           '--controller-manager-timeout', '30'],
                output='screen'),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['hand_controller',
                           '--controller-manager-timeout', '30'],
                condition=IfCondition(with_hands),
                output='screen'),
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


# ---------------------------------------------------------------- main
def main():
    for p in (BODY, HANDS):
        if not os.path.exists(p):
            sys.exit(f'Nenasiel som {p}')

    backup = os.path.expanduser('~/backup_urdf')
    os.makedirs(backup, exist_ok=True)
    if os.path.exists(LAUNCH):
        shutil.copy(LAUNCH, os.path.join(backup, 'gazebo.launch.before_g1.py'))

    body_tree = etree.parse(BODY)
    body = body_tree.getroot()
    hands = etree.parse(HANDS).getroot()

    body_joints = revolute_names(body)
    hand_joints = hand_joint_names(hands)

    write_ros2_control(body_joints, hand_joints)
    with open(f'{URDF}/humanoid.sensors.xacro', 'w') as f:
        f.write(SENSORS)
    feet = write_gazebo_friction(body)
    write_controllers(body_joints, hand_joints)

    for fn in ('humanoid.ros2_control.xacro', 'humanoid.sensors.xacro',
               'humanoid.gazebo.xacro'):
        ensure_include(body, fn)
    etree.indent(body, space='  ')
    body_tree.write(BODY, xml_declaration=True, encoding='utf-8')

    z = spawn_height(body)
    if z is None:
        print('VAROVANIE: nenasiel som kontaktne gule na chodidlach, '
              'pouzivam spawn_z 0.8 (uprav rucne)')
        z = 0.8
    spawn_z = f'{z + 0.01:.2f}'

    os.makedirs(os.path.dirname(LAUNCH), exist_ok=True)
    with open(LAUNCH, 'w') as f:
        f.write(LAUNCH_TEMPLATE.replace('__SPAWN_Z__', spawn_z))

    print(f'klby tela: {len(body_joints)} (cakam 29), klby rúk: {len(hand_joints)} (cakam 14)')
    print(f'chodidla s trenim: {feet}')
    print(f'base_link nad zemou: {z:.3f} m  ->  spawn_z default = {spawn_z}')
    print('zapisane: ros2_control, sensors, gazebo xacro, controllers.yaml, gazebo.launch.py')


if __name__ == '__main__':
    main()
