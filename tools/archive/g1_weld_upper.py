#!/usr/bin/env python3
"""
Prida argument weld_upper: driek a ruky (17 klbov) sa stanu pevnymi (type=fixed),
takze sdformat ich zluci s panvou do jedneho telesa, rovnako ako v tréningovom
MuJoCo modeli (g1_12dof.xml). Default je weld_upper:=false (nic sa nemeni).

Upravi: humanoid.urdf.xacro, humanoid.ros2_control.xacro, gazebo.launch.py

Pozor: po spusteni tohto skriptu uz nespustaj g1_walk_stage1.py ani g1_control.py
(hladaju klby podla type="revolute").
"""
import os
import shutil
import sys

from lxml import etree

SRC = os.path.expanduser('~/agent-based-humanoid-control/src')
URDF = f'{SRC}/humanoid_description/urdf'
BODY = f'{URDF}/humanoid.urdf.xacro'
ROS2C = f'{URDF}/humanoid.ros2_control.xacro'
LAUNCH = f'{SRC}/humanoid_gazebo/launch/gazebo.launch.py'
NS = 'http://www.ros.org/wiki/xacro'

LEG = [f'{side}_{joint}_joint'
       for side in ('left', 'right')
       for joint in ('hip_pitch', 'hip_roll', 'hip_yaw',
                     'knee', 'ankle_pitch', 'ankle_roll')]


def X(tag, **attrs):
    return etree.Element(f'{{{NS}}}{tag}', **attrs)


def edit_body():
    tree = etree.parse(BODY)
    root = tree.getroot()

    if not any(a.get('name') == 'weld_upper' for a in root.findall(f'{{{NS}}}arg')):
        anchor = None
        for p in root.findall(f'{{{NS}}}property'):
            if p.get('name') == 'is_walk':
                anchor = p
        if anchor is None:
            sys.exit('Nenasiel som property is_walk (spusti najprv g1_walk_stage1.py)')
        idx = root.index(anchor)
        new = [
            X('arg', name='weld_upper', default='false'),
            X('property', name='weld_str', value='$(arg weld_upper)'),
            X('property', name='is_weld',
              value="${weld_str in ['true', 'True', True, 1]}"),
        ]
        for off, el in enumerate(new, start=1):
            root.insert(idx + off, el)

    n = 0
    for j in root.findall('joint'):
        if j.get('type') == 'revolute' and j.get('name') not in LEG:
            j.set('type', "${'fixed' if is_weld else 'revolute'}")
            n += 1

    etree.indent(root, space='  ')
    tree.write(BODY, xml_declaration=True, encoding='utf-8')
    return n


def edit_ros2c():
    tree = etree.parse(ROS2C)
    root = tree.getroot()
    n = 0
    for j in list(root.iter('joint')):
        name = j.get('name')
        if name in LEG or '_hand_' in name:
            continue
        parent = j.getparent()
        if parent.tag == f'{{{NS}}}unless':
            continue                       # uz upravene
        unless = X('unless', value='$(arg weld_upper)')
        parent.replace(j, unless)
        unless.append(j)
        n += 1
    etree.indent(root, space='  ')
    tree.write(ROS2C, xml_declaration=True, encoding='utf-8')
    return n


def edit_launch():
    s = open(LAUNCH).read()
    if 'weld_upper' in s:
        return 'uz upravene'

    def rep(old, new):
        nonlocal s
        if old not in s:
            sys.exit(f'V launchi som nenasiel: {old[:60]}')
        s = s.replace(old, new, 1)

    rep("from launch.substitutions import Command, LaunchConfiguration",
        "from launch.substitutions import Command, LaunchConfiguration, PythonExpression")
    rep("    walk_mode = LaunchConfiguration('walk_mode')\n",
        "    walk_mode = LaunchConfiguration('walk_mode')\n"
        "    weld_upper = LaunchConfiguration('weld_upper')\n")
    rep("                 ' walk_mode:=', walk_mode]),",
        "                 ' walk_mode:=', walk_mode,\n"
        "                 ' weld_upper:=', weld_upper]),")
    rep("        DeclareLaunchArgument('walk_mode', default_value='false'),\n",
        "        DeclareLaunchArgument('walk_mode', default_value='false'),\n"
        "        DeclareLaunchArgument('weld_upper', default_value='false'),\n")
    rep("            spawner('upper_body_controller', condition=IfCondition(walk_mode)),",
        "            spawner('upper_body_controller', condition=IfCondition(PythonExpression(\n"
        "                [\"'\", walk_mode, \"' == 'true' and '\", weld_upper, \"' != 'true'\"]))),")
    open(LAUNCH, 'w').write(s)
    return 'upravene'


def main():
    for p in (BODY, ROS2C, LAUNCH):
        if not os.path.exists(p):
            sys.exit(f'Nenasiel som {p}')
    backup = os.path.expanduser('~/backup_urdf')
    os.makedirs(backup, exist_ok=True)
    for p in (BODY, ROS2C, LAUNCH):
        shutil.copy(p, os.path.join(backup, os.path.basename(p) + '.before_weld'))

    n_body = edit_body()
    n_ctrl = edit_ros2c()
    launch_state = edit_launch()
    print(f'urdf: klbov prepnutych na weld_upper: {n_body} (cakam 17)')
    print(f'ros2_control: obalenych klbov: {n_ctrl} (cakam 17)')
    print(f'launch: {launch_state}')
    print(f'zalohy: {backup}/*.before_weld')


if __name__ == '__main__':
    main()
