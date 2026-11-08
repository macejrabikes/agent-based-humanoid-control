#!/usr/bin/env python3
"""
Nahradi 'armature' z MuJoCo modelu (g1_12dof.xml: armature=0.01 na kazdom klbe)
pridanim zotrvacnosti do clankov nohy: disk s osou v osi klbu
(os = A, kolme osi = A/2), platne len pri walk_mode:=true.

Pouzitie:
  python g1_armature.py                 # armature 0.01
  python g1_armature.py --armature 0.02
"""
import argparse
import os
import shutil
import sys

import numpy as np
from lxml import etree

BODY = os.path.expanduser(
    '~/agent-based-humanoid-control/src/humanoid_description/urdf/humanoid.urdf.xacro')

LEG = [f'{side}_{joint}_joint'
       for side in ('left', 'right')
       for joint in ('hip_pitch', 'hip_roll', 'hip_yaw',
                     'knee', 'ankle_pitch', 'ankle_roll')]


def rpy_to_mat(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def floats(text, default):
    return [float(v) for v in text.split()] if text else default


def is_physical(I):
    ev = np.sort(np.linalg.eigvalsh(I))
    return ev[0] > 0 and ev[0] + ev[1] >= ev[2] * (1 - 1e-9)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--armature', type=float, default=0.01)
    args = ap.parse_args()
    A = args.armature

    if not os.path.exists(BODY):
        sys.exit(f'Nenasiel som {BODY}')
    backup = os.path.expanduser('~/backup_urdf')
    os.makedirs(backup, exist_ok=True)
    shutil.copy(BODY, os.path.join(backup, 'humanoid.before_armature.xacro'))

    tree = etree.parse(BODY)
    root = tree.getroot()
    links = {l.get('name'): l for l in root.findall('link')}

    print(f"{'klb':30s} {'clanok':26s} {'mass':>6s}  stav")
    print('-' * 80)
    changed = 0
    for j in root.findall('joint'):
        name = j.get('name')
        if name not in LEG:
            continue
        child_name = j.find('child').get('link')
        link = links.get(child_name)
        inertial = link.find('inertial') if link is not None else None
        inertia = inertial.find('inertia') if inertial is not None else None
        if inertia is None:
            print(f'{name:30s} {child_name:26s} CHYBA: clanok nema inertial')
            continue
        if inertia.get('ixx').startswith('${'):
            print(f'{name:30s} {child_name:26s} uz upravene')
            continue

        g = lambda k: float(inertia.get(k, 0))
        I = np.array([[g('ixx'), g('ixy'), g('ixz')],
                      [g('ixy'), g('iyy'), g('iyz')],
                      [g('ixz'), g('iyz'), g('izz')]])

        axis = np.array(floats(j.find('axis').get('xyz'), [1.0, 0.0, 0.0]))
        axis = axis / np.linalg.norm(axis)

        origin = inertial.find('origin')
        rpy = floats(origin.get('rpy') if origin is not None else None, [0, 0, 0])
        R = rpy_to_mat(*rpy)                       # inertialny ram -> ram clanku

        dI_link = 0.5 * A * (np.eye(3) + np.outer(axis, axis))
        new = I + R.T @ dI_link @ R

        ok = is_physical(new)
        mass = float(inertial.find('mass').get('value'))
        print(f'{name:30s} {child_name:26s} {mass:6.3f}  {"OK" if ok else "NEFYZIKALNE"}')
        if not ok:
            continue

        for key, i, k in (('ixx', 0, 0), ('ixy', 0, 1), ('ixz', 0, 2),
                          ('iyy', 1, 1), ('iyz', 1, 2), ('izz', 2, 2)):
            old = inertia.get(key, '0')
            inertia.set(key, '${%s if is_walk else %s}' % (f'{new[i, k]:.8g}', old))
        changed += 1

    etree.indent(root, space='  ')
    tree.write(BODY, xml_declaration=True, encoding='utf-8')
    print(f'\nupravenych clankov: {changed} (cakam 12), armature = {A}')
    print(f'zaloha: {backup}/humanoid.before_armature.xacro')


if __name__ == '__main__':
    main()
