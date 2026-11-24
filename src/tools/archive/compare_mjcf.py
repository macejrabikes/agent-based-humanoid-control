#!/usr/bin/env python3
"""
Porovna tréningovy MuJoCo model (g1_12dof.xml) s nasim URDF:
  1) kinematika 12 klbov nohy: os, rozsah, posun a natocenie klbu voci rodicovi, limit momentu
  2) horna cast tela zlucena do jedneho telesa: hmotnost, tazisko, zotrvacnost

Pouzitie:
  xacro humanoid.urdf.xacro walk_mode:=true weld_upper:=true > /tmp/ww_true.urdf
  python compare_mjcf.py
"""
import argparse
import math
import os
import sys

import numpy as np
from lxml import etree

LEG = [f'{side}_{joint}_joint'
       for side in ('left', 'right')
       for joint in ('hip_pitch', 'hip_roll', 'hip_yaw',
                     'knee', 'ankle_pitch', 'ankle_roll')]


def fl(text, default):
    return [float(v) for v in text.split()] if text else default


def quat_to_mat(w, x, y, z):
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rpy_to_mat(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def origin_T(origin):
    T = np.eye(4)
    if origin is not None:
        T[:3, :3] = rpy_to_mat(*fl(origin.get('rpy'), [0, 0, 0]))
        T[:3, 3] = fl(origin.get('xyz'), [0, 0, 0])
    return T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mjcf', default=os.path.expanduser(
        '~/refs/unitree_rl_gym/resources/robots/g1_description/g1_12dof.xml'))
    ap.add_argument('--urdf', default='/tmp/ww_true.urdf')
    args = ap.parse_args()
    for p in (args.mjcf, args.urdf):
        if not os.path.exists(p):
            sys.exit(f'Nenasiel som {p}')

    mj = etree.parse(args.mjcf).getroot()
    ur = etree.parse(args.urdf).getroot()

    # ------------------------------------------------ 1) kinematika nohy
    mj_j = {}
    for j in mj.iter('joint'):
        name = j.get('name')
        if name in LEG:
            body = j.getparent()
            if body.get('euler') or body.get('axisangle'):
                print(f'VAROVANIE: telo klbu {name} pouziva euler/axisangle, '
                      'porovnanie natocenia nemusi sediet')
            mj_j[name] = dict(
                pos=np.array(fl(body.get('pos'), [0, 0, 0])),
                R=quat_to_mat(*fl(body.get('quat'), [1, 0, 0, 0])),
                axis=np.array(fl(j.get('axis'), [0, 0, 1])),
                rng=fl(j.get('range'), [float('nan')] * 2),
                frc=fl(j.get('actuatorfrcrange'), [float('nan')] * 2))

    ur_j = {j.get('name'): j for j in ur.findall('joint')}

    print('=== 1) KINEMATIKA NOHY: MuJoCo (tréning) vs nase URDF ===')
    print(f"{'klb':26s} {'os':>10s} {'os URDF':>10s} {'dxyz[mm]':>9s} {'drot[deg]':>9s} "
          f"{'rozsah MuJoCo':>17s} {'rozsah URDF':>17s} {'moment M/U':>11s}")
    bad = 0
    for n in LEG:
        if n not in mj_j or n not in ur_j:
            print(f'{n:26s} CHYBA: klb sa nenasiel')
            bad += 1
            continue
        m, u = mj_j[n], ur_j[n]
        T = origin_T(u.find('origin'))
        dxyz = np.linalg.norm(T[:3, 3] - m['pos']) * 1000
        cosang = (np.trace(m['R'].T @ T[:3, :3]) - 1) / 2
        drot = math.degrees(math.acos(max(-1.0, min(1.0, cosang))))
        ax_m = m['axis'] / np.linalg.norm(m['axis'])
        ax_u = np.array(fl(u.find('axis').get('xyz'), [1, 0, 0]))
        ax_u = ax_u / np.linalg.norm(ax_u)
        lim = u.find('limit')
        lo, hi = float(lim.get('lower')), float(lim.get('upper'))
        eff = float(lim.get('effort'))
        axis_ok = np.allclose(ax_m, ax_u, atol=1e-3)
        flags = []
        if not axis_ok:
            flags.append('OS!')
        if dxyz > 2.0:
            flags.append('POSUN!')
        if drot > 1.0:
            flags.append('NATOCENIE!')
        if abs(lo - m['rng'][0]) > 0.02 or abs(hi - m['rng'][1]) > 0.02:
            flags.append('ROZSAH!')
        if abs(eff - m['frc'][1]) > 1.0:
            flags.append('MOMENT!')
        bad += bool(flags)
        print(f"{n:26s} {str(ax_m.astype(int).tolist()):>10s} {str(ax_u.astype(int).tolist()):>10s} "
              f"{dxyz:9.1f} {drot:9.2f} "
              f"{m['rng'][0]:+7.2f}..{m['rng'][1]:+6.2f}  {lo:+7.2f}..{hi:+6.2f} "
              f"{m['frc'][1]:5.0f}/{eff:<5.0f} {' '.join(flags)}")
    print(f'\nklbov s rozdielom: {bad} z {len(LEG)}')

    # ------------------------------------------------ 2) horne teleso zlucene
    links = {l.get('name'): l for l in ur.findall('link')}
    children = {}
    for j in ur.findall('joint'):
        children.setdefault(j.find('parent').get('link'), []).append(j)

    root_name = 'pelvis'
    if root_name not in links:
        sys.exit('V URDF nie je link pelvis')

    parts = []     # (nazov, hmotnost, com v ráme panvy, R inertialny->panva, I v inertialnom ráme)
    stack = [(root_name, np.eye(4))]
    while stack:
        name, T = stack.pop()
        inertial = links[name].find('inertial')
        if inertial is not None:
            m = float(inertial.find('mass').get('value'))
            o = inertial.find('origin')
            Ti = T @ origin_T(o)
            i = inertial.find('inertia')
            g = lambda k: float(i.get(k, 0))
            I = np.array([[g('ixx'), g('ixy'), g('ixz')],
                          [g('ixy'), g('iyy'), g('iyz')],
                          [g('ixz'), g('iyz'), g('izz')]])
            parts.append((name, m, Ti[:3, 3].copy(), Ti[:3, :3].copy(), I))
        for j in children.get(name, []):
            if j.get('name') in LEG:
                continue
            stack.append((j.find('child').get('link'), T @ origin_T(j.find('origin'))))

    M = sum(p[1] for p in parts)
    com = sum(p[1] * p[2] for p in parts) / M
    I_tot = np.zeros((3, 3))
    for _, m, c, R, I in parts:
        d = c - com
        I_tot += R @ I @ R.T + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))

    # MuJoCo: inertial telesa pelvis
    pel = None
    for b in mj.iter('body'):
        if b.get('name') == 'pelvis':
            pel = b
            break
    ine = pel.find('inertial')
    m_mass = float(ine.get('mass'))
    m_com = np.array(fl(ine.get('pos'), [0, 0, 0]))
    Rq = quat_to_mat(*fl(ine.get('quat'), [1, 0, 0, 0]))
    m_I = Rq @ np.diag(fl(ine.get('diaginertia'), [0, 0, 0])) @ Rq.T

    print('\n=== 2) HORNA POLOVICA TELA (pelvis + driek + ruky + hlava) zlucena do jedneho telesa ===')
    print(f"{'':22s} {'MuJoCo (trening)':>22s} {'nase URDF':>22s}")
    print(f"{'hmotnost [kg]':22s} {m_mass:22.3f} {M:22.3f}")
    print(f"{'tazisko x [cm]':22s} {m_com[0] * 100:22.1f} {com[0] * 100:22.1f}")
    print(f"{'tazisko y [cm]':22s} {m_com[1] * 100:22.1f} {com[1] * 100:22.1f}")
    print(f"{'tazisko z [cm]':22s} {m_com[2] * 100:22.1f} {com[2] * 100:22.1f}")
    for k, nm in enumerate(('Ixx', 'Iyy', 'Izz')):
        print(f"{nm + ' [kg m2]':22s} {m_I[k, k]:22.3f} {I_tot[k, k]:22.3f}")

    print('\nhmotnosti jednotlivych casti nasho horneho telesa (najtazsie):')
    for name, m, c, _, _ in sorted(parts, key=lambda p: -p[1])[:10]:
        print(f'  {name:28s} {m:7.3f} kg   com v ráme panvy = '
              f'({c[0] * 100:+5.1f}, {c[1] * 100:+5.1f}, {c[2] * 100:+5.1f}) cm')


if __name__ == '__main__':
    main()
