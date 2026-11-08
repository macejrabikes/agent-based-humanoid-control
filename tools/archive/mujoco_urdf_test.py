#!/usr/bin/env python3
"""
Rozdelovaci test: pusti tu istu predtrenovanu politiku v MuJoCo, ale s NASIM URDF
(/tmp/ww_true.urdf = walk_mode:=true weld_upper:=true) namiesto tréningového g1_12dof.xml.

  referencny model  chodi?   nase URDF chodi?   =>  zaver
  ----------------  ------   ----------------   ---------------------------------------
  ano               ano                          nas model je OK, problem je fyzika Gazeba
  ano               nie                          problem je v nasom modeli (hmotnosti,
                                                 chodidla, limity momentov...)

Bezi bez okna (headless). Skusi tri varianty:
  ref      - povodny g1_12dof.xml (overenie, ze test funguje: ma ist ~0.4 m/s)
  urdf35   - nase URDF, limity momentov ako v URDF (clenok 35 Nm)
  urdf50   - nase URDF, limit clenkov 50 Nm ako pri tréningu

Pouzitie:
  ~/mujoco_venv/bin/python mujoco_urdf_test.py
  ~/mujoco_venv/bin/python mujoco_urdf_test.py --vx 0.5 --seconds 12 --mu 2.0
"""
import argparse
import os
import re
import sys

import mujoco
import numpy as np
import yaml
from lxml import etree

LEG = [f'{side}_{joint}_joint'
       for side in ('left', 'right')
       for joint in ('hip_pitch', 'hip_roll', 'hip_yaw',
                     'knee', 'ankle_pitch', 'ankle_roll')]


def gravity_orientation(q):
    qw, qx, qy, qz = q
    return np.array([2 * (-qz * qx + qw * qy),
                     -2 * (qz * qy + qw * qx),
                     1 - 2 * (qw * qw + qz * qz)])


def build_urdf_model(urdf_path, meshdir, mu, dt):
    root = etree.parse(urdf_path).getroot()

    # nepotrebne znacky pre MuJoCo
    for tag in ('gazebo', 'ros2_control', 'transmission', 'material'):
        for e in root.findall(tag):
            root.remove(e)

    # base_link je prazdny koren -> panva ma byt koren (aby sa dal pridat volny klb)
    links = {l.get('name'): l for l in root.findall('link')}
    for j in root.findall('joint'):
        if j.get('name') == 'base_joint':
            root.remove(j)
    if 'base_link' in links:
        root.remove(links['base_link'])

    # odstran material z visual/collision a package:// cesty
    for e in list(root.iter('material')):
        e.getparent().remove(e)
    for m in root.iter('mesh'):
        fn = m.get('filename')
        m.set('filename', re.sub(r'^package://[^/]+/(?:meshes/)?', '', fn))

    mj = etree.SubElement(root, 'mujoco')
    etree.SubElement(mj, 'compiler', meshdir=meshdir, discardvisual='true',
                     balanceinertia='true', fusestatic='true')
    xml = etree.tostring(root).decode()

    spec = mujoco.MjSpec.from_string(xml)
    spec.option.timestep = dt
    pelvis = spec.body('pelvis')
    if pelvis is None:
        sys.exit('V URDF som nenasiel telo pelvis')
    pelvis.add_freejoint()

    floor = spec.worldbody.add_geom()
    floor.name = 'floor'
    floor.type = mujoco.mjtGeom.mjGEOM_PLANE
    floor.size = [0, 0, 0.05]
    floor.friction = [mu, 0.005, 0.0001]

    # motory (moment = ctrl, orezanie robime sami)
    for n in LEG:
        a = spec.add_actuator()
        a.name = n
        a.target = n
        a.trntype = mujoco.mjtTrn.mjTRN_JOINT

    m = spec.compile()

    # ako v Gazebe (self_collide=false): koliduje iba chodidlo/telo s podlahou
    fid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'floor')
    m.geom_contype[:] = 1
    m.geom_conaffinity[:] = 0
    m.geom_contype[fid] = 0
    m.geom_conaffinity[fid] = 1
    m.geom_friction[:, 0] = mu
    # limity momentov z URDF
    effort = {}
    for j in etree.parse(urdf_path).getroot().findall('joint'):
        if j.get('name') in LEG:
            effort[j.get('name')] = float(j.find('limit').get('effort'))
    return m, effort


def run(label, m, effort, cfg, policy, vx, seconds, z0, verbose):
    d = mujoco.MjData(m)
    dt = m.opt.timestep
    decimation = cfg['control_decimation']
    kps = np.array(cfg['kps'], dtype=np.float32)
    kds = np.array(cfg['kds'], dtype=np.float32)
    default = np.array(cfg['default_angles'], dtype=np.float32)
    cmd_scale = np.array(cfg['cmd_scale'], dtype=np.float32)
    n = cfg['num_actions']

    qadr = np.array([m.joint(j).qposadr[0] for j in LEG])
    dadr = np.array([m.joint(j).dofadr[0] for j in LEG])
    # ctrl index pre kazdy klb
    ctrl_idx = np.zeros(12, dtype=int)
    for a in range(m.nu):
        jid = m.actuator_trnid[a, 0]
        for k, j in enumerate(LEG):
            if m.joint(j).id == jid:
                ctrl_idx[k] = a
    lim = np.array([effort[j] for j in LEG]) if effort else None

    d.qpos[2] = z0
    d.qpos[3] = 1.0
    d.qpos[qadr] = default
    mujoco.mj_forward(m, d)

    cmd = np.array([vx, 0.0, 0.0], dtype=np.float32)
    action = np.zeros(n, dtype=np.float32)
    target = default.copy()
    obs = np.zeros(cfg['num_obs'], dtype=np.float32)
    steps = int(seconds / dt)
    per_s = int(round(1.0 / dt))
    x_at = {}
    zmin = 9.0
    fell = None
    import torch
    for i in range(1, steps + 1):
        q = d.qpos[qadr]
        dq = d.qvel[dadr]
        tau = (target - q) * kps - dq * kds
        if lim is not None:
            tau = np.clip(tau, -lim, lim)
        d.ctrl[ctrl_idx] = tau
        mujoco.mj_step(m, d)

        if i % decimation == 0:
            q = d.qpos[qadr]
            dq = d.qvel[dadr]
            phase = (i * dt) % 0.8 / 0.8
            obs[:3] = d.qvel[3:6] * cfg['ang_vel_scale']
            obs[3:6] = gravity_orientation(d.qpos[3:7])
            obs[6:9] = cmd * cmd_scale
            obs[9:9 + n] = (q - default) * cfg['dof_pos_scale']
            obs[9 + n:9 + 2 * n] = dq * cfg['dof_vel_scale']
            obs[9 + 2 * n:9 + 3 * n] = action
            obs[9 + 3 * n:9 + 3 * n + 2] = [np.sin(2 * np.pi * phase),
                                            np.cos(2 * np.pi * phase)]
            with torch.no_grad():
                action = policy(torch.from_numpy(obs).unsqueeze(0)).numpy().squeeze()
            target = action * cfg['action_scale'] + default

        zmin = min(zmin, d.qpos[2])
        if fell is None and gravity_orientation(d.qpos[3:7])[2] > -0.4:
            fell = i * dt
        if i % per_s == 0:
            sec = i // per_s
            x_at[sec] = d.qpos[0]
            if verbose:
                g = gravity_orientation(d.qpos[3:7])
                print(f'  [{label}] t={sec:2d}s x={d.qpos[0]:+.3f} y={d.qpos[1]:+.3f} '
                      f'z={d.qpos[2]:.3f} grav_z={g[2]:+.2f}')
    s0, s1 = 2, max(x_at)
    speed = (x_at[s1] - x_at[s0]) / (s1 - s0) if s1 > s0 else 0.0
    return dict(label=label, speed=speed, x_end=x_at[s1], zmin=zmin, fell=fell)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', default=os.path.expanduser('~/refs/unitree_rl_gym'))
    ap.add_argument('--config', default='g1.yaml')
    ap.add_argument('--urdf', default='/tmp/ww_true.urdf')
    ap.add_argument('--meshdir', default=os.path.expanduser(
        '~/agent-based-humanoid-control/src/humanoid_description/meshes'))
    ap.add_argument('--vx', type=float, default=0.5)
    ap.add_argument('--seconds', type=float, default=12.0)
    ap.add_argument('--mu', type=float, default=1.0, help='trenie chodidlo-podlaha pre nase URDF')
    ap.add_argument('--z0', type=float, default=0.80)
    ap.add_argument('--only', default='', help='ref | urdf35 | urdf50')
    ap.add_argument('-v', '--verbose', action='store_true')
    args = ap.parse_args()

    import torch
    root = os.path.abspath(os.path.expanduser(args.root))
    with open(f'{root}/deploy/deploy_mujoco/configs/{args.config}') as f:
        cfg = yaml.load(f, Loader=yaml.FullLoader)
    sub = lambda s: s.replace('{LEGGED_GYM_ROOT_DIR}', root)
    policy = torch.jit.load(sub(cfg['policy_path']))
    dt = cfg['simulation_dt']

    results = []
    todo = [args.only] if args.only else ['ref', 'urdf35', 'urdf50']
    for name in todo:
        print(f'--- {name} ---')
        if name == 'ref':
            m = mujoco.MjModel.from_xml_path(sub(cfg['xml_path']))
            m.opt.timestep = dt
            eff = None
        else:
            m, eff = build_urdf_model(args.urdf, args.meshdir, args.mu, dt)
            if name == 'urdf50':
                for k in ('left_ankle_pitch_joint', 'left_ankle_roll_joint',
                          'right_ankle_pitch_joint', 'right_ankle_roll_joint'):
                    eff[k] = 50.0
            print('  hmotnost celeho modelu [kg]:', round(float(m.body_mass.sum()), 3),
                  ' limity momentov:', [int(eff[j]) for j in LEG])
        results.append(run(name, m, eff, cfg, policy, args.vx, args.seconds,
                           args.z0, args.verbose))

    print('\n=== VYSLEDOK (cmd vx = %.2f m/s, rychlost meraná od 2. s) ===' % args.vx)
    print(f"{'model':8s} {'rychlost [m/s]':>15s} {'x na konci [m]':>15s} {'min z [m]':>10s} {'spadol v [s]':>13s}")
    for r in results:
        print(f"{r['label']:8s} {r['speed']:15.2f} {r['x_end']:15.2f} {r['zmin']:10.3f} "
              f"{('-' if r['fell'] is None else '%.1f' % r['fell']):>13s}")


if __name__ == '__main__':
    main()
