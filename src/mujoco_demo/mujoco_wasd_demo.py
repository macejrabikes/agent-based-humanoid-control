#!/usr/bin/env python3
"""
Unitree G1: predtrenovana chodza (unitree_rl_gym) v MuJoCo
 + ovladanie z klavesnice, kamera sleduje robota, arena s prekazkami.

Logika regulatora je prevzata z deploy_mujoco.py (unitree_rl_gym).

Pouzitie:
  ~/mujoco_venv/bin/python mujoco_wasd_demo.py
  ~/mujoco_venv/bin/python mujoco_wasd_demo.py --root ~/refs/unitree_rl_gym --no-arena

Ovladanie (kazde stlacenie klavesu zmeni prikaz, drzanie nefunguje):
  W / S   rychlost dopredu / dozadu
  A / D   otacanie vlavo / vpravo
  Q / E   posun do strany vlavo / vpravo
  MEDZERA zastav
  R       reset robota (ked spadne)
"""
import argparse
import os
import time

import mujoco
import mujoco.viewer
import numpy as np
import torch
import yaml

ARENA_XML = """<mujoco model="g1_arena">
  <include file="__SCENE__"/>
  <worldbody>
    <!-- obvodove steny 16 x 16 m -->
    <geom name="wall_n" type="box" pos="0 8 0.75"  size="8.2 0.1 0.75" rgba="0.85 0.85 0.88 1"/>
    <geom name="wall_s" type="box" pos="0 -8 0.75" size="8.2 0.1 0.75" rgba="0.85 0.85 0.88 1"/>
    <geom name="wall_e" type="box" pos="8 0 0.75"  size="0.1 8 0.75"   rgba="0.85 0.85 0.88 1"/>
    <geom name="wall_w" type="box" pos="-8 0 0.75" size="0.1 8 0.75"   rgba="0.85 0.85 0.88 1"/>

    <!-- prekazky -->
    <geom name="box_1"  type="box" pos="3 2 0.5"     size="0.5 0.5 0.5"  euler="0 0 17"  rgba="0.9 0.4 0.1 1"/>
    <geom name="box_2"  type="box" pos="5 -2.5 0.4"  size="0.3 1 0.4"    euler="0 0 -23" rgba="0.2 0.5 0.8 1"/>
    <geom name="cyl_1"  type="cylinder" pos="2 -3 0.6"    size="0.4 0.6"  rgba="0.3 0.7 0.3 1"/>
    <geom name="cyl_2"  type="cylinder" pos="6 3.5 0.75"  size="0.5 0.75" rgba="0.7 0.2 0.5 1"/>
    <geom name="low_wall" type="box" pos="-3 0 0.5"  size="0.1 2 0.5"   rgba="0.6 0.6 0.2 1"/>
    <geom name="pillar_1" type="box" pos="-5 -4 0.75" size="0.25 0.25 0.75" rgba="0.4 0.4 0.45 1"/>
    <geom name="pillar_2" type="box" pos="-5 4 0.75"  size="0.25 0.25 0.75" rgba="0.4 0.4 0.45 1"/>
  </worldbody>
</mujoco>
"""

# rozsahy prikazov [min, max] pre vx, vy, wz (opatrne, politika moze mat uzsi rozsah)
CMD_LIMITS = np.array([[-0.5, 1.0], [-0.5, 0.5], [-1.0, 1.0]], dtype=np.float32)
CMD_STEP = np.array([0.1, 0.1, 0.2], dtype=np.float32)


def get_gravity_orientation(q):
    qw, qx, qy, qz = q
    g = np.zeros(3)
    g[0] = 2 * (-qz * qx + qw * qy)
    g[1] = -2 * (qz * qy + qw * qx)
    g[2] = 1 - 2 * (qw * qw + qz * qz)
    return g


def pd_control(target_q, q, kp, target_dq, dq, kd):
    return (target_q - q) * kp + (target_dq - dq) * kd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', default=os.path.expanduser('~/refs/unitree_rl_gym'))
    ap.add_argument('--config', default='g1.yaml')
    ap.add_argument('--no-arena', action='store_true')
    ap.add_argument('--vx', type=float, default=0.0, help='pociatocny prikaz vx')
    ap.add_argument('--log', action='store_true', help='vypisuj merania raz za sekundu')
    ap.add_argument('--record', default='', help='zapis cielovych poloh (CSV) pre replay v Gazebe')
    args = ap.parse_args()
    root = os.path.abspath(os.path.expanduser(args.root))

    with open(f'{root}/deploy/deploy_mujoco/configs/{args.config}') as f:
        cfg = yaml.load(f, Loader=yaml.FullLoader)

    def sub(s):
        return s.replace('{LEGGED_GYM_ROOT_DIR}', root)

    policy_path = sub(cfg['policy_path'])
    xml_path = sub(cfg['xml_path'])
    sim_dt = cfg['simulation_dt']
    decimation = cfg['control_decimation']
    kps = np.array(cfg['kps'], dtype=np.float32)
    kds = np.array(cfg['kds'], dtype=np.float32)
    default_angles = np.array(cfg['default_angles'], dtype=np.float32)
    ang_vel_scale = cfg['ang_vel_scale']
    dof_pos_scale = cfg['dof_pos_scale']
    dof_vel_scale = cfg['dof_vel_scale']
    action_scale = cfg['action_scale']
    cmd_scale = np.array(cfg['cmd_scale'], dtype=np.float32)
    num_actions = cfg['num_actions']
    num_obs = cfg['num_obs']

    if not args.no_arena:
        scene_dir = os.path.dirname(xml_path)
        arena_path = os.path.join(scene_dir, 'arena_scene.xml')
        with open(arena_path, 'w') as f:
            f.write(ARENA_XML.replace('__SCENE__', os.path.basename(xml_path)))
        xml_path = arena_path

    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)
    m.opt.timestep = sim_dt
    policy = torch.jit.load(policy_path)

    cmd = np.zeros(3, dtype=np.float32)
    cmd[0] = args.vx
    stats = {'n': 0, 'err': 0.0, 'errmax': 0.0, 'dq': 0.0, 'x': 0.0,
             'ticks': 0, 'qmin': np.full(12, 9.0), 'qmax': np.full(12, -9.0),
             'tmin': np.full(12, 9.0), 'tmax': np.full(12, -9.0)}
    rec = open(args.record, 'w') if args.record else None
    reset_request = [False]

    def key_callback(keycode):
        k = chr(keycode).upper() if 0 <= keycode < 256 else ''
        if k == 'W':
            cmd[0] += CMD_STEP[0]
        elif k == 'S':
            cmd[0] -= CMD_STEP[0]
        elif k == 'A':
            cmd[2] += CMD_STEP[2]
        elif k == 'D':
            cmd[2] -= CMD_STEP[2]
        elif k == 'Q':
            cmd[1] += CMD_STEP[1]
        elif k == 'E':
            cmd[1] -= CMD_STEP[1]
        elif k == ' ':
            cmd[:] = 0.0
        elif k == 'R':
            reset_request[0] = True
            return
        else:
            return
        cmd[:] = np.clip(cmd, CMD_LIMITS[:, 0], CMD_LIMITS[:, 1])
        print(f'prikaz: vx={cmd[0]:+.1f}  vy={cmd[1]:+.1f}  wz={cmd[2]:+.1f}')

    action = np.zeros(num_actions, dtype=np.float32)
    target_dof_pos = default_angles.copy()
    obs = np.zeros(num_obs, dtype=np.float32)
    counter = 0

    print(__doc__)

    with mujoco.viewer.launch_passive(m, d, key_callback=key_callback) as viewer:
        pelvis = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'pelvis')
        with viewer.lock():
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            viewer.cam.trackbodyid = pelvis if pelvis >= 0 else 1
            viewer.cam.distance = 3.5
            viewer.cam.azimuth = 150
            viewer.cam.elevation = -20

        while viewer.is_running():
            step_start = time.time()

            if reset_request[0]:
                reset_request[0] = False
                mujoco.mj_resetData(m, d)
                cmd[:] = 0.0
                action[:] = 0.0
                target_dof_pos = default_angles.copy()
                counter = 0
                print('reset')

            tau = pd_control(target_dof_pos, d.qpos[7:], kps,
                             np.zeros_like(kds), d.qvel[6:], kds)
            d.ctrl[:] = tau
            mujoco.mj_step(m, d)
            counter += 1

            if counter % decimation == 0:
                qj = (d.qpos[7:] - default_angles) * dof_pos_scale
                dqj = d.qvel[6:] * dof_vel_scale
                quat = d.qpos[3:7]
                omega = d.qvel[3:6] * ang_vel_scale
                gravity = get_gravity_orientation(quat)

                period = 0.8
                phase = (counter * sim_dt) % period / period
                obs[:3] = omega
                obs[3:6] = gravity
                obs[6:9] = cmd * cmd_scale
                obs[9:9 + num_actions] = qj
                obs[9 + num_actions:9 + 2 * num_actions] = dqj
                obs[9 + 2 * num_actions:9 + 3 * num_actions] = action
                obs[9 + 3 * num_actions:9 + 3 * num_actions + 2] = [
                    np.sin(2 * np.pi * phase), np.cos(2 * np.pi * phase)]

                with torch.no_grad():
                    action = policy(torch.from_numpy(obs).unsqueeze(0)).numpy().squeeze()
                target_dof_pos = action * action_scale + default_angles
                if rec is not None:
                    rec.write(','.join(f'{v:.6f}' for v in target_dof_pos) + '\n')
                    rec.flush()

                if args.log:
                    err = np.abs(target_dof_pos - d.qpos[7:])
                    stats['err'] += float(err.mean())
                    stats['errmax'] = max(stats['errmax'], float(err.max()))
                    stats['dq'] += float(np.abs(d.qvel[6:]).mean())
                    stats['qmin'] = np.minimum(stats['qmin'], d.qpos[7:])
                    stats['qmax'] = np.maximum(stats['qmax'], d.qpos[7:])
                    stats['tmin'] = np.minimum(stats['tmin'], target_dof_pos)
                    stats['tmax'] = np.maximum(stats['tmax'], target_dof_pos)
                    stats['ticks'] += 1
                    stats['n'] += 1
                    if stats['n'] == 50:   # 50 krokov politiky = 1 s simulacie
                        x = d.qpos[0]
                        print(f'x={x:+.3f} rychlost={x - stats["x"]:+.2f} m/s z={d.qpos[2]:.3f} '
                              f'| chyba_poloh priem={stats["err"] / 50:.3f} max={stats["errmax"]:.2f} '
                              f'| priem|dq|={stats["dq"] / 50:.2f}')
                        stats.update({'n': 0, 'err': 0.0, 'errmax': 0.0, 'dq': 0.0, 'x': x})
                    if stats['ticks'] % 200 == 0:   # raz za 4 s simulacie
                        amp = stats['qmax'] - stats['qmin']
                        print('amplituda klbov [rad] L(HP HR HY K AP AR) P(HP HR HY K AP AR): '
                              + ' '.join(f'{a:.2f}' for a in amp))
                        stats['qmin'][:] = 9.0
                        stats['qmax'][:] = -9.0
                        tamp = stats['tmax'] - stats['tmin']
                        print(f'amplituda CIELOV  [rad] (cmd_x={cmd[0]:+.1f}): '
                              + ' '.join(f'{a:.2f}' for a in tamp))
                        stats['tmin'][:] = 9.0
                        stats['tmax'][:] = -9.0

            viewer.sync()
            dt_left = m.opt.timestep - (time.time() - step_start)
            if dt_left > 0:
                time.sleep(dt_left)


if __name__ == '__main__':
    main()
