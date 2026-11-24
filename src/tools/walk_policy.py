#!/usr/bin/env python3
"""
walk_policy: predtrenovana chodza Unitree G1 (unitree_rl_gym) v ROS2 + Gazebo.

Vstupy:   /joint_states, /imu/pelvis, /cmd_vel, /walk_policy/enable (std_msgs/Bool)
Vystup:   /leg_pid_controller/reference (cielove polohy 12 klbov nohy, 50 Hz)
          /chappie/detach (std_msgs/Empty) - uvolni robota zo zaveska

Rezimy:
  vypnuty (default)  drzi vychodiskovu pozu z g1.yaml
  zapnuty            spusta politiku; pri zapnuti sa robot uvolni zo zaveska
Ak sa robot nakloni o viac ako ~65 stupnov, politika sa sama vypne.

Zapnutie:  ros2 topic pub --once /walk_policy/enable std_msgs/msg/Bool "{data: true}"
Vypnutie:  ros2 topic pub --once /walk_policy/enable std_msgs/msg/Bool "{data: false}"

Spustenie (po source ROS a workspace):
  ~/ros_torch_venv/bin/python walk_policy.py
"""
import argparse
import os
import subprocess

import numpy as np
import rclpy
import torch
import yaml
from control_msgs.msg import MultiDOFCommand
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Empty

# poradie 12 klbov v politike (ako default_angles v g1.yaml)
LEG_ORDER = [f'{side}_{joint}_joint'
             for side in ('left', 'right')
             for joint in ('hip_pitch', 'hip_roll', 'hip_yaw',
                           'knee', 'ankle_pitch', 'ankle_roll')]

PERIOD = 0.8          # perioda kroku [s], rovnako ako v deploy_mujoco.py
FALL_GRAVITY_Z = -0.4  # gravity_z nad touto hodnotou = robot padol
HOLD_TICKS = 150       # kolko krokov (3 s simulacie) drzat pozu pred uvolnenim
CMD_LIMITS = np.array([[-0.5, 1.0], [-0.5, 0.5], [-1.0, 1.0]], dtype=np.float32)


def quat_to_mat(x, y, z, w):
    """Rotacia body->world z kvaternionu (x, y, z, w)."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rpy_to_mat(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


class WalkPolicy(Node):
    def __init__(self, cfg, policy, imu_rpy, replay=None):
        super().__init__(
            'walk_policy',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.policy = policy
        self.replay = replay
        self.replay_i = 0

        self.default = np.array(cfg['default_angles'], dtype=np.float32)
        self.ang_vel_scale = cfg['ang_vel_scale']
        self.dof_pos_scale = cfg['dof_pos_scale']
        self.dof_vel_scale = cfg['dof_vel_scale']
        self.action_scale = cfg['action_scale']
        self.cmd_scale = np.array(cfg['cmd_scale'], dtype=np.float32)
        self.num_actions = cfg['num_actions']
        self.num_obs = cfg['num_obs']
        dt = cfg['simulation_dt'] * cfg['control_decimation']

        # rotacia IMU -> panva (z rpy klbu imu_in_pelvis_joint)
        self.R_p_i = rpy_to_mat(*imu_rpy)

        self.joint_index = None
        self.q = np.zeros(self.num_actions, dtype=np.float32)
        self.dq = np.zeros(self.num_actions, dtype=np.float32)
        self.have_joints = False
        self.imu = None
        self.cmd = np.zeros(3, dtype=np.float32)
        self.action = np.zeros(self.num_actions, dtype=np.float32)
        self.enabled = False
        self.enable_requested = False
        self.hold_ticks = 0
        self.detach_left = 0
        self.t0 = None
        self.ticks = 0
        self.err_sum = 0.0
        self.err_max = 0.0
        self.dq_sum = 0.0
        self.dt_max = 0.0
        self.last_t = None
        self.q_min = np.full(self.num_actions, 9.0)
        self.q_max = np.full(self.num_actions, -9.0)
        self.t_min = np.full(self.num_actions, 9.0)
        self.t_max = np.full(self.num_actions, -9.0)

        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(Imu, '/imu/pelvis', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(Bool, '/walk_policy/enable', self.on_enable, 10)
        self.pub = self.create_publisher(
            MultiDOFCommand, '/leg_pid_controller/reference', 10)
        self.detach_pub = self.create_publisher(Empty, '/chappie/detach', 10)
        self.timer = self.create_timer(dt, self.step)
        self.get_logger().info(
            f'walk_policy: perioda {dt:.3f} s, politika VYPNUTA (drzim vychodiskovu pozu)')

    # ------------------------------------------------------------ callbacky
    def on_joints(self, msg):
        if self.joint_index is None:
            try:
                self.joint_index = [msg.name.index(n) for n in LEG_ORDER]
            except ValueError as e:
                self.get_logger().error(f'V /joint_states chyba klb: {e}')
                return
        self.q = np.array([msg.position[i] for i in self.joint_index], dtype=np.float32)
        self.dq = np.array([msg.velocity[i] for i in self.joint_index], dtype=np.float32)
        self.have_joints = True

    def on_imu(self, msg):
        self.imu = msg

    def on_cmd(self, msg):
        c = np.array([msg.linear.x, msg.linear.y, msg.angular.z], dtype=np.float32)
        self.cmd = np.clip(c, CMD_LIMITS[:, 0], CMD_LIMITS[:, 1])
        self.cmd_stamp = self.get_clock().now()

    def on_enable(self, msg):
        if msg.data:
            if not self.enabled and not self.enable_requested:
                self.enable_requested = True
                self.get_logger().info(
                    'Zapnutie vyziadane, pockam kym je robot pripraveny a drzi pozu')
        else:
            self.enable_requested = False
            if self.enabled:
                self.enabled = False
                self.hold_ticks = 0
                self.get_logger().info('Politika VYPNUTA, drzim vychodiskovu pozu')

    # ------------------------------------------------------------ vystup
    def gz_detach(self):
        """Uvolnenie priamo cez Gazebo (obchadza ros_gz_bridge)."""
        try:
            subprocess.Popen(
                ['gz', 'topic', '-t', '/chappie/detach', '-m', 'gz.msgs.Empty', '-p', ' '],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError as e:
            self.get_logger().warn(f'Nepodarilo sa spustit gz topic: {e}')

    def publish_targets(self, target):
        msg = MultiDOFCommand()
        msg.dof_names = LEG_ORDER
        msg.values = [float(v) for v in target]
        msg.values_dot = [0.0] * self.num_actions
        self.pub.publish(msg)

    # ------------------------------------------------------------ hlavny krok
    def step(self):
        # opakovane posielanie uvolnenia (sprava nic nepokazi, ak pride viackrat)
        if self.detach_left > 0:
            if self.detach_left % 10 == 0:
                self.detach_pub.publish(Empty())
            if self.detach_left in (100, 50):
                self.gz_detach()
            self.detach_left -= 1

        if self.imu is None or not self.have_joints:
            return

        # watchdog: ak prikazy z klavesnice prestanu chodit, robot sa zastavi
        timeout = float(os.environ.get('CHAPPIE_CMD_TIMEOUT', '0'))
        stamp = getattr(self, 'cmd_stamp', None)
        if timeout > 0 and stamp is not None:
            if (self.get_clock().now() - stamp).nanoseconds * 1e-9 > timeout:
                self.cmd[:] = 0.0

        o = self.imu.orientation
        R_w_i = quat_to_mat(o.x, o.y, o.z, o.w)
        gravity = self.R_p_i @ (R_w_i.T @ np.array([0.0, 0.0, -1.0]))

        if not self.enabled:
            self.publish_targets(self.default)
            self.hold_ticks += 1
            if self.enable_requested and self.hold_ticks >= HOLD_TICKS:
                self.enable_requested = False
                self.enabled = True
                self.t0 = None
                self.action[:] = 0.0
                if hasattr(self.policy, 'reset_memory'):
                    self.policy.reset_memory()
                self.detach_left = 100
                self.replay_i = 0
                self.get_logger().info('Politika ZAPNUTA, uvolnujem robota zo zaveska')
            return

        if gravity[2] > FALL_GRAVITY_Z:
            self.enabled = False
            self.hold_ticks = 0
            self.get_logger().warn('Robot sa nakláňa viac ako ~65 stupnov, politika vypnuta')
            self.publish_targets(self.default)
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        phase = ((now - self.t0) % PERIOD) / PERIOD

        w = self.imu.angular_velocity
        omega = self.R_p_i @ np.array([w.x, w.y, w.z])

        n = self.num_actions
        obs = np.zeros(self.num_obs, dtype=np.float32)
        obs[:3] = omega * self.ang_vel_scale
        obs[3:6] = gravity
        obs[6:9] = self.cmd * self.cmd_scale
        obs[9:9 + n] = (self.q - self.default) * self.dof_pos_scale
        obs[9 + n:9 + 2 * n] = self.dq * self.dof_vel_scale
        obs[9 + 2 * n:9 + 3 * n] = self.action
        obs[9 + 3 * n:9 + 3 * n + 2] = [np.sin(2 * np.pi * phase),
                                        np.cos(2 * np.pi * phase)]

        with torch.no_grad():
            action = self.policy(torch.from_numpy(obs).unsqueeze(0)).numpy().squeeze()
        if not np.all(np.isfinite(action)):
            self.get_logger().warn('Politika vratila nekonecne hodnoty, preskakujem')
            return
        self.action = action.astype(np.float32)
        target = self.action * self.action_scale + self.default
        if self.replay is not None:   # open-loop: ciele zo suboru namiesto politiky
            target = self.replay[min(self.replay_i, len(self.replay) - 1)].astype(np.float32)
            self.replay_i += 1
        self.publish_targets(target)

        # meranie pre porovnanie s MuJoCo
        err = np.abs(target - self.q)
        self.err_sum += float(err.mean())
        self.err_max = max(self.err_max, float(err.max()))
        self.dq_sum += float(np.abs(self.dq).mean())
        if self.last_t is not None:
            self.dt_max = max(self.dt_max, now - self.last_t)
        self.last_t = now
        self.q_min = np.minimum(self.q_min, self.q)
        self.q_max = np.maximum(self.q_max, self.q)
        self.t_min = np.minimum(self.t_min, target)
        self.t_max = np.maximum(self.t_max, target)

        self.ticks += 1
        if self.ticks % 50 == 0:   # raz za sekundu
            stamp = self.imu.header.stamp
            imu_t = stamp.sec + stamp.nanosec * 1e-9
            age = f'{(now - imu_t) * 1000:.0f} ms' if imu_t > 0 else 'n/a'
            self.get_logger().info(
                f'gravity_z={gravity[2]:+.2f} cmd_x={self.cmd[0]:+.1f} '
                f'| chyba_poloh priem={self.err_sum / 50:.3f} max={self.err_max:.2f} '
                f'| priem|dq|={self.dq_sum / 50:.2f} '
                f'| max_dt={self.dt_max * 1000:.0f} ms | vek_imu={age}')
            self.err_sum = self.err_max = self.dq_sum = self.dt_max = 0.0

        if self.ticks % 200 == 0:   # raz za 4 s simulacie
            amp = self.q_max - self.q_min
            self.get_logger().info(
                'amplituda klbov [rad] L(HP HR HY K AP AR) P(HP HR HY K AP AR): '
                + ' '.join(f'{a:.2f}' for a in amp))
            tamp = self.t_max - self.t_min
            self.get_logger().info(
                f'amplituda CIELOV  [rad] (cmd_x={self.cmd[0]:+.1f}): '
                + ' '.join(f'{a:.2f}' for a in tamp))
            self.q_min[:] = 9.0
            self.q_max[:] = -9.0
            self.t_min[:] = 9.0
            self.t_max[:] = -9.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', default=os.path.expanduser('~/refs/unitree_rl_gym'))
    ap.add_argument('--config', default='g1.yaml')
    ap.add_argument('--imu-rpy', type=float, nargs=3, default=[0.0, 0.0, 0.0])
    ap.add_argument('--replay', default='', help='CSV s cielmi z MuJoCo (open-loop)')
    args, ros_args = ap.parse_known_args()
    root = os.path.abspath(os.path.expanduser(args.root))

    with open(f'{root}/deploy/deploy_mujoco/configs/{args.config}') as f:
        cfg = yaml.safe_load(f)
    policy_path = cfg['policy_path'].replace('{LEGGED_GYM_ROOT_DIR}', root)
    policy = torch.jit.load(policy_path, map_location='cpu')
    policy.eval()

    replay = np.loadtxt(args.replay, delimiter=',') if args.replay else None
    if replay is not None:
        print(f'REPLAY: nacitanych {len(replay)} krokov z {args.replay}')
    rclpy.init(args=ros_args)
    node = WalkPolicy(cfg, policy, args.imu_rpy, replay)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
