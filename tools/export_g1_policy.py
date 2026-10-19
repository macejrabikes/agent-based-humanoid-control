#!/usr/bin/env python3
"""
Exportuje vahy predtrenovanej politiky Unitree G1 (motion.pt, TorchScript) do jednoduchého
binarneho suboru pre C++ uzol (bez potreby LibTorch) + referencne data na overenie.

Siet: LSTM(47 -> 64) + Linear(64 -> 32) + ELU + Linear(32 -> 12), s pamatou medzi krokmi.

Pouzitie (v ros_torch_venv):
  ~/ros_torch_venv/bin/python export_g1_policy.py
Vystup:
  <balik>/config/g1_policy.bin       vahy (float32)
  <balik>/config/g1_policy_ref.txt   300 krokov: 47 vstupov + 12 vystupov (z PyTorchu)
"""
import argparse
import os
import struct

import numpy as np
import torch

ap = argparse.ArgumentParser()
ap.add_argument('--policy', default=os.path.expanduser(
    '~/refs/unitree_rl_gym/deploy/pre_train/g1/motion.pt'))
ap.add_argument('--outdir', default=os.path.expanduser(
    '~/agent-based-humanoid-control/src/humanoid_walk/config'))
args = ap.parse_args()
os.makedirs(args.outdir, exist_ok=True)

m = torch.jit.load(args.policy, map_location='cpu')
m.eval()
sd = {k: v.detach().cpu().numpy().astype('<f4') for k, v in m.state_dict().items()}

order = ['memory.weight_ih_l0', 'memory.weight_hh_l0', 'memory.bias_ih_l0',
         'memory.bias_hh_l0', 'actor.0.weight', 'actor.0.bias',
         'actor.2.weight', 'actor.2.bias']
expect = {'memory.weight_ih_l0': (256, 47), 'memory.weight_hh_l0': (256, 64),
          'memory.bias_ih_l0': (256,), 'memory.bias_hh_l0': (256,),
          'actor.0.weight': (32, 64), 'actor.0.bias': (32,),
          'actor.2.weight': (12, 32), 'actor.2.bias': (12,)}
for k in order:
    assert sd[k].shape == expect[k], (k, sd[k].shape)

path = os.path.join(args.outdir, 'g1_policy.bin')
with open(path, 'wb') as f:
    f.write(b'G1PL')
    f.write(struct.pack('<4i', 47, 64, 32, 12))
    for k in order:
        f.write(np.ascontiguousarray(sd[k]).tobytes())
print('zapisane:', path, os.path.getsize(path), 'B')

# referencny priebeh z PyTorchu (so zachovanim pamate LSTM)
rng = np.random.default_rng(1)
m.reset_memory()
rows = []
with torch.no_grad():
    for i in range(300):
        x = (rng.normal(size=47) * 0.6).astype('<f4')
        y = m(torch.from_numpy(x).unsqueeze(0)).numpy().squeeze()
        rows.append(np.concatenate([x, y]))
ref = os.path.join(args.outdir, 'g1_policy_ref.txt')
np.savetxt(ref, np.array(rows), fmt='%.9g')
print('zapisane:', ref)
