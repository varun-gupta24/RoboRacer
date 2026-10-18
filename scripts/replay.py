#!/usr/bin/env python3
"""Replay a recording (scripts/record.py) through the controller offline and print the
steering terms, so gains can be inspected without the simulator.
  scripts/dk.sh python3 /home/scripts/replay.py /home/scripts/rec.npz [start_idx] [count] [kp=.. ...]
"""
import sys
from types import SimpleNamespace

import numpy as np

from autodrive_roboracer.wall_follow_node import PARAMS, control, new_state

d = np.load(sys.argv[1])
i0 = int(sys.argv[2]) if len(sys.argv) > 2 else 0
cnt = int(sys.argv[3]) if len(sys.argv) > 3 else 40
p = SimpleNamespace(**{k: v[0] for k, v in PARAMS.items()})
for a in sys.argv[4:]:
    k, v = a.split('=')
    setattr(p, k, float(v))
st = new_state(p)
am, inc, rmin, rmax = (float(d[k]) for k in ('angle_min', 'angle_increment', 'range_min', 'range_max'))
t = d['t']
v = np.hypot(d['vx'], d['vy'])
print('   i     t  steer  rec_st  p_term d_term corner  err   dl_l  dl_r  front  vt   thr')
for i in range(len(t)):
    r = d['ranges'][i].astype(float)
    r[~np.isfinite(r) | (r < rmin) | (r > rmax)] = rmax
    steer, thr, front, g = control(r, am, inc, float(v[i]), p, st, float(t[i]))
    if i0 <= i < i0 + cnt:
        print(f"{i:4d} {t[i]-t[0]:6.2f} {steer:6.2f} {d['steer'][i]:6.2f}  {g['p_term']:6.2f} {g['d_term']:6.2f} {g['corner']:6.2f} "
              f"{g['err']:5.2f} {g['dl_l']:5.2f} {g['dl_r']:5.2f} {front:5.2f} {g['v_target']:4.1f} {thr:5.2f}")
