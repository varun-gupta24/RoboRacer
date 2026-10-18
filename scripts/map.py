#!/usr/bin/env python3
"""Render a recording as a PNG map: LiDAR points (grey), trajectory coloured by speed
(blue slow -> red fast), collisions as red circles, every 5 s a time label.
  scripts/dk.sh python3 /home/scripts/map.py /home/scripts/rec.npz /home/scripts/map.png
"""
import sys

import numpy as np
from PIL import Image, ImageDraw

d = np.load(sys.argv[1])
x, y, yaw = d['x'], d['y'], d['yaw']
am, inc, rmax = float(d['angle_min']), float(d['angle_increment']), float(d['range_max'])
ang = am + inc * np.arange(d['ranges'].shape[1])
pts = []
for i in range(0, len(x), 3):
    r = d['ranges'][i].astype(float)
    ok = np.isfinite(r) & (r < rmax)
    a = ang[ok] + yaw[i]
    pts.append(np.c_[x[i] + 0.27 * np.cos(yaw[i]) + r[ok] * np.cos(a), y[i] + 0.27 * np.sin(yaw[i]) + r[ok] * np.sin(a)])
pts = np.vstack(pts)
lo, hi = pts.min(0) - 0.5, pts.max(0) + 0.5
S = 60  # px per metre
W, H = int((hi[0] - lo[0]) * S), int((hi[1] - lo[1]) * S)
img = Image.new('RGB', (W, H), 'white')
dr = ImageDraw.Draw(img)
P = lambda px, py: (int((px - lo[0]) * S), int(H - (py - lo[1]) * S))
for px, py in pts:
    img.putpixel(P(px, py), (150, 150, 150))
v = np.hypot(d['vx'], d['vy'])
t = d['t'] - d['t'][0]
for i in range(1, len(x)):
    f = min(v[i] / 5.0, 1.0)
    dr.line([P(x[i - 1], y[i - 1]), P(x[i], y[i])], fill=(int(255 * f), 0, int(255 * (1 - f))), width=2)
col = d['col']
for i in np.flatnonzero(np.diff(col) > 0) + 1:
    cx, cy = P(x[i], y[i])
    dr.ellipse([cx - 8, cy - 8, cx + 8, cy + 8], outline='red', width=3)
    dr.text((cx + 10, cy - 10), f'{t[i]:.0f}s', fill='red')
for i in range(0, len(x), 1):
    if i == 0 or int(t[i] // 5) != int(t[i - 1] // 5):
        dr.text(P(x[i], y[i]), f'{t[i]:.0f}', fill='black')
cx, cy = P(x[0], y[0])
dr.ellipse([cx - 5, cy - 5, cx + 5, cy + 5], fill='green')
img.save(sys.argv[2])
print('saved', sys.argv[2], img.size, 'collisions', int(col[-1] - col[0]))
