#!/usr/bin/env python3
"""Print one telemetry line per `period` seconds (default 1): time, speed, lap, lap time,
best lap, collisions, throttle, steering, front clearance, side distances.
Run inside the devkit container:  scripts/dk.sh python3 /home/scripts/monitor.py [period] [duration]
"""
import math
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Int32

NS = '/autodrive/roboracer_1'
period = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
duration = float(sys.argv[2]) if len(sys.argv) > 2 else 1e9
rclpy.init()
node = rclpy.create_node('monitor')
s = dict(speed=0.0, lap=0, lap_time=0.0, best=0.0, last=0.0, col=0, thr=0.0, steer=0.0, front=0.0, left=0.0, right=0.0)


def on_scan(m):
    r = np.asarray(m.ranges, dtype=float)
    r[~np.isfinite(r) | (r > m.range_max)] = m.range_max
    idx = lambda deg: int(round((math.radians(deg) - m.angle_min) / m.angle_increment))
    s['front'] = float(r[idx(-10):idx(10) + 1].min())
    s['left'] = float(r[idx(90)])
    s['right'] = float(r[idx(-90)])


def setter(k, f=lambda d: d):
    return lambda m: s.__setitem__(k, f(m.data))


node.create_subscription(LaserScan, NS + '/lidar', on_scan, 1)
node.create_subscription(Odometry, NS + '/odom', lambda m: s.__setitem__('speed', math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)), 1)
node.create_subscription(Int32, NS + '/lap_count', setter('lap'), 1)
node.create_subscription(Float32, NS + '/lap_time', setter('lap_time'), 1)
node.create_subscription(Float32, NS + '/last_lap_time', setter('last'), 1)
node.create_subscription(Float32, NS + '/best_lap_time', setter('best'), 1)
node.create_subscription(Int32, NS + '/collision_count', setter('col'), 1)
node.create_subscription(Float32, NS + '/throttle_command', setter('thr'), 1)
node.create_subscription(Float32, NS + '/steering_command', setter('steer'), 1)

t0 = time.time()
nxt = t0
print('t  speed  lap  lap_t  last  best  col  thr  steer  front  left  right', flush=True)
try:
    while rclpy.ok() and time.time() - t0 < duration:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() >= nxt:
            nxt += period
            print(f"{time.time()-t0:5.1f} {s['speed']:5.2f} {s['lap']:3d} {s['lap_time']:6.2f} {s['last']:6.2f} {s['best']:6.2f} "
                  f"{s['col']:3d} {s['thr']:5.2f} {s['steer']:5.2f} {s['front']:5.2f} {s['left']:5.2f} {s['right']:5.2f}", flush=True)
except KeyboardInterrupt:
    pass
node.destroy_node()
rclpy.try_shutdown()
