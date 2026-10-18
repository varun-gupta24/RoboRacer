#!/usr/bin/env python3
"""Record scans, odom and commands to an .npz for offline analysis.
  scripts/dk.sh python3 /home/scripts/record.py 20 /home/scripts/rec.npz
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
duration, out = float(sys.argv[1]), sys.argv[2]
rclpy.init()
node = rclpy.create_node('recorder')
rows, meta, cur = [], {}, dict(vx=0.0, vy=0.0, wz=0.0, thr=0.0, steer=0.0, col=0, lap=0)


def on_scan(m):
    meta.update(angle_min=m.angle_min, angle_increment=m.angle_increment, range_min=m.range_min, range_max=m.range_max)
    rows.append((time.monotonic(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, np.asarray(m.ranges, dtype=np.float32), dict(cur)))


def on_odom(m):
    q = m.pose.pose.orientation
    cur.update(vx=m.twist.twist.linear.x, vy=m.twist.twist.linear.y, wz=m.twist.twist.angular.z,
               x=m.pose.pose.position.x, y=m.pose.pose.position.y,
               yaw=math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


node.create_subscription(LaserScan, NS + '/lidar', on_scan, 1)
node.create_subscription(Odometry, NS + '/odom', on_odom, 1)
for k, topic, T in (('thr', 'throttle_command', Float32), ('steer', 'steering_command', Float32), ('col', 'collision_count', Int32), ('lap', 'lap_count', Int32)):
    node.create_subscription(T, f'{NS}/{topic}', (lambda k: lambda m: cur.__setitem__(k, m.data))(k), 1)
t0 = time.time()
while time.time() - t0 < duration:
    rclpy.spin_once(node, timeout_sec=0.05)
np.savez(out, t=[r[0] for r in rows], stamp=[r[1] for r in rows], ranges=np.stack([r[2] for r in rows]),
         **{k: [r[3][k] for r in rows] for k in cur}, **meta)
print(f'saved {len(rows)} scans to {out}')
node.destroy_node()
rclpy.try_shutdown()
