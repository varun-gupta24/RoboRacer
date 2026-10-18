#!/usr/bin/env python3
"""One tuning trial: set node parameters, reset the simulator, watch for `duration` seconds,
then print a one-line summary and append it to /home/scripts/trials.csv.
Run inside the devkit container:
  scripts/dk.sh python3 /home/scripts/trial.py 90 kp=0.7 v_max=4.0
"""
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Bool, Float32, Int32

NS = '/autodrive/roboracer_1'
duration = float(sys.argv[1])
overrides = dict(a.split('=', 1) for a in sys.argv[2:])

rclpy.init()
node = rclpy.create_node('trial')

# 1. parameters
if overrides:
    cli = node.create_client(SetParameters, '/wall_follow_node/set_parameters')
    assert cli.wait_for_service(5.0), 'wall_follow_node not running'
    req = SetParameters.Request()
    for k, v in overrides.items():
        pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v))
        req.parameters.append(Parameter(name=k, value=pv))
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    assert fut.result() and all(r.successful for r in fut.result().results), fut.result()

# 2. reset (True then False, like teleop; the bridge latches the last value)
pub_reset = node.create_publisher(Bool, '/autodrive/reset_command', 1)
for val in (True, False):
    pub_reset.publish(Bool(data=val))
    time.sleep(0.3)

# 3. watch
s = dict(speed=0.0, lap=None, last=0.0, best=0.0, col=None)
laps, cols = [], []
node.create_subscription(Odometry, NS + '/odom', lambda m: s.__setitem__('speed', math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)), 1)
node.create_subscription(Float32, NS + '/last_lap_time', lambda m: s.__setitem__('last', m.data), 1)
node.create_subscription(Float32, NS + '/best_lap_time', lambda m: s.__setitem__('best', m.data), 1)


def on_lap(m):
    if s['lap'] is not None and m.data > s['lap']:
        laps.append(s['last'])
        print(f'  lap {m.data}: {s["last"]:.2f}s  collisions so far {len(cols)}', flush=True)
    s['lap'] = m.data


def on_col(m):
    if s['col'] is not None and m.data > s['col']:
        cols.append(time.time() - t0)
        print(f'  COLLISION at t={cols[-1]:.1f}s (lap {len(laps) + 1})', flush=True)
    s['col'] = m.data


node.create_subscription(Int32, NS + '/lap_count', on_lap, 1)
node.create_subscription(Int32, NS + '/collision_count', on_col, 1)

t0 = time.time()
speeds, stalls, stop_since = [], 0, None
time.sleep(1.0)  # let the reset land before judging
while time.time() - t0 < duration:
    rclpy.spin_once(node, timeout_sec=0.05)
    speeds.append(s['speed'])
    if s['speed'] < 0.05:
        stop_since = stop_since or time.time()
        if time.time() - stop_since > 2.0:
            stalls += 1
            stop_since = time.time()
    else:
        stop_since = None

# note: lap 1 after a reset is a partial lap from the spawn point, so it is dropped from the stats
full = laps[1:] if len(laps) > 1 else []
summary = (f'laps={len(laps)} best={min(full) if full else float("nan"):.2f} mean={sum(full)/len(full) if full else float("nan"):.2f} '
           f'collisions={len(cols)} stalls={stalls} vmax={max(speeds):.2f} vmean={sum(speeds)/len(speeds):.2f}')
print(summary, flush=True)
with open('/home/scripts/trials.csv', 'a') as f:
    f.write(f'{time.strftime("%Y-%m-%d %H:%M:%S")},{duration:.0f},"{" ".join(sys.argv[2:])}",{summary}\n')
node.destroy_node()
rclpy.try_shutdown()
