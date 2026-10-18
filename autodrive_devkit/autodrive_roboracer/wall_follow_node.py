#!/usr/bin/env python3
"""LiDAR-only wall following for the AutoDRIVE RoboRacer.

Two-wall centering with F1TENTH-style lookahead projection, PD steering,
front-clearance speed scheduling with closed-loop speed from odometry, and a
safety brake. All gains are ROS parameters so they can be tuned live with
`ros2 param set /wall_follow_node <name> <value>`.
"""

import math
import time
from collections import deque
from types import SimpleNamespace

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Int32

NS = '/autodrive/roboracer_1'
MAX_STEER_RAD = 0.52  # steering command +1 == full left lock

# name: (default, comment)
PARAMS = {
    # wall geometry
    'side_angle_deg': (90.0, 'beam b, perpendicular to the car'),
    'alpha_deg': (50.0, 'beam a is alpha degrees ahead of beam b'),
    'lookahead': (0.6, 'projection distance L [m]'),
    'lookahead_per_mps': (0.2, 'extra L per m/s of speed'),
    'open_dist': (2.5, 'side beam longer than this => wall missing, use the other side'),
    'open_pull': (0.3, 'one-wall mode: gain on pulling toward the wall (pushing away stays at 1)'),
    'half_width_init': (0.75, 'initial guess of half track width [m]'),
    'half_width_alpha': (0.02, 'EMA rate for the half-width estimate'),
    # steering PD
    'kp': (0.4, 'rad per m of centering error'),
    'kd': (0.02, 'rad per m/s of error rate'),
    'deriv_window': (0.08, 'seconds of error history the derivative is taken over'),
    # speed schedule
    'front_cone_deg': (15.0, 'half-angle of the front clearance cone'),
    'front_alpha': (0.4, 'EMA rate for front clearance used by the speed schedule'),
    'v_min': (1.0, 'speed floor [m/s]'),
    'v_max': (4.0, 'speed ceiling [m/s]'),
    'brake_accel': (3.0, 'deceleration budget [m/s^2]: v_target = sqrt(2*a*(front - brake_dist - v*react_time))'),
    'react_time': (0.1, 'latency allowance [s]; the distance covered in it is not available for braking'),
    'speed_lead': (0.25, 'seconds of speed prediction (v + accel*lead) fed to the throttle loop'),
    'steer_slowdown': (0.5, 'v_target *= 1 - steer_slowdown*|steer_cmd|'),
    'throttle_ff': (0.1, 'feed-forward throttle per m/s of target speed'),
    'throttle_kp': (0.8, 'throttle per m/s of speed error'),
    # corner assist: steer away from whichever front-diagonal cone is closer
    'corner_cone_deg': (45.0, 'centre of the diagonal cones (+/-), half-width 20 deg'),
    'corner_dist': (0.7, 'diagonal clearance below this adds avoidance steering'),
    'corner_gain': (0.6, 'rad of steering per m of diagonal clearance deficit'),
    # safety
    'brake_dist': (0.5, 'front clearance below this => brake'),
    'brake_throttle': (-0.3, 'throttle while braking and still moving'),
    'reverse_throttle': (-0.25, 'throttle while backing out of a stall'),
    'reverse_time': (1.0, 'seconds to back up when stalled against a wall'),
    'scan_timeout': (0.5, 'seconds without a scan before throttle is zeroed'),
    # logging
    'log_file': ('', 'append one CSV line per completed lap (empty = off)'),
}


def clip(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def wall(b, a, alpha, lookahead):
    """Perpendicular distance and lookahead-projected distance to a wall from
    beam b (perpendicular) and beam a (alpha rad ahead of b). phi < 0 means the
    car is heading toward that wall, so the projected distance shrinks."""
    phi = math.atan2(a * math.cos(alpha) - b, a * math.sin(alpha))
    d = b * math.cos(phi)
    return d, d + lookahead * math.sin(phi)


def new_state(p):
    return SimpleNamespace(half_width=p.half_width_init, err_hist=deque(), front_f=None, v_hist=deque())


def control(r, angle_min, angle_increment, speed, p, st, now):
    """Pure controller step. r: cleaned ranges (invalid -> range_max). Returns
    (steer_cmd, throttle, front_clearance, debug dict). Mutates st."""
    n = len(r)

    def index(theta_deg):
        return clip(int(round((math.radians(theta_deg) - angle_min) / angle_increment)), 0, n - 1)

    def sample(theta_deg):  # median over +/-2 beams around angle theta
        i = index(theta_deg)
        return float(np.median(r[max(0, i - 2):i + 3]))

    def cone(lo_deg, hi_deg):
        return float(np.min(r[index(lo_deg):index(hi_deg) + 1]))

    alpha = math.radians(p.alpha_deg)
    L = p.lookahead + p.lookahead_per_mps * max(0.0, speed)
    b_l = sample(p.side_angle_deg)
    a_l = sample(p.side_angle_deg - p.alpha_deg)
    b_r = sample(-p.side_angle_deg)
    a_r = sample(-p.side_angle_deg + p.alpha_deg)
    d_l, dl_l = wall(b_l, a_l, alpha, L)
    d_r, dl_r = wall(b_r, a_r, alpha, L)
    left_ok = b_l < p.open_dist
    right_ok = b_r < p.open_dist

    if left_ok and right_ok:
        err = dl_l - dl_r  # >0: too far right -> steer left
        st.half_width += p.half_width_alpha * ((d_l + d_r) / 2 - st.half_width)
    elif right_ok:  # inner wall gone at a corner exit: the remaining wall is usually ahead, so
        e = st.half_width - dl_r  # push away from it at full gain, pull toward it only weakly
        err = e if e > 0 else p.open_pull * e
    elif left_ok:
        e = dl_l - st.half_width
        err = e if e < 0 else p.open_pull * e
    else:
        err = 0.0  # ponytail: nothing to follow, hold straight; a gap-follower would go here

    # derivative over a fixed time window (scan timing is bursty), reset on mode switch
    mode = (left_ok, right_ok)
    if st.err_hist and st.err_hist[0][2] != mode:
        st.err_hist.clear()
    st.err_hist.append((now, err, mode))
    while len(st.err_hist) > 2 and now - st.err_hist[1][0] >= p.deriv_window:
        st.err_hist.popleft()
    t_old, e_old, _ = st.err_hist[0]
    d_err = (err - e_old) / (now - t_old) if now - t_old >= p.deriv_window / 2 else 0.0

    # ponytail: crude obstacle repulsion for hairpins; a proper gap follower is the upgrade path
    c = p.corner_cone_deg
    near_l = cone(c - 20, c + 20)
    near_r = cone(-c - 20, -c + 20)
    corner = p.corner_gain * (max(0.0, p.corner_dist - near_r) - max(0.0, p.corner_dist - near_l))
    steer_rad = p.kp * err + p.kd * d_err + corner
    steer_cmd = clip(steer_rad / MAX_STEER_RAD, -1.0, 1.0)

    # speed: front clearance in a cone, scaled down by steering, closed loop on odom speed
    front = cone(-p.front_cone_deg, p.front_cone_deg)
    st.front_f = front if st.front_f is None else st.front_f + p.front_alpha * (front - st.front_f)
    stop_dist = max(0.0, st.front_f - p.brake_dist - p.react_time * speed)
    v_target = clip(math.sqrt(2.0 * p.brake_accel * stop_dist), p.v_min, p.v_max)
    v_target *= 1.0 - p.steer_slowdown * abs(steer_cmd)
    # predict the speed a little ahead (the drivetrain and bridge add ~0.1-0.2 s of lag)
    st.v_hist.append((now, speed))
    while len(st.v_hist) > 2 and now - st.v_hist[1][0] >= 0.2:
        st.v_hist.popleft()
    t_old, v_old = st.v_hist[0]
    accel = (speed - v_old) / (now - t_old) if now - t_old >= 0.1 else 0.0
    v_pred = speed + p.speed_lead * accel
    throttle = clip(p.throttle_ff * v_target + p.throttle_kp * (v_target - v_pred), -1.0, 1.0)
    if front < p.brake_dist:  # brake, but never push backwards (speed is signed)
        throttle = p.brake_throttle if speed > 0.3 else 0.0
    dbg = dict(err=err, p_term=p.kp * err, d_term=p.kd * d_err, corner=corner, dl_l=dl_l, dl_r=dl_r,
               d_l=d_l, d_r=d_r, mode=mode, front=front, v_target=v_target, near_l=near_l, near_r=near_r)
    return steer_cmd, throttle, front, dbg


class WallFollowNode(Node):
    def __init__(self):
        super().__init__('wall_follow_node')
        self.p = SimpleNamespace(**{k: v[0] for k, v in PARAMS.items()})
        for k, (default, desc) in PARAMS.items():
            self.declare_parameter(k, default)
            setattr(self.p, k, self.get_parameter(k).value)
        self.add_on_set_parameters_callback(self._on_params)

        qos = QoSProfile(depth=1)  # same as teleop_keyboard
        self.pub_throttle = self.create_publisher(Float32, NS + '/throttle_command', qos)
        self.pub_steering = self.create_publisher(Float32, NS + '/steering_command', qos)
        self.create_subscription(LaserScan, NS + '/lidar', self.on_scan, qos)
        self.create_subscription(Odometry, NS + '/odom', self.on_odom, qos)
        self.create_subscription(Int32, NS + '/lap_count', self.on_lap, qos)
        self.create_subscription(Float32, NS + '/last_lap_time', lambda m: setattr(self, 'last_lap_time', m.data), qos)
        self.create_subscription(Float32, NS + '/best_lap_time', lambda m: setattr(self, 'best_lap_time', m.data), qos)
        self.create_subscription(Int32, NS + '/collision_count', self.on_collision, qos)
        self.create_timer(0.1, self.watchdog)

        self.speed = 0.0
        self.st = new_state(self.p)
        self.last_scan_t = None
        self.lap_count = 0
        self.last_lap_time = 0.0
        self.best_lap_time = 0.0
        self.collisions = 0
        self.lap_collisions = 0
        self.reverse_until = 0.0
        self.stall_since = None
        self.throttle_msg = Float32()
        self.steering_msg = Float32()
        self.get_logger().info('wall_follow_node up; tune with: ros2 param set /wall_follow_node <name> <value>')

    # ---- callbacks -------------------------------------------------------
    def _on_params(self, params):
        for prm in params:
            if prm.name in PARAMS:
                setattr(self.p, prm.name, prm.value)
        return SetParametersResult(successful=True)

    def on_odom(self, msg):
        self.speed = msg.twist.twist.linear.x  # body frame: signed forward speed

    def on_collision(self, msg):
        if msg.data > self.collisions:
            self.get_logger().warn(f'COLLISION #{msg.data} (lap {self.lap_count + 1})')
        self.collisions = msg.data

    def on_lap(self, msg):
        if msg.data <= self.lap_count:
            self.lap_count = msg.data
            return
        self.lap_count = msg.data
        hit = self.collisions - self.lap_collisions
        self.lap_collisions = self.collisions
        self.get_logger().info(
            f'LAP {msg.data} done: last={self.last_lap_time:.2f}s best={self.best_lap_time:.2f}s '
            f'collisions_this_lap={hit} total={self.collisions}')
        if self.p.log_file:
            gains = ','.join(f'{getattr(self.p, k)}' for k in PARAMS if k != 'log_file')
            with open(self.p.log_file, 'a') as f:
                f.write(f'{time.time():.0f},{msg.data},{self.last_lap_time:.3f},{self.best_lap_time:.3f},'
                        f'{hit},{self.collisions},{gains}\n')

    def watchdog(self):
        if self.last_scan_t is not None and time.monotonic() - self.last_scan_t > self.p.scan_timeout:
            self.command(0.0, 0.0)

    # ---- controller ------------------------------------------------------
    def on_scan(self, msg):
        p = self.p
        now = time.monotonic()
        self.last_scan_t = now
        r = np.asarray(msg.ranges, dtype=float)
        bad = ~np.isfinite(r) | (r < msg.range_min) | (r > msg.range_max)
        r[bad] = msg.range_max
        steer_cmd, throttle, front, _ = control(r, msg.angle_min, msg.angle_increment, self.speed, p, self.st, now)

        # stall recovery: stopped against a wall -> back up with the wheel turned the other way
        stalled = front < p.brake_dist and abs(self.speed) < 0.1
        if stalled and self.stall_since is None:
            self.stall_since = now
        elif not stalled:
            self.stall_since = None
        if self.stall_since is not None and now - self.stall_since > 0.5 and now > self.reverse_until:
            self.reverse_until = now + p.reverse_time
            self.get_logger().warn(f'stalled (front={front:.2f} m), reversing')
        if now < self.reverse_until:
            throttle, steer_cmd = p.reverse_throttle, -steer_cmd

        self.command(throttle, steer_cmd)

    def command(self, throttle, steer):
        self.throttle_msg.data = float(throttle)
        self.steering_msg.data = float(steer)
        self.pub_throttle.publish(self.throttle_msg)
        self.pub_steering.publish(self.steering_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.command(0.0, 0.0)  # leave the car stopped
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
