# Plan

Goal: LiDAR-only wall following around the AutoDRIVE RoboRacer practice track, 3 consecutive
contact-free laps, then the best lap time. Deliverable is this repo; the human records the
video from the simulator GUI.

## Constraints honoured
- Only `autodrive_devkit/` changed: new `autodrive_roboracer/wall_follow_node.py`, one
  `console_scripts` entry in `setup.py`, one geometry test. `autodrive_bridge.py` untouched.
- `main()` = `rclpy.init()`, construct node, `rclpy.spin()`, destroy node, `rclpy.shutdown()`,
  Ctrl-C handled.
- Publishing matches `teleop_keyboard.py`: `QoSProfile(depth=1)`, one reused `Float32` per
  topic, zeros published on exit.
- Scan indices come from the message's `angle_min` / `angle_increment`; inf/NaN/out-of-range
  beams become `range_max`.
- No simulator, Docker image, `build/`, `install/`, `log/` in git (`.gitignore` allowlist).

## Steps
1. Recon: bridge topics (it publishes odom, lap count/time, best lap, collision count — all
   used), teleop QoS, Dockerfile. Done.
2. Node: two-wall lookahead centering + one-wall fallback + PD steering + front-clearance
   speed schedule closed on odom speed + safety brake / stall recovery / scan watchdog.
   All gains ROS parameters. Done.
3. Tooling: container run/drive scripts, simulator launcher with automatic Connect click,
   trial runner, telemetry monitor, recorder, offline replay, track map. Done.
4. Tune in the GUI simulator (the headless one has bursty frame pacing): fix weaving
   (derivative term was acting as yaw-rate feedback), rescale for the 1.2–1.5 m track, then
   raise speed until collisions appear and back off. Done — see `TUNING.md`.
5. Validate the defaults over long runs, document, push. Done.

## What was deliberately left out
- Gap following / disparity extender: the chicane is the limiting section and a gap follower
  would corner faster, but the brief asked for wall following.
- Speed-dependent gain scheduling beyond the lookahead: the fixed `kp` with `L = 0.6 + 0.2·v`
  stays damped up to ~4.5 m/s; faster needs latency compensation, not more gain.
- A racing-line or map: LiDAR only, no localisation.
