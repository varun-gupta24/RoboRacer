# Penn RoboRacer Technical Challenge — LiDAR wall following

ROS 2 (Humble, Python) node that drives the AutoDRIVE RoboRacer around the practice
track using only the 2D LiDAR. Goal: 3 consecutive laps with zero wall contact, then the
best lap time.

Submission package: `autodrive_devkit/` (unchanged AutoDRIVE devkit plus
`autodrive_roboracer/wall_follow_node.py` and one `console_scripts` line in `setup.py`).
`scripts/` holds the tooling used to run and tune it; `PLAN.md` and `TUNING.md` document the
approach and the tuning log. AI assistance (Claude Code) was used for the whole build.

## How it drives

`wall_follow_node` subscribes to `/autodrive/roboracer_1/lidar` and publishes
`throttle_command` / `steering_command` (`std_msgs/Float32`, both in [-1, 1]) with the same
QoS and message conventions as `teleop_keyboard.py`.

1. **Two-wall centering.** For each side it samples beam *b* at ±90° and beam *a* 50° ahead of
   it (median over ±2 beams), gets the wall angle `phi = atan2(a cos α − b, a sin α)`, the
   perpendicular distance `D = b cos phi` and the lookahead projection `D_L = D + L sin phi`
   with `L = 0.6 m + 0.2 s · speed`. Error = `D_L(left) − D_L(right)`.
2. **One-wall fallback.** If a side beam reads beyond 2.5 m the wall is treated as missing
   and the car keeps the running estimate of half the track width from the other wall:
   pushing away from it at full gain, pulling toward it at 0.3× (at a corner exit the
   remaining wall is the one ahead, so pulling toward it is what put the car into it).
3. **Steering.** PD on the error (`kp = 0.4 rad/m`, `kd = 0.02`), plus a small repulsion
   term when a front-diagonal cone (45° ± 20°) sees something closer than 0.7 m. The
   steering angle is divided by 0.52 rad (full lock) and clipped to [-1, 1].
4. **Speed.** Constant-deceleration law: `v_target = sqrt(2·a·(front − 0.5 m − v·0.1 s))`
   with `a = 3 m/s²` and `front` the min range in a ±15° cone (low-pass filtered), clipped to
   [1, 4] m/s and scaled by `1 − 0.5·|steer|`. Throttle is closed-loop on the signed forward
   speed from `/odom`, predicted 0.25 s ahead: `0.1·v_target + 0.8·(v_target − v_pred)`.
5. **Safety.** Front clearance under 0.5 m brakes; a stall against a wall backs up for 1 s with
   opposite steering; losing scans for 0.5 s zeroes the throttle; Ctrl-C publishes zeros, then
   destroys the node and shuts rclpy down.

Every gain is a ROS parameter, changeable live:
`ros2 param set /wall_follow_node v_max 4.5`. Lap and collision events are logged from the
bridge's `lap_count`, `last_lap_time`, `best_lap_time` and `collision_count` topics.

## Results (practice track, GUI simulator, Apple-silicon Docker)

| setting | window | full laps | best lap | mean lap | collisions |
|---|---|---|---|---|---|
| defaults | 150 s | 10 | 12.8 s | 13.9 s | 0 |
| defaults | 300 s | 21 | 12.5 s | 13.5 s | 1 (lap 15) |
| `brake_accel 3`, no latency allowance | 2 × 130 s | 17 | 11.7 s | 12.4 s | 1 (first lap) |
| linear speed law, `v_max 5` | 65 s | 5 | 10.9 s | 12.1 s | 1 |

Faster settings lap in 10–11 s but touch a wall every few laps at the S-bend; the defaults are
the setting that reliably strings clean laps together. For a guaranteed-clean run at the cost of
~2 s per lap: `scripts/drive.sh --ros-args -p brake_accel:=2.0 -p v_max:=3.5`.
See `TUNING.md` for the log.

## Run it

Prerequisites: Docker Desktop, the AutoDRIVE practice simulator at
`~/autodrive-sim/autodrive_simulator/AutoDRIVE Simulator.app`.

```bash
scripts/run.sh            # builds the devkit image once, colcon-builds the package, runs the bridge on :4567
scripts/sim.sh            # (another terminal) launches the simulator and clicks Connect
scripts/drive.sh          # (another terminal) runs the wall follower
```

`scripts/drive.sh --ros-args -p v_max:=3.5` overrides parameters at launch. Without the
scripts: build the image from `autodrive_devkit.Dockerfile`, run it with `-p 4567:4567`,
`ros2 launch autodrive_roboracer bringup_headless.launch.py`, connect the simulator, then
`ros2 run autodrive_roboracer wall_follow_node`.

Notes
- Restarting the bridge while the simulator is connected puts the simulator into a
  reconnect storm; restart the simulator too (`scripts/sim.sh` does that).
- `scripts/sim.sh headless` runs the simulator with `-batchmode -nographics`; it auto-connects,
  but its frame pacing is bursty (0.2–0.6 s scan gaps) and lap times and collisions there are
  not representative. Tune in GUI mode.

## Tuning tools (`scripts/`)

| script | purpose |
|---|---|
| `dk.sh CMD` | run a command inside the devkit container with ROS sourced |
| `trial.py T k=v ...` | set parameters, reset the sim, watch T seconds, print laps/collisions, append to `trials.csv` |
| `monitor.py [period] [dur]` | live telemetry: speed, lap, collisions, commands, clearances |
| `record.py T out.npz` | record scans, odom and commands |
| `replay.py rec.npz i n [k=v]` | run the controller offline over a recording and print the steering terms |
| `map.py rec.npz out.png` | draw the track from the recording with the trajectory and collisions |

`autodrive_devkit/test/test_wall_follow.py` checks the wall geometry
(`scripts/dk.sh python3 -m pytest src/autodrive_devkit/test/test_wall_follow.py`).
