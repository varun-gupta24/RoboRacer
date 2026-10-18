# Tuning log

All runs: practice track, simulator in GUI mode on the host (macOS, Apple silicon), bridge and
node in the `ros:humble` arm64 container, scans arriving at ~28 Hz. `scripts/trial.py` sets
parameters, resets the simulator, watches, and appends one line to `scripts/trials.csv`.
Lap 1 after a reset is a partial lap and is excluded from best/mean.

## Track
Closed loop about 10 m × 18 m, 1.2–1.5 m wide, two long straights, two 180° ends and an S-bend
at the waist of the left side (the inner wall juts out). `scripts/map.py` renders it from a
recording. The S-bend is where every fast run eventually touches.

## Findings, in order

1. **Headless simulator is not representative.** `-batchmode -nographics` auto-connects but
   delivers scans in 100 Hz bursts with 0.2–0.6 s silences; the car covers up to 1.8 m blind.
   The GUI simulator paces at ~28 Hz with a 0.19 s worst gap. All tuning below is GUI mode.
2. **Weaving came from the derivative term.** Offline replay (`scripts/replay.py`) of a
   recording showed the D term 3–5× the P term. With the lookahead projection the error already
   contains heading, so its derivative is yaw rate; with ~100 ms loop latency that is
   destabilising. `kd` 0.12 → 0.02. For a bicycle model the lookahead gives damping ratio
   `ζ = L·sqrt(kp / (2·wheelbase))`, so the lookahead, not `kd`, is the damping knob:
   `kp 0.4`, `L = 0.6 + 0.2·v` gives ζ ≈ 0.9 at 3 m/s.
3. **Scale.** Defaults written for a ~2 m track were wrong for 1.3 m: `half_width_init 0.75`,
   `open_dist 2.5`, `brake_dist 0.5`.
4. **Stalls.** The brake alone parked the car nose-to-wall; the 1 s reverse-with-opposite-lock
   recovery fixed that. Not needed at the final settings but kept.
5. **Speed.** The speed schedule only sees the ±cone ahead, so bends are entered at straight-line
   speed and braking authority decides the outcome. `throttle_kp` 0.2 → 0.8 and a ±15° cone made
   the biggest reliability difference. An "in-path clearance" variant (nearest point within the
   car's swept width) was tried and reverted: in a bend the outer wall sits inside the band and
   the car braked and jerked through every corner.
6. **Reset artefact.** Resetting while the car is moving sometimes registers one collision in the
   first 2 s. Start the node with the car at rest. The reset teleports the car to the start line
   on the right straight heading south about 1.5 s after the command and zeroes the counters.
7. **Speed law shape.** `v = k·d` demands deceleration `k²·d`, which at `k = 1.3` is 5 m/s² at
   3 m of clearance — the car's whole braking capability (measured: throttle −1 gives about
   −5 m/s², +1 about +5 m/s², coasting −0.9 m/s²). A constant-deceleration law
   `v = sqrt(2·a·(d − d0 − v·τ))` with `a = 3`, `τ = 0.1 s` is faster far from walls and safer near
   them: 12–13 s laps instead of 14–15 s, fewer hits. `τ = 0.25` cost 2 s per lap for no gain.
8. **One-wall fallback pulled the car into corner-exit walls.** When the inner wall ends at a
   corner exit the controller drops to one-wall mode, and "follow the remaining wall at half
   width" steers toward the outer wall that is directly ahead. Now it pushes away at full gain
   and pulls toward at 0.3×.
9. **Reverse runaway (real bug).** Speed was `hypot(vx, vy)`; after a near-stop at an apex the
   brake branch (`front < 0.5 m and speed > 0.3`) kept commanding −0.3 while the car was
   accelerating *backwards*, until it hit something behind. Odometry twist is body-frame, so the
   signed `vx` is used now and reversing only happens through the 1 s stall-recovery branch.
10. **Scan gaps remain the limiting factor.** Even in GUI mode ~50 gaps of 0.15–0.35 s per
    150 s; the bridge is a single Python thread at ~110 % CPU decoding the camera image for every
    frame. The latency allowance in the speed law is what covers these.

## Trials (from `scripts/trials.csv`)

| parameters (rest default) | window | full laps | best | mean | collisions |
|---|---|---|---|---|---|
| kp 0.6 kd 0.12 (first gains) v_max 3 | 70 s | 2 | 17.5 | 19.0 | 0 |
| kp 0.4 kd 0.02 L 0.6+0.2v, v_max 3 | 70 s | 3 | 14.4 | 14.9 | 0 |
| v_max 4 v_per_m_front 1.5 | 65 s | 4 | 10.2 | 11.3 | 4 |
| v_max 5 v_per_m_front 1.5 throttle_kp 0.4 | 65 s | 5 | 10.9 | 12.1 | 1 |
| v_max 4 v_per_m_front 1.2 throttle_kp 0.4 | 120 s | 7 | 12.9 | 14.8 | 2 |
| v_max 4.5 v_per_m_front 1.3 throttle_kp 0.4 steer_slowdown 0.6 | 120 s | 7 | 13.0 | 14.1 | 3 |
| v_max 3.5 v_per_m_front 1.3 throttle_kp 0.8 cone ±10° | 130 s | 8 | 13.8 | 14.4 | 1 (t=9 s) |
| v_max 4 v_per_m_front 1.3 throttle_kp 0.8 cone ±15° | 130 s | 7 | 14.4 | 15.5 | 0 |
| same, 200 s validation | 200 s | 12 | 13.6 | 15.0 | 1 |
| sqrt law brake_accel 3 (+ speed prediction) | 2 × 130 s | 8 + 9 | 11.7 | 12.4 | 1 (first lap) |
| + open_pull 0.3 | 2 × 130 s | 5 + 8 | 12.4 | 13.1 | 4 |
| + react_time 0.25 | 2 × 150 s | 8 + 8 | 14.8 | 16.2 | 1 |
| + signed speed, react_time 0.1 | 150 s | 9 | 12.8 | 13.9 | 0 |
| + signed speed, react_time 0.25 | 150 s | 8 | 14.5 | 15.7 | 1 |
| **defaults (signed speed, sqrt law a=3, react 0.1, open_pull 0.3)** | 300 s | 21 | 12.5 | 13.5 | 1 (lap 15; 13 clean laps before it) |

## Going faster
Lap time is bounded by the S-bend entry speed, not by the straights (the car reaches ~5 m/s
there already). The levers, in order of expected payoff: brake earlier for bends using a
wider clearance cone only while the two walls diverge; a disparity-extended gap follower for
the S-bend; and reducing control latency (the bridge is single-threaded Python at ~100 % CPU).
