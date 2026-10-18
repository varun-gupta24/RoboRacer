#!/bin/bash
# Run the wall follower in the running devkit container. Extra args go to the node, e.g.
#   scripts/drive.sh --ros-args -p kp:=1.0 -p v_max:=4.0
# Ctrl-C stops it (throttle is zeroed on exit).
exec "$(dirname "$0")/dk.sh" ros2 run autodrive_roboracer wall_follow_node "$@"
