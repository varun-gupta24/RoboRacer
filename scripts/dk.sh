#!/bin/bash
# Run a command inside the running devkit container with ROS sourced, e.g.
#   scripts/dk.sh ros2 param set /wall_follow_node kp 1.0
#   scripts/dk.sh ros2 topic pub -1 /autodrive/reset_command std_msgs/msg/Bool "{data: true}"
exec docker exec -i autodrive_devkit bash -c 'source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash && "$@"' -- "$@"
