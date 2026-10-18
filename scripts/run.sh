#!/bin/bash
# Start the devkit container: build the package (source mounted, not copied) and run the
# bridge on :4567 in the foreground. Then connect the simulator and run scripts/drive.sh.
set -e
cd "$(dirname "$0")/.."
docker image inspect autodrive_devkit >/dev/null 2>&1 || docker build -f autodrive_devkit.Dockerfile -t autodrive_devkit .
[ -t 0 ] && TTY=-it || TTY=-i
exec docker run --rm $TTY --name autodrive_devkit -p 4567:4567 \
  -v "$PWD/autodrive_devkit:/home/autodrive_devkit/src/autodrive_devkit" \
  -v "$PWD/scripts:/home/scripts" \
  --entrypoint bash autodrive_devkit -c "
    source /opt/ros/humble/setup.bash && cd /home/autodrive_devkit &&
    colcon build --symlink-install >/dev/null && source install/setup.bash &&
    exec ros2 launch autodrive_roboracer bringup_headless.launch.py"
