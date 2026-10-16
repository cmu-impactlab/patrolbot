#!/bin/bash
# Run the Pi-side ROS pytest suites inside the test image.
# Usage: run-ros-tests.sh [extra pytest paths/args...]
# With no arguments the full bridge + dock-manager suites run.
set -u
source /opt/ros/jazzy/setup.bash
source /opt/patrolbot/install/setup.bash
cd /opt/patrolbot
if [ "$#" -gt 0 ]; then
    exec python3 -B -m pytest -q -p no:cacheprovider "$@"
fi
exec python3 -B -m pytest -q -p no:cacheprovider \
    ros2_ws/src/patrolbot_bridge/test \
    ros2_ws/src/patrolbot_dock_manager/test
