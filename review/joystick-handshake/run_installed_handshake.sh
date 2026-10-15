#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /opt/patrolbot_nav2/install/setup.bash
source /opt/undock_test_ws/install/setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
export HANDSHAKE_INSTALLED=1
export PATROLBOT_TEST_INSTALLED=1
cd /candidate
python3 -B verify_handshake_installed.py
set +e
python3 -B test_manual_handshake.py
contracts=$?
python3 -B evidence-recovered/test_recovered_ownership.py
ownership=$?
python3 -B test_handshake_graph.py
graph=$?
printf 'installed_exit_codes contracts=%s ownership=%s graph=%s\n' "$contracts" "$ownership" "$graph"
test "$contracts" = 0 && test "$ownership" = 0 && test "$graph" = 0
