#!/usr/bin/env python3
"""Nonmoving fixture process for isolated Docker replacement-boot acceptance."""

import json
import os
from pathlib import Path
import time

from patrolbot_bridge.tuning_run import prepare
from patrolbot_bridge.tuning_yaml import strict_yaml_load

if os.environ.get('PATROLBOT_ISOLATED_TEST') != '1':
    raise SystemExit('isolated fixture only')

guard, command = prepare('navigation')
assert command == ['ros2', 'launch', 'patrolbot_navigation', 'bringup.launch.py']
path = os.environ['PATROLBOT_NAVIGATION_PARAMS_FILE']
values = strict_yaml_load(Path(path).read_text())['amcl']['ros__parameters']
record = {'state': 'candidate' if Path(path).name.startswith('patrolbot-nav2-boot-') else 'active',
          'pf_z': values['pf_z'], 'pid': os.getpid()}
temporary = Path('/fixture-output/.boot.json')
temporary.write_text(json.dumps(record))
temporary.replace('/fixture-output/boot.json')
print(json.dumps(record), flush=True)
# No ROS graph, velocity publisher, hardware peer, pose, or device is created.
while True:
    time.sleep(1.)
