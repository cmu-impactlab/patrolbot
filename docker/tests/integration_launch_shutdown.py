#!/usr/bin/env python3
"""Exercise actual launch/ROS adapter cleanup on expected and unexpected exits."""
import importlib.util
import os
from pathlib import Path
import subprocess
import sys

assert os.environ.get('PATROLBOT_ISOLATED_TEST') == '1'
assert {p.name for p in Path('/sys/class/net').iterdir()} == {'lo'}

if len(sys.argv) == 1:
    for mode in ('container_exit', 'operator_shutdown'):
        subprocess.run([sys.executable, __file__, mode], check=True, timeout=20)
    print('PASS: container exit and operator shutdown both clean up ROS launch')
    sys.exit(0)

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.ros_adapters import get_ros_adapter

path = Path(get_package_share_directory('patrolbot_navigation')) / 'launch/bringup.launch.py'
spec = importlib.util.spec_from_file_location('patrolbot_bringup', path)
bringup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bringup)
service = LaunchService()
get_ros_adapter(service.context)
process = ExecuteProcess(cmd=[sys.executable, '-c',
    'import signal, sys, time; '
    'signal.signal(signal.SIGINT, lambda *_: sys.exit(0)); time.sleep(1)'])
actions = [RegisterEventHandler(OnProcessExit(
    target_action=process, on_exit=bringup.shutdown_on_container_exit)), process]
if sys.argv[1] == 'operator_shutdown':
    actions.append(TimerAction(period=0.2, actions=[EmitEvent(event=Shutdown())]))
service.include_launch_description(LaunchDescription(actions))
assert service.run() == 0, 'launch failed during ' + sys.argv[1]
assert service.context.is_shutdown, 'container exit failed to shut down launch'
