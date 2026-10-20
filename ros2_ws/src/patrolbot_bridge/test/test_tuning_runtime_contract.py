"""Static/unit agreement for the paired runtime interface, without a ROS graph."""

import json
from pathlib import Path
import re

from patrolbot_bridge.tuning_backend import MANAGERS, NODES


ROOT = Path(__file__).resolve().parents[4]
CONTRACT = json.loads((ROOT / 'contracts/robot-tuning-runtime.json').read_text())


def test_fixed_nodes_and_managers_match_the_shared_runtime_contract():
    assert list(NODES) == CONTRACT['nodes']
    assert list(MANAGERS) == CONTRACT['managers']


def test_real_core_and_physical_input_sources_use_the_shared_topics():
    bridge = (ROOT / 'ros2_ws/src/patrolbot_bridge/patrolbot_bridge/bridge_node.py').read_text()
    joystick = (ROOT / 'ros2_ws/src/patrolbot_navigation/scripts'
                / 'patrolbot_joy_teleop.py').read_text()
    for topic in CONTRACT['topics'].values():
        assert topic in bridge
    assert CONTRACT['services']['physical_idle'] in joystick
    assert CONTRACT['topics']['hold'] in joystick


def test_navigation_admission_patch_owns_one_atomic_hold_and_goal_mutex():
    patch = (ROOT / 'docker/nav2-patches/0018-tuning-navigation-admission.patch').read_text()
    assert 'bool setTuningHold(bool held)' in patch
    assert 'bool tryStartNavigating' in patch
    assert 'tuning_hold_ || !current_navigator_.empty()' in patch
    assert 'if (!plugin_muxer_->tryStartNavigating(getName()))' in patch
    # Reject malformed unified diffs without compiling/running any ROS code.
    lines = patch.splitlines()
    for i, line in enumerate(lines):
        if not line.startswith('@@ '):
            continue
        match = re.match(r'@@ -\d+,(\d+) \+\d+,(\d+) @@', line)
        assert match is not None
        body = []
        for following in lines[i + 1:]:
            if following.startswith(('@@ ', '--- ', '+++ ')):
                break
            body.append(following)
        assert sum(s.startswith((' ', '-')) for s in body) == int(match[1])
        assert sum(s.startswith((' ', '+')) for s in body) == int(match[2])
