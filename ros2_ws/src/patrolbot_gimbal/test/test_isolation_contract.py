"""Isolation contract: the gimbal cannot affect robot motion.

Static assertions over source and configuration, in the style of
patrolbot_navigation's test_safety_contract.py. These run without ROS and
without hardware, so they gate every change to this package.

The guarantee being protected: this package reads /joy and drives a serial
gimbal controller. It must never publish anything that reaches twist_mux, the
velocity smoothers, or the bridge, and it must never claim a controller input
that the drive teleop depends on.

Written before the nodes existed, deliberately. A containment rule added after
the code it constrains has already been violated once.
"""

import re
import sys
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_DIR = PACKAGE_ROOT / 'patrolbot_gimbal'
WORKSPACE_SRC = PACKAGE_ROOT.parent
NAV_PACKAGE = WORKSPACE_SRC / 'patrolbot_navigation'
DRIVE_TELEOP = NAV_PACKAGE / 'scripts' / 'patrolbot_joy_teleop.py'
NAV_BRINGUP = NAV_PACKAGE / 'launch' / 'bringup.launch.py'

sys.path.insert(0, str(PACKAGE_ROOT))


def _source_files():
    """Every Python source file in this package, excluding its tests."""
    return sorted(SOURCE_DIR.rglob('*.py'))


def _strip_comments_and_docstrings(text):
    """Crude but adequate: drop # comments and triple-quoted blocks.

    Keeps the forbidden-token scan from tripping over prose that legitimately
    mentions cmd_vel while explaining why this package never touches it.
    """
    text = re.sub(r'"""..*?"""', '', text, flags=re.S)
    text = re.sub(r"'''.*?'''", '', text, flags=re.S)
    text = re.sub(r'#[^\n]*', '', text)
    return text


# ------------------------------------------------ no path to robot motion --

FORBIDDEN = [
    'cmd_vel',
    'twist_mux',
    'input/navi',
    'input/teleop',
    'input/safety_controller',
    'input/switch',
    'input/docking',
    'Twist',
]


@pytest.mark.parametrize('token', FORBIDDEN)
def test_no_source_file_references_the_velocity_chain(token):
    offenders = []
    for path in _source_files():
        body = _strip_comments_and_docstrings(path.read_text(encoding='utf-8'))
        if token in body:
            offenders.append(path.name)
    assert not offenders, (
        f'{token!r} appears in {offenders}. This package must not be able to '
        'reach the velocity chain.')


def test_package_declares_no_geometry_msgs_dependency():
    """No geometry_msgs means Twist cannot even be constructed."""
    manifest = (PACKAGE_ROOT / 'package.xml').read_text(encoding='utf-8')
    assert 'geometry_msgs' not in manifest


# ------------------------------------------------------------ namespacing --

TOPIC_RE = re.compile(r"""create_(?:publisher|subscription)\s*\(\s*[^,]+,\s*['"]([^'"]+)['"]""")

ALLOWED_UNPREFIXED = {'/joy', 'joy', '/diagnostics', 'diagnostics'}


def test_every_topic_is_gimbal_namespaced():
    """Anything outside /gimbal/ is either /joy (read) or /diagnostics."""
    bad = []
    for path in _source_files():
        for topic in TOPIC_RE.findall(path.read_text(encoding='utf-8')):
            if topic in ALLOWED_UNPREFIXED:
                continue
            if topic.startswith('gimbal/') or topic.startswith('/gimbal/'):
                continue
            bad.append((path.name, topic))
    assert not bad, f'topics outside the gimbal namespace: {bad}'


def test_joy_is_only_ever_subscribed_never_published():
    """Publishing /joy would let the gimbal forge drive commands."""
    bad = []
    for path in _source_files():
        text = path.read_text(encoding='utf-8')
        for match in re.finditer(
                r"create_publisher\s*\(\s*[^,]+,\s*['\"]([^'\"]+)['\"]", text):
            if match.group(1).lstrip('/') == 'joy':
                bad.append(path.name)
    assert not bad, f'{bad} publishes /joy'


# ------------------------------------------- controller input disjointness --

INDEX_RE = re.compile(r'^\s*(AX_[A-Z_]+|BTN_[A-Z_]+)\s*=\s*(\d+)', re.M)


def _indices(path):
    """Extract AX_*/BTN_* index constants from a source file."""
    axes, buttons = set(), set()
    for name, value in INDEX_RE.findall(path.read_text(encoding='utf-8')):
        (axes if name.startswith('AX_') else buttons).add(int(value))
    return axes, buttons


@pytest.mark.skipif(not DRIVE_TELEOP.exists(),
                    reason='drive teleop not present in this checkout')
def test_gimbal_controller_inputs_do_not_collide_with_drive_teleop():
    """Parsed from both live sources, so a future remap breaks this test.

    This is the point of the contract: the two nodes share one physical
    controller, and the only thing stopping them fighting over it is that
    their index sets are disjoint. Asserting it against the real drive teleop
    source means the guarantee survives someone editing either file.
    """
    from patrolbot_gimbal import gimbal_policy  # noqa: F401

    policy_path = SOURCE_DIR / 'gimbal_policy.py'
    gimbal_axes, gimbal_buttons = _indices(policy_path)
    drive_axes, drive_buttons = _indices(DRIVE_TELEOP)

    assert gimbal_axes, 'no gimbal axis constants found'
    assert drive_axes, 'no drive axis constants found'

    axis_overlap = gimbal_axes & drive_axes
    button_overlap = gimbal_buttons & drive_buttons

    assert not axis_overlap, (
        f'gimbal and drive teleop both claim joystick axes {sorted(axis_overlap)}')
    assert not button_overlap, (
        f'gimbal and drive teleop both claim buttons {sorted(button_overlap)}')


# ------------------------------------------------------ launch containment --

@pytest.mark.skipif(not NAV_BRINGUP.exists(),
                    reason='nav bringup not present in this checkout')
def test_gimbal_is_not_started_by_navigation_bringup():
    """The gimbal must never come up with the navigation stack."""
    text = NAV_BRINGUP.read_text(encoding='utf-8')
    assert 'patrolbot_gimbal' not in text
    assert 'gimbal' not in text.lower()


def test_gimbal_launch_starts_no_navigation_or_base_nodes():
    launch = PACKAGE_ROOT / 'launch' / 'gimbal.launch.py'
    if not launch.exists():
        pytest.skip('launch file arrives in phase 6')
    text = launch.read_text(encoding='utf-8')
    for token in ('nav2', 'twist_mux', 'velocity_smoother', 'controller_server',
                  'patrolbot_bridge', 'component_container'):
        assert token not in text, f'{token} must not be launched by the gimbal'
