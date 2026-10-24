"""Shipped callbacks require actual fresh sole-publisher samples, not local zero calls."""

import ast
import math
from pathlib import Path
from types import MethodType, SimpleNamespace as NS

from patrolbot_bridge.tuning_runtime import TuningObservations
import pytest


SOURCE = Path(__file__).resolve().parents[1] / 'patrolbot_bridge/bridge_node.py'


def setup():
    tree = ast.parse(SOURCE.read_text())
    klass = next(node for node in tree.body if isinstance(node, ast.ClassDef)
                 and node.name == 'PatrolBotBridge')
    methods = [node for node in klass.body if isinstance(node, ast.FunctionDef)
               and node.name in ('_tuning_source_is_current', '_tuning_velocity_observed',
                                 '_tuning_deadman_observed')]
    namespace = {'math': math}
    exec(compile(ast.fix_missing_locations(ast.Module(body=methods, type_ignores=[])),
                 str(SOURCE), 'exec'), namespace)
    endpoint = NS(node_name='teleop_velocity_smoother', node_namespace='/',
                  topic_type='geometry_msgs/msg/Twist', endpoint_gid=[1, 2, 3])
    node = NS(_tuning_source_stamps={}, _tuning_observations=TuningObservations(lambda: 10.),
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000)),
              get_publishers_info_by_topic=lambda _: [endpoint])
    for method in methods:
        setattr(node, method.name, MethodType(namespace[method.name], node))
    message = NS(linear=NS(x=0., y=0., z=0.), angular=NS(x=0., y=0., z=0.))
    info = NS(publisher_gid=[1, 2, 3], source_timestamp=10_000_000_000)
    return node, endpoint, message, info


def test_internal_zero_is_never_recorded_as_a_final_producer_observation():
    node, _, msg, _ = setup()
    node._tuning_velocity_observed('/cmd_vel', 'teleop_velocity_smoother', msg, None)
    assert node._tuning_observations.velocities == {}


def test_dictionary_metadata_keeps_exact_publisher_and_source_timestamp_binding():
    node, _, msg, info = setup()
    metadata = vars(info).copy()
    node._tuning_velocity_observed('/cmd_vel', 'teleop_velocity_smoother', msg, metadata)
    assert node._tuning_observations.velocities['/cmd_vel'][0]
    # Older rclpy builds omit source GID. A sole graph endpoint is not a
    # substitute for binding this received sample; remain unavailable.
    del metadata['publisher_gid']
    node._tuning_velocity_observed('/cmd_vel', 'teleop_velocity_smoother', msg, metadata)
    assert '/cmd_vel' not in node._tuning_observations.velocities


@pytest.mark.parametrize('fault', [
    'gid', 'namespace', 'name', 'type', 'stale', 'future', 'replay',
])
def test_invalid_or_replayed_source_cannot_preserve_prior_zero_evidence(fault):
    node, endpoint, msg, info = setup()
    node._tuning_velocity_observed('/cmd_vel', 'teleop_velocity_smoother', msg, info)
    assert node._tuning_observations.velocities['/cmd_vel'][0]
    if fault == 'gid':
        info.publisher_gid = [4, 5, 6]
    elif fault == 'namespace':
        endpoint.node_namespace = '/other'
    elif fault == 'name':
        endpoint.node_name = 'forged'
    elif fault == 'type':
        endpoint.topic_type = 'not-a-twist'
    elif fault == 'stale':
        info.source_timestamp = 1
    elif fault == 'future':
        info.source_timestamp += 1
    node._tuning_velocity_observed('/cmd_vel', 'teleop_velocity_smoother', msg, info)
    assert '/cmd_vel' not in node._tuning_observations.velocities


def test_nonplanar_values_never_prove_zero_command_buffers():
    node, _, msg, info = setup()
    msg.angular.x = .1
    node._tuning_velocity_observed('/cmd_vel', 'teleop_velocity_smoother', msg, info)
    assert node._tuning_observations.velocities['/cmd_vel'][0] is False
