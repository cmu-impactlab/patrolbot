"""Jazzy dictionary and newer MessageInfo provenance boundaries."""

from types import SimpleNamespace as NS

from patrolbot_bridge.local_source import LocalSource
import pytest


@pytest.mark.parametrize('representation', ['jazzy-dict', 'object-with-gid'])
def test_stable_endpoint_fresh_samples_and_duplicate_rejection(representation):
    clock = NS(ns=1_000_000_000)
    endpoint = NS(node_name='joy_node', node_namespace='/', topic_type='sensor_msgs/msg/Joy',
                  endpoint_gid=bytes([1] * 16))
    endpoints = [endpoint]
    node = NS(get_publishers_info_by_topic=lambda _: endpoints,
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=clock.ns)))
    source = LocalSource('/joy', 'joy_node', 'sensor_msgs/msg/Joy', .4)

    def info(stamp=None, gid=None):
        values = {'source_timestamp': clock.ns if stamp is None else stamp}
        if representation == 'object-with-gid':
            values['publisher_gid'] = endpoint.endpoint_gid if gid is None else gid
            return NS(**values)
        return values
    assert not source.accepts(node, info())  # Must observe a stable endpoint first.
    clock.ns += 500_000_000
    assert source.accepts(node, info())
    assert not source.accepts(node, info())  # Duplicate sample timestamp.
    clock.ns += 50_000_000
    assert not source.accepts(node, info(stamp=clock.ns - 500_000_000))
    clock.ns += 500_000_000
    assert not source.accepts(node, info())  # Invalid evidence resets provenance.
    clock.ns += 500_000_000
    assert source.accepts(node, info())
    endpoints.append(endpoint)
    assert not source.accepts(node, info())  # Duplicate node names are duplicates too.
    endpoints.pop()
    clock.ns += 500_000_000
    assert not source.accepts(node, info())
    clock.ns += 500_000_000
    if representation == 'object-with-gid':
        assert not source.accepts(node, info(gid=bytes([2] * 16)))
    else:
        assert source.accepts(node, info())


def test_missing_metadata_and_wrong_endpoint_are_never_trusted():
    source = LocalSource('/joy', 'joy_node', 'sensor_msgs/msg/Joy', .4)
    node = NS(get_publishers_info_by_topic=lambda _: [])
    assert not source.accepts(node, {})
    node.get_publishers_info_by_topic = lambda _: [NS(node_name='foreign')]
    assert not source.accepts(node, {})
