"""Strict YAML loader: unsafe input fails closed. No ROS, no network."""

from patrolbot_bridge.tuning_yaml import (
    strict_yaml_dump,
    strict_yaml_load,
    strict_yaml_load_file,
    StrictYAMLError,
)
import pytest


def test_round_trip_is_deterministic():
    data = {'b': 2, 'a': {'z': [1, 0.5, True, None, 'x']}}
    text = strict_yaml_dump(data)
    assert strict_yaml_load(text) == data
    assert text == strict_yaml_dump(strict_yaml_load(text))


def test_duplicate_key_rejected():
    with pytest.raises(StrictYAMLError, match='duplicate'):
        strict_yaml_load('a: 1\na: 2\n')


def test_anchor_and_alias_rejected():
    with pytest.raises(StrictYAMLError):
        strict_yaml_load('a: &x 1\nb: *x\n')


def test_merge_key_bomb_rejected():
    text = 'base: &b {a: 1, b: 2}\nchild:\n  <<: *b\n'
    with pytest.raises(StrictYAMLError):
        strict_yaml_load(text)


def test_non_string_key_rejected():
    with pytest.raises(StrictYAMLError, match='string'):
        strict_yaml_load('1: a\n')


def test_non_finite_float_rejected():
    for literal in ('a: .inf\n', 'a: -.inf\n', 'a: .nan\n'):
        with pytest.raises(StrictYAMLError):
            strict_yaml_load(literal)


def test_custom_tag_rejected():
    with pytest.raises(StrictYAMLError):
        strict_yaml_load('a: !!python/object/apply:os.system ["true"]\n')


def test_byte_limit_enforced():
    with pytest.raises(StrictYAMLError, match='byte'):
        strict_yaml_load('a: ' + 'x' * 100, max_bytes=32)


def test_depth_limit_enforced():
    text = 'a:\n' + ''.join('  ' * i + 'b:\n' for i in range(1, 40)) + '  ' * 40 + 'x: 1\n'
    with pytest.raises(StrictYAMLError):
        strict_yaml_load(text)


def test_nul_byte_rejected():
    with pytest.raises(StrictYAMLError):
        strict_yaml_load('a: 1\n\x00')


def test_bytes_input_rejected():
    with pytest.raises(StrictYAMLError, match='text'):
        strict_yaml_load(b'a: 1\n')


def test_scalar_length_bound():
    with pytest.raises(StrictYAMLError, match='too long'):
        strict_yaml_load('a: "' + 'x' * 5000 + '"\n')


def test_load_file_refuses_symlink(tmp_path):
    target = tmp_path / 'real.yaml'
    target.write_text('a: 1\n')
    link = tmp_path / 'link.yaml'
    link.symlink_to(target)
    with pytest.raises(StrictYAMLError):
        strict_yaml_load_file(link)
