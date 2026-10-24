"""Unit checks for authenticated, bounded, robot-specific tuning mutations."""

import copy

from patrolbot_bridge.tuning_authorization import AuthorizationError, load_key, sign, verify
import pytest


KEY = b'unit-test-secret-not-for-deployment-123456'
WRAPPER = {'actor': {'id': 1, 'username': 'admin', 'role': 'administrator'},
           'request': {'operation': 'save', 'request_id': 'fixture',
                       'changes': [{'id': 'amcl.alpha1', 'value': .1}]}}


def test_signed_mutation_round_trip_is_detached_and_robot_bound():
    original = copy.deepcopy(WRAPPER)
    signed = sign(original, KEY, 'robot-1', now=100)
    assert signed['authorization']['signature'] == (
        'a4e0578045fdf029613dc151caef9c0365c4000ea1576ed19563be8faa6745ca')
    assert verify(signed, KEY, 'robot-1', now=101) == WRAPPER
    original['request']['operation'] = 'restore'
    assert verify(signed, KEY, 'robot-1', now=101) == WRAPPER
    with pytest.raises(AuthorizationError):
        verify(signed, KEY, 'robot-2', now=101)


@pytest.mark.parametrize('part', ['actor', 'request', 'authorization'])
def test_tampering_any_signed_claim_is_refused(part):
    signed = sign(WRAPPER, KEY, 'robot-1', now=100)
    if part == 'actor':
        signed['actor']['id'] = 2
    elif part == 'request':
        signed['request']['changes'][0]['value'] = .9
    else:
        signed['authorization']['issued_at_ms'] += 1
    with pytest.raises(AuthorizationError):
        verify(signed, KEY, 'robot-1', now=101)


@pytest.mark.parametrize('now', [130, 200, 90])
def test_expired_or_future_authorization_is_refused(now):
    with pytest.raises(AuthorizationError):
        verify(sign(WRAPPER, KEY, 'robot-1', now=100), KEY, 'robot-1', now=now)


@pytest.mark.parametrize('key', [None, b'', b'short', 'not-bytes'])
def test_no_default_or_weak_key_can_authorize_a_mutation(key):
    with pytest.raises(AuthorizationError):
        sign(WRAPPER, key, 'robot-1')


def test_unsigned_admin_claim_and_wrong_key_are_refused():
    with pytest.raises(AuthorizationError):
        verify(WRAPPER, KEY, 'robot-1')
    with pytest.raises(AuthorizationError):
        verify(sign(WRAPPER, KEY, 'robot-1'), b'x' * 32, 'robot-1')


def test_private_key_file_permissions_and_symlinks(tmp_path):
    path = tmp_path / 'key'
    path.write_bytes(KEY)
    path.chmod(0o600)
    assert load_key(path) == KEY
    path.chmod(0o644)
    with pytest.raises(AuthorizationError):
        load_key(path)
    link = tmp_path / 'link'
    link.symlink_to(path)
    with pytest.raises((AuthorizationError, OSError)):
        load_key(link)


def test_roles_and_nonfinite_payloads_are_never_signed():
    body = copy.deepcopy(WRAPPER)
    body['actor']['role'] = 'operator'
    with pytest.raises(AuthorizationError):
        sign(body, KEY, 'robot-1')
    body = copy.deepcopy(WRAPPER)
    body['request']['changes'][0]['value'] = float('nan')
    with pytest.raises(AuthorizationError):
        sign(body, KEY, 'robot-1')


@pytest.mark.parametrize('part', ['digest', 'plan_hash'])
def test_reviewed_revision_and_scope_are_covered_by_the_signature(part):
    body = copy.deepcopy(WRAPPER)
    body['request'] = {'operation': 'apply', 'request_id': 'fixture', 'expected_revision': 2,
                       'review': {'digest': 'sha256:' + 'a' * 64, 'plan_hash': 'sha256:' + 'b' * 64}}
    signed = sign(body, KEY, 'robot-1', now=100)
    signed['request']['review'][part] = 'sha256:' + 'c' * 64
    with pytest.raises(AuthorizationError):
        verify(signed, KEY, 'robot-1', now=101)
