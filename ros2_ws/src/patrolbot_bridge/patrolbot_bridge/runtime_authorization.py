"""Operator/admin runtime authorizations; distinct claims and HMAC purpose."""
import hashlib
import hmac
import json
import re

from .tuning_authorization import (AuthorizationError, load_key, _canonical, _key,
                                   _clock_ms, MAX_LIFETIME_MS, CLOCK_SKEW_MS)

DOMAIN = b'patrolbot-runtime-controls-authorization-v1\n'
OPERATIONS = frozenset({'set', 'reconcile'})


def _subject(wrapper, robot_id):
    if not isinstance(robot_id, str) or not 1 <= len(robot_id) <= 128:
        raise AuthorizationError('invalid robot authorization subject')
    if not isinstance(wrapper, dict) or not {'request', 'actor'} <= set(wrapper):
        raise AuthorizationError('invalid authorization envelope')
    actor, request = wrapper['actor'], wrapper['request']
    if (not isinstance(actor, dict) or actor.get('role') not in ('operator', 'administrator')
            or set(actor) != {'id', 'username', 'role'}
            or type(actor.get('id')) is not int
            or not isinstance(actor.get('username'), str) or not 1 <= len(actor['username']) <= 128
            or not isinstance(request, dict) or request.get('operation') not in OPERATIONS
            or request.get('contract') != 'patrolbot-runtime-controls'):
        raise AuthorizationError('administrator mutation required')


def sign(wrapper, key, robot_id, *, now=None):
    """Sign the exact server-derived actor and canonical request for one robot."""
    _subject(wrapper, robot_id)
    if set(wrapper) != {'actor', 'request'}:
        raise AuthorizationError('unexpected authorization envelope fields')
    issued = _clock_ms(now)
    claims = {'version': 1, 'robot_id': robot_id, 'issued_at_ms': issued,
              'expires_at_ms': issued + MAX_LIFETIME_MS}
    body = dict(wrapper, authorization=claims)
    signature = hmac.new(_key(key), DOMAIN + _canonical(body), hashlib.sha256).hexdigest()
    # Round-trip to detach mutable caller-owned request/actor dictionaries.
    body = json.loads(_canonical(body))
    body['authorization']['signature'] = signature
    return body


def verify(wrapper, key, robot_id, *, now=None):
    """Verify a short-lived mutation and return only its request and actor."""
    _subject(wrapper, robot_id)
    if set(wrapper) != {'actor', 'request', 'authorization'}:
        raise AuthorizationError('signed tuning authorization required')
    claims = wrapper.get('authorization')
    fields = {'version', 'robot_id', 'issued_at_ms', 'expires_at_ms', 'signature'}
    if not isinstance(claims, dict) or set(claims) != fields:
        raise AuthorizationError('invalid tuning authorization claims')
    issued, expires = claims['issued_at_ms'], claims['expires_at_ms']
    if (type(claims['version']) is not int or claims['version'] != 1
            or claims['robot_id'] != robot_id or type(issued) is not int
            or type(expires) is not int or not 0 <= issued < expires <= 2 ** 53 - 1
            or not 0 < expires - issued <= MAX_LIFETIME_MS):
        raise AuthorizationError('invalid tuning authorization binding')
    current = _clock_ms(now)
    if issued > current + CLOCK_SKEW_MS or expires <= current:
        raise AuthorizationError('expired or future tuning authorization')
    signature = claims['signature']
    if not isinstance(signature, str) or re.fullmatch('[0-9a-f]{64}', signature) is None:
        raise AuthorizationError('invalid tuning authorization signature')
    unsigned = {k: v for k, v in claims.items() if k != 'signature'}
    body = dict(wrapper, authorization=unsigned)
    expected = hmac.new(_key(key), DOMAIN + _canonical(body), hashlib.sha256).hexdigest()
    if not hmac.compare_digest(expected, signature):
        raise AuthorizationError('invalid tuning authorization signature')
    return json.loads(_canonical({'request': wrapper['request'], 'actor': wrapper['actor']}))
