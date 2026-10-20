"""Bind administrator tuning mutations to the authenticated dashboard server."""

import hashlib
import hmac
import json
import math
import os
import re
import stat
import time


DOMAIN = b'patrolbot-tuning-authorization-v1\n'
MAX_BYTES = 20 * 1024
MAX_LIFETIME_MS = 30000
CLOCK_SKEW_MS = 5000
OPERATIONS = frozenset({'save', 'apply', 'redeploy', 'restore'})


class AuthorizationError(ValueError):
    """A mutation lacks a current, valid server authorization."""


def load_key(path):
    """Read a private, regular key file; no default or permissive fallback."""
    if not path:
        raise AuthorizationError('tuning authorization key is not configured')
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | getattr(os, 'O_NOFOLLOW', 0))
    try:
        info = os.fstat(fd)
        if (not stat.S_ISREG(info.st_mode) or info.st_mode & 0o077
                or info.st_uid not in (0, os.geteuid())):
            raise AuthorizationError('tuning key must be a private regular file')
        value = os.read(fd, 4097)
    finally:
        os.close(fd)
    if not 32 <= len(value) <= 4096:
        raise AuthorizationError('tuning key must contain 32 to 4096 bytes')
    return value


def _key(key):
    if not isinstance(key, bytes) or not 32 <= len(key) <= 4096:
        raise AuthorizationError('tuning authorization key is unavailable')
    return key


def _canonical(value):
    try:
        raw = json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)
        encoded = raw.encode('utf-8')
    except (TypeError, ValueError, RecursionError, UnicodeError) as exc:
        raise AuthorizationError('invalid authorization payload') from exc
    if len(encoded) > MAX_BYTES:
        raise AuthorizationError('authorization payload too large')
    return encoded


def _clock_ms(now):
    value = time.time() if now is None else now
    if (type(value) not in (int, float) or not math.isfinite(value)
            or not 0 <= value <= (2 ** 53 - 1) / 1000):
        raise AuthorizationError('invalid authorization clock')
    return int(value * 1000)


def _subject(wrapper, robot_id):
    if not isinstance(robot_id, str) or not 1 <= len(robot_id) <= 128:
        raise AuthorizationError('invalid robot authorization subject')
    if not isinstance(wrapper, dict) or not {'request', 'actor'} <= set(wrapper):
        raise AuthorizationError('invalid authorization envelope')
    actor, request = wrapper['actor'], wrapper['request']
    if (not isinstance(actor, dict) or actor.get('role') != 'administrator'
            or not isinstance(request, dict) or request.get('operation') not in OPERATIONS):
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
