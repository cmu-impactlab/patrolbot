"""Purpose-bound core-to-producer delegation, separate from HTTP runtime requests."""
from .runtime_authorization import (_canonical, _key, _clock_ms, AuthorizationError,
                                    MAX_LIFETIME_MS, CLOCK_SKEW_MS)
import hashlib
import hmac
import json

DOMAIN = b'patrolbot-runtime-manual-consumer-authorization-v1\n'


def sign(request, actor, key, robot_id, consumer):
    issued = _clock_ms(None)
    body = dict(request=request, actor=actor, robot_id=robot_id, consumer=consumer,
                issued_at_ms=issued, expires_at_ms=issued + MAX_LIFETIME_MS)
    body['signature'] = hmac.new(_key(key), DOMAIN + _canonical(body), hashlib.sha256).hexdigest()
    return json.dumps(body, allow_nan=False)


def verify(raw, request, key, robot_id, consumer):
    from .runtime_contract import loads
    body = loads(raw)
    if not isinstance(body, dict) or set(body) != {'request', 'actor', 'robot_id', 'consumer', 'issued_at_ms', 'expires_at_ms', 'signature'}:
        raise AuthorizationError('Signed consumer delegation required')
    signature = body.pop('signature')
    issued, expires, actor = body['issued_at_ms'], body['expires_at_ms'], body['actor']
    if (type(issued) is not int or type(expires) is not int or not 0 < expires-issued <= MAX_LIFETIME_MS
            or issued > _clock_ms(None) + CLOCK_SKEW_MS or expires <= _clock_ms(None)
            or body['robot_id'] != robot_id or body['consumer'] != consumer or body['request'] != request
            or not isinstance(actor, dict) or set(actor) != {'id', 'username', 'role'}
            or actor['role'] not in ('operator', 'administrator') or type(actor['id']) is not int
            or not isinstance(actor['username'], str) or not 1 <= len(actor['username']) <= 128):
        raise AuthorizationError('Invalid consumer authorization binding')
    expected = hmac.new(_key(key), DOMAIN + _canonical(body), hashlib.sha256).hexdigest()
    if (not isinstance(signature, str) or len(signature) != 64
            or any(c not in '0123456789abcdef' for c in signature)
            or not hmac.compare_digest(signature, expected)):
        raise AuthorizationError('Invalid consumer authorization signature')


def claims(request):
    return {key: getattr(request, key) for key in ('phase', 'request_id', 'expected_generation',
            'hardware_generation', 'linear_m_s', 'angular_rad_s')}
