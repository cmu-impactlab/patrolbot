"""Separate Ed25519 host capabilities; runtimes receive PUBLIC keys only.

The private key belongs exclusively to a locally approved host operator/helper,
not the core tuning signer, web bridge, browser, Docker image or shared state.
Signing is a trusted local API, not an RPC endpoint or dashboard operation.
"""

import base64
import math

from .tuning_handoff import SERVICES, canonical_bytes, is_digest
from .tuning_store import StoreUnavailable, _canonical_uuid


MAX_LIFETIME = 300
FIELDS = frozenset((
    'version', 'kind', 'authorization_id', 'job_id', 'revision', 'digest',
    'plan_hash', 'handoff_digest', 'store_generation', 'services', 'host_boot_id',
    'config_digest', 'volume_id', 'issued_at', 'expires_at',
))
BOOT_FIELDS = frozenset(('boot_id', 'execution_authorization_id'))


def validate_claims(claims, *, kind, now, host_boot_id):
    fields = FIELDS | (BOOT_FIELDS if kind == 'candidate_boot' else frozenset())
    if (kind not in ('execute_robot_docker', 'candidate_boot')
            or not isinstance(claims, dict) or set(claims) != fields
            or type(claims.get('version')) is not int or claims['version'] != 1
            or claims.get('kind') != kind
            or claims.get('services') != list(SERVICES)
            or type(claims.get('revision')) is not int or not 1 <= claims['revision'] < 2 ** 53
            or any(not _canonical_uuid(claims.get(key)) for key in (
                'authorization_id', 'job_id', 'store_generation', 'host_boot_id', 'volume_id'))
            or any(not is_digest(claims.get(key)) for key in (
                'digest', 'plan_hash', 'handoff_digest', 'config_digest'))
            or claims.get('host_boot_id') != host_boot_id):
        raise StoreUnavailable('invalid_host_authorization')
    if kind == 'candidate_boot' and any(not _canonical_uuid(claims.get(key))
                                      for key in BOOT_FIELDS):
        raise StoreUnavailable('invalid_host_boot_binding')
    issued, expires = claims.get('issued_at'), claims.get('expires_at')
    if (any(type(value) not in (int, float) or not math.isfinite(value)
            for value in (issued, expires, now))
            or not issued <= now < expires or not 0 < expires - issued <= MAX_LIFETIME):
        raise StoreUnavailable('host_authorization_expired_or_invalid_clock')
    return claims


def sign(claims, private_key):
    """Trusted host-only API. Never load a private key from a wire-supplied path."""
    from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
    if not isinstance(private_key, Ed25519PrivateKey):
        raise StoreUnavailable('ed25519_host_key_required')
    return {'claims': claims, 'signature': base64.b64encode(
        private_key.sign(canonical_bytes(claims))).decode('ascii')}


def verify(envelope, public_key, *, kind, now, host_boot_id):
    from cryptography.exceptions import InvalidSignature
    from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey
    if (not isinstance(public_key, Ed25519PublicKey) or not isinstance(envelope, dict)
            or set(envelope) != {'claims', 'signature'}
            or not isinstance(envelope.get('signature'), str)
            or len(envelope['signature']) != 88):
        raise StoreUnavailable('invalid_host_signature')
    try:
        signature = base64.b64decode(envelope['signature'], validate=True)
        public_key.verify(signature, canonical_bytes(envelope['claims']))
    except (InvalidSignature, ValueError, TypeError) as exc:
        raise StoreUnavailable('invalid_host_signature') from exc
    return validate_claims(envelope['claims'], kind=kind, now=now, host_boot_id=host_boot_id)


def bind_handoff(claims, handoff):
    for key in ('job_id', 'revision', 'digest', 'plan_hash', 'handoff_digest',
                'store_generation', 'services'):
        if claims.get(key) != handoff.get(key):
            raise StoreUnavailable('host_authorization_handoff_mismatch')
