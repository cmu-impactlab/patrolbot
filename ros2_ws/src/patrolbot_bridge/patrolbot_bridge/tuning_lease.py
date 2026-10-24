"""Job-bound maintenance admission, independent of odometry and motor state.

No expiry releases a lease: an interrupted mutation requires explicit recovery.
This object is serialized by the owning node's callback/command mutex. It is not
a second motion owner, a signature verifier, or authority to run arbitrary work.
"""

import re
import json
from uuid import UUID


VERSION = 2
CONTEXT_KEYS = frozenset(('job_id', 'digest', 'plan_hash'))


def context(value):
    if not isinstance(value, dict) or set(value) != CONTEXT_KEYS:
        raise ValueError('exact maintenance context required')
    if not isinstance(value['job_id'], str) or str(UUID(value['job_id'])) != value['job_id']:
        raise ValueError('canonical maintenance job required')
    if any(not isinstance(value[key], str) or not re.fullmatch(r'sha256:[0-9a-f]{64}', value[key])
           for key in ('digest', 'plan_hash')):
        raise ValueError('exact maintenance digest and plan hash required')
    return dict(value)


class MaintenanceLease:
    def __init__(self):
        self.owner = None
        self.released = None

    @property
    def held(self):
        return self.owner is not None

    def acquire(self, value, *, idle):
        value = context(value)
        if self.owner is not None:
            return self.owner == value
        if idle is not True or value == self.released:
            return False
        self.owner = value
        return True

    def matches(self, value):
        return self.owner == context(value)

    def release(self, value):
        value = context(value)
        if self.owner is None:
            return self.released == value
        if self.owner != value:
            return False
        self.owner, self.released = None, value
        return True

    def snapshot(self):
        return {'version': VERSION, 'owner': dict(self.owner) if self.owner else None}


def request_context(request):
    return context({key: getattr(request, key) for key in CONTEXT_KEYS})


def serve(lease, request, response, *, idle, evidence):
    """Node callbacks supply real idle/cache evidence under their owner mutex."""
    try:
        value = request_context(request)
        data = evidence()
        if not isinstance(data, dict) or set(data) & {'version', 'owner'}:
            raise ValueError('invalid native maintenance evidence fields')
        # Validate the entire response BEFORE acquiring or releasing. A failed
        # encoding must not return a definitive refusal after creating an
        # invisible remote owner (nor clear an owner with no usable ACK).
        prospective = dict(data, version=VERSION, owner=value)
        if len(json.dumps(prospective, sort_keys=True, allow_nan=False).encode()) > 20 * 1024:
            raise ValueError('maintenance evidence too large')
        if request.operation == 'acquire':
            accepted = lease.acquire(value, idle=idle())
        elif request.operation == 'release':
            accepted = lease.release(value)
        elif request.operation == 'inspect':
            accepted = lease.matches(value)
        elif request.operation == 'observe':
            # Read-only actual state; it creates no owner and grants no command.
            accepted = True
        else:
            raise ValueError('unallowlisted maintenance operation')
        response.accepted = accepted
        response.detail = 'maintenance acknowledged' if accepted else 'maintenance owner/input busy'
        response.evidence_json = json.dumps(dict(lease.snapshot(), **(data if accepted else {})), sort_keys=True, allow_nan=False)
        if len(response.evidence_json.encode()) > 20 * 1024:
            raise ValueError('maintenance evidence too large')
    except (ValueError, TypeError, AttributeError):
        response.accepted = False
        response.detail = 'invalid maintenance context or evidence'
        response.evidence_json = '{}'
    return response
