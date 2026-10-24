"""
Typed, actor-aware tuning service (inspect / save / status / activate).

The service is transport-agnostic and ROS-free. A ROS ``/patrolbot/tuning``
service, the web bridge or a test all call :meth:`TuningService.handle` with the
same bounded JSON wrapper::

    {"request": <canonical request>, "actor": {"id": int, "username": str,
                                               "role": "administrator"|"operator"|"observer"}}

Rules enforced here:

* The wrapper and the canonical request are strictly parsed (bounded bytes,
  no ``NaN``/``Infinity``, no duplicate keys, no unknown fields).
* ``read``/``status`` are allowed for any authenticated role. Every mutating
  operation requires a fresh ``administrator`` actor. The actor is derived by
  the trusted server and passed in; a browser cannot assert privilege.
* No operation takes a file path, node name, ROS name or shell command. The
  only inputs are logical ids from the registry.
* No operation publishes velocity, arms teleop, enables motors, undocks or
  injects a pose. Mutations only write robot-owned YAML and job records.
* ``robot_tuning_v1`` is never advertised; ``robot_tuning_read_v1`` is
  advertised only because inspection is actually reachable.
"""

from __future__ import annotations

import json
import uuid

from . import tuning_spec
from .tuning_json import loads as bounded_json_loads
from .tuning_store import READY, StoreError, StoreUnavailable

REQUEST_MAX_BYTES = 20 * 1024
RESPONSE_MAX_BYTES = 256 * 1024

CAPABILITY = tuning_spec.CAPABILITY
READ_CAPABILITY = 'robot_tuning_read_v1'
ROLES = ('administrator', 'operator', 'observer')
REQUEST_OPERATIONS = ('read', 'save', 'apply', 'redeploy', 'status', 'restore')
# Mutations that must be administrator-only. `save` changes desired state and
# is a mutation even though it never activates; it belongs here too.
ADMIN_OPERATIONS = ('save', 'apply', 'redeploy', 'restore')
ACTOR_KEYS = frozenset({'id', 'username', 'role'})
ENVELOPE_KEYS = frozenset({'request', 'actor'})
MAX_USERNAME = 128

_COMMON_REQUEST_KEYS = frozenset({'contract', 'version', 'operation', 'request_id',
                                  'generation'})
_REQUEST_KEYS = {
    'read': _COMMON_REQUEST_KEYS,
    'status': _COMMON_REQUEST_KEYS | {'job_id'},
    'save': _COMMON_REQUEST_KEYS | {'base_revision', 'changes'},
    'apply': _COMMON_REQUEST_KEYS | {'expected_revision', 'review'},
    'redeploy': _COMMON_REQUEST_KEYS | {'expected_revision', 'review'},
    'restore': _COMMON_REQUEST_KEYS | {'expected_revision', 'restore_revision', 'target', 'review'},
}


class ServiceError(ValueError):
    def __init__(self, outcome: str, detail: str):
        super().__init__(detail)
        self.outcome = outcome
        self.detail = detail


def _reject_constant(name):
    raise ServiceError('refused', 'invalid_json: ' + name)


def _no_duplicates(pairs):
    seen = set()
    result = {}
    for key, value in pairs:
        if key in seen:
            raise ServiceError('refused', 'duplicate_key: ' + str(key))
        seen.add(key)
        result[key] = value
    return result


def strict_json_loads(text: str, *, max_bytes: int = REQUEST_MAX_BYTES):
    if not isinstance(text, str):
        raise ServiceError('refused', 'invalid_json: expected text')
    encoded = text.encode('utf-8')
    if len(encoded) > max_bytes:
        raise ServiceError('refused', 'payload_too_large')
    try:
        value = bounded_json_loads(text, max_bytes)
    except ServiceError:
        raise
    except (ValueError, RecursionError) as exc:
        raise ServiceError('refused', 'invalid_json: ' + exc.__class__.__name__) from exc
    return value


def parse_actor(actor) -> dict:
    if not isinstance(actor, dict):
        raise ServiceError('refused', 'invalid_actor')
    if set(actor) != ACTOR_KEYS:
        raise ServiceError('refused', 'invalid_actor_fields')
    actor_id = actor.get('id')
    username = actor.get('username')
    role = actor.get('role')
    if not isinstance(actor_id, int) or isinstance(actor_id, bool):
        raise ServiceError('refused', 'invalid_actor_id')
    if not isinstance(username, str) or not username or len(username) > MAX_USERNAME:
        raise ServiceError('refused', 'invalid_actor_username')
    if role not in ROLES:
        raise ServiceError('refused', 'invalid_actor_role')
    return {'id': actor_id, 'username': username, 'role': role}


def parse_request(request) -> dict:
    if not isinstance(request, dict):
        raise ServiceError('refused', 'invalid_request')
    if request.get('contract') != tuning_spec.CONTRACT:
        raise ServiceError('refused', 'bad_contract')
    if type(request.get('version')) is not int \
            or request['version'] != tuning_spec.CONTRACT_VERSION:
        raise ServiceError('refused', 'bad_version')
    operation = request.get('operation')
    if operation not in REQUEST_OPERATIONS:
        raise ServiceError('refused', 'unsupported_operation')
    allowed = _REQUEST_KEYS[operation]
    required = allowed - {'review'} - ({'job_id'} if operation == 'status' else set())
    if not required.issubset(request) or set(request) - allowed:
        raise ServiceError('refused', 'unknown_field')
    if not _canonical_uuid(request.get('request_id')):
        raise ServiceError('refused', 'invalid_request_id')
    generation = request.get('generation')
    if not isinstance(generation, dict) \
            or set(generation) != {'capability', 'session_id'} \
            or generation.get('capability') != CAPABILITY \
            or not _canonical_uuid(generation.get('session_id')):
        raise ServiceError('refused', 'invalid_generation')
    if operation == 'save':
        if not _safe_revision(request.get('base_revision')):
            raise ServiceError('refused', 'invalid_base_revision')
        changes = request.get('changes')
        if not isinstance(changes, list) or not 1 <= len(changes) <= 32:
            raise ServiceError('refused', 'invalid_changes')
        for change in changes:
            if not isinstance(change, dict) or set(change) != {'id', 'value'}:
                raise ServiceError('refused', 'invalid_change')
    if operation in ('apply', 'redeploy', 'restore'):
        if not _safe_revision(request.get('expected_revision')):
            raise ServiceError('refused', 'invalid_expected_revision')
        if 'review' in request:
            import re
            review = request['review']
            if (not isinstance(review, dict) or set(review) != {'digest', 'plan_hash'}
                    or any(not isinstance(value, str) or not re.fullmatch(r'sha256:[0-9a-f]{64}', value)
                           for value in review.values())):
                raise ServiceError('refused', 'invalid_review')
    if operation == 'restore':
        if not _safe_revision(request.get('restore_revision')) \
                or request.get('target') != 'last_known_good':
            raise ServiceError('refused', 'invalid_restore')
    if operation == 'status' and request.get('job_id') is not None \
            and not _canonical_uuid(request.get('job_id')):
        raise ServiceError('refused', 'invalid_job_id')
    return request


def _safe_revision(value) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and 0 <= value < 2 ** 53


def _canonical_uuid(value) -> bool:
    import uuid as _uuid
    try:
        return str(_uuid.UUID(value)) == value
    except (ValueError, AttributeError, TypeError):
        return False


class TuningService:
    def __init__(self, store, coordinator, *, clock=None, session_id=None,
                 mutations_enabled=False, runtime_readback=None, parameter_readback=None,
                 write_admission=None):
        self.store = store
        self.coordinator = coordinator
        self.clock = clock
        self.session_id = session_id or str(uuid.uuid4())
        self.coordinator.session_id = self.session_id
        self.mutations_enabled = mutations_enabled is True
        self.runtime_readback = runtime_readback
        self.parameter_readback = parameter_readback
        self.write_admission = write_admission

    def _save_admission(self):
        if self.write_admission is None:
            return False, 'Fresh SBC link evidence is unavailable.'
        try:
            allowed, reason = self.write_admission()
            return allowed is True, str(reason)
        except Exception:
            return False, 'Fresh SBC link evidence is unavailable.'

    # ------------------------------------------------------------------
    # Snapshot
    # ------------------------------------------------------------------
    def runtime_snapshot(self) -> dict:
        if self.store.base.is_dir():
            with self.store.writer():
                return self._runtime_snapshot()
        return self._runtime_snapshot()

    def _runtime_snapshot(self) -> dict:
        state = self.store.snapshot_state()
        registry = self.store.registry
        index = self.store.index
        ready = state['status'] == READY
        desired = {}
        active = {}
        active_known = False
        effective_limits = None
        digest = None
        desired_state = None
        active_overrides = None
        activation_plan = None
        restore_plan = None
        if ready:
            # Existing saved state must never be replaced by YAML starting
            # values when its revision cannot be read or validated.
            # Validate each immutable revision once per response while holding
            # the writer lock. Re-reading the same YAML/checksums for desired,
            # committed values and digest used up the observation freshness
            # budget. Nothing is cached across requests: corruption is still
            # checked on every read.
            desired_state = self.store.load_revision(state['desired_revision'])
            desired = desired_state['overrides']
            if state['active_revision'] is not None:
                committed = (desired_state if state['active_revision'] == state['desired_revision']
                             else self.store.load_revision(state['active_revision']))
                active_overrides = committed['overrides']
                activation_plan = self.coordinator.preview(
                    candidate=desired_state, active=committed, desired_revision=state['desired_revision'])
                last_good = state.get('last_known_good')
                if last_good is not None:
                    restore_state = (committed if last_good == state['active_revision']
                                     else desired_state if last_good == state['desired_revision']
                                     else self.store.load_revision(last_good))
                    restore_plan = self.coordinator.preview(
                        candidate=restore_state, active=committed,
                        desired_revision=state['desired_revision'], operation='restore')
                if active_overrides is not None:
                    digest = committed['meta']['generated_digest']
                    observed = self.runtime_readback() if self.runtime_readback else None
                    if (isinstance(observed, dict)
                            and observed.get('revision') == state['active_revision']
                            and observed.get('digest') == digest):
                        active = observed['values']
                        effective_limits = observed['effective_limits']
                        active_known = True
        # Before the store is initialized there is no durable DESIRED revision.
        # The operator still needs honest starting values to inspect, so read
        # them from the packaged consumer YAML (the real deployment
        # configuration) rather than the reviewed catalog. They are never
        # presented as saved: desired_revision stays None and active_known stays
        # False below.
        starting_defaults = {}
        if not ready:
            starting_defaults = tuning_spec.yaml_starting_values(index)
        desired_full = {}
        active_full = {}
        for pid, entry in sorted(index.items()):
            if entry.get('accepts_edits') is not True and entry.get('default') is None:
                continue
            desired_full[pid] = desired.get(
                pid, starting_defaults.get(pid, entry.get('default')))
            active_full[pid] = active.get(pid) if active_known else None
        safe_apply = self.coordinator.safe_apply_state() if ready else {
            'allowed': False, 'reason': 'Robot tuning has not been initialized.'}
        reviewed_backend = getattr(self.coordinator.backend, 'requires_review', False)
        activation_supported = ready and self.coordinator.activation_supported
        if reviewed_backend and (activation_plan is None or not activation_plan['executable']):
            activation_supported = False
            safe_apply = {'allowed': False, 'reason': 'Reviewed scoped activation is not commissioned; saving targets remains available.'}
        job = self.store.latest_job() if ready else None
        save_allowed, save_reason = self._save_admission()
        write_reason = ('Robot tuning has not been initialized.' if not ready else
                        'Signed tuning authorization has not been configured.'
                        if not self.mutations_enabled else
                        save_reason if not save_allowed else None)
        return {
            'readback': self.parameter_readback() if self.parameter_readback else None,
            'activation_plan': activation_plan,
            'restore_plan': restore_plan,
            'write_disabled_reason': write_reason,
            'session_id': self.session_id if ready else None,
            'desired_revision': state.get('desired_revision'),
            'active_revision': state.get('active_revision') if active_known else None,
            'hardware_session_generation': (
                observed.get('hardware_session_generation') if active_known else None),
            'committed_active_revision': state.get('active_revision'),
            'committed_active': active_overrides,
            'last_known_good_revision': state.get('last_known_good'),
            'policy_required': ready,
            'desired': desired_full,
            'active': active_full,
            'active_known': active_known,
            'effective_limits': effective_limits,
            'pending_keys': state.get('pending_keys') or [],
            'catalog': [
                _catalog_entry(
                    entry, starting_defaults.get(entry['id'], entry.get('default')),
                     activation_supported=activation_supported)
                for entry in registry['parameters']
            ],
            'writable': ready and self.mutations_enabled and save_allowed,
            'save_supported': ready and self.mutations_enabled and save_allowed,
            'advertised_capability': READ_CAPABILITY if ready else None,
            'activation_supported': activation_supported,
            'safe_apply': safe_apply,
            'latest_job': job,
            'digest': digest,
            'desired_digest': desired_state['meta']['generated_digest'] if ready else None,
            'compatibility': f'{CAPABILITY}:{self.store.registry_digest}',
        }

    # ------------------------------------------------------------------
    # Dispatch
    # ------------------------------------------------------------------
    def handle(self, payload) -> dict:
        request_id = None
        mutation_dispatched = False
        try:
            envelope = strict_json_loads(
                payload if isinstance(payload, str)
                else json.dumps(payload, allow_nan=False))
            if not isinstance(envelope, dict) or set(envelope) != ENVELOPE_KEYS:
                raise ServiceError('refused', 'invalid_envelope')
            actor = parse_actor(envelope.get('actor'))
            request = parse_request(envelope.get('request'))
            request_id = request['request_id']
            operation = request['operation']
            is_admin = actor['role'] == 'administrator'
            if operation in ADMIN_OPERATIONS and not is_admin:
                raise ServiceError('refused', 'not_administrator')
            if operation in ADMIN_OPERATIONS and not self.mutations_enabled:
                raise ServiceError('refused', 'signed_mutation_transport_unavailable')
            if operation == 'save':
                allowed, reason = self._save_admission()
                if not allowed:
                    raise ServiceError('refused', reason)
            mutation_dispatched = operation in ADMIN_OPERATIONS
            result = self._dispatch(operation, request, actor)
            return self._response(request_id, result['outcome'], result['detail'],
                                  result.get('job'), mutation=mutation_dispatched)
        except ServiceError as exc:
            return self._response(request_id, exc.outcome, exc.detail, None)
        except StoreUnavailable as exc:
            outcome = 'unknown' if mutation_dispatched else 'refused'
            return self._response(request_id, outcome,
                                  'store_unavailable: ' + exc.code, None,
                                  mutation=mutation_dispatched)
        except StoreError as exc:
            return self._response(request_id, 'refused', exc.code, None)
        except (ValueError, UnicodeError, TypeError, RecursionError):
            return self._response(
                request_id, 'unknown' if mutation_dispatched else 'refused',
                'mutation outcome unknown' if mutation_dispatched else 'invalid_json', None,
                mutation=mutation_dispatched)
        except OSError:
            return self._response(
                request_id, 'unknown' if mutation_dispatched else 'refused',
                'persistence outcome unknown; query the same request, never retry blindly', None,
                mutation=mutation_dispatched)

    def _dispatch(self, operation, request, actor):
        if operation in ('read', 'status'):
            job = None
            if operation == 'status':
                job = self.coordinator.status(request.get('job_id'))
            return {'outcome': 'ok', 'detail': 'snapshot', 'job': job}
        if request['generation']['session_id'] != self.session_id:
            return {
                'outcome': 'conflict',
                'detail': 'tuning session changed; refresh state',
                'job': None,
            }
        if operation == 'save':
            with self.store.writer():
                # A queued filesystem writer must not reuse old link evidence.
                allowed, reason = self._save_admission()
                if not allowed:
                    raise ServiceError('refused', reason)
                outcome = self.store.save(
                    request_id=request['request_id'],
                    base_revision=request.get('base_revision'),
                    changes=request['changes'],
                    actor=actor,
                    reason=request.get('reason', ''),
                )
            return {'outcome': outcome.get('outcome', 'failed'),
                    'detail': outcome.get('detail', ''), 'job': None}
        # apply / redeploy / restore
        target = request.get('restore_revision') if operation == 'restore' else None
        outcome = self.coordinator.request(
            operation=operation,
            request_id=request['request_id'],
            expected_revision=request['expected_revision'],
            actor=actor,
            target_revision=target,
            review=request.get('review'),
        )
        return {'outcome': outcome.get('outcome', 'failed'),
                'detail': outcome.get('detail', ''),
                'job': self.coordinator.status(outcome.get('job_id'))}

    def _response(self, request_id, outcome, detail, job, *, mutation=False):
        try:
            snapshot = self.runtime_snapshot()
        except (StoreError, ValueError, KeyError, TypeError, OSError):
            snapshot = None
            outcome = 'unknown' if mutation else 'refused'
            detail = ('Robot-owned tuning state is unavailable or corrupt; '
                      + ('mutation outcome unknown; query the same request.' if mutation
                         else 'no activation is permitted.'))
        response = {
            'request_id': request_id,
            'outcome': outcome,
            'detail': _bounded_detail(detail),
            'snapshot': snapshot,
            'job': job,
        }
        encoded = json.dumps(response, sort_keys=True, allow_nan=False)
        if len(encoded.encode('utf-8')) > RESPONSE_MAX_BYTES:  # pragma: no cover - defensive
            response['snapshot'] = None
            encoded = json.dumps(response, sort_keys=True, allow_nan=False)
        return response

    def result_json(self, payload):
        """Return the serialized response for the ROS ``RobotTuning`` service."""
        return json.dumps(self.handle(payload), sort_keys=True, allow_nan=False)


def _bounded_detail(detail, limit=200) -> str:
    if not isinstance(detail, str):
        detail = '' if detail is None else str(detail)
    detail = detail.replace('\n', ' ')
    return detail[:limit]


_UNSET = object()


def _catalog_entry(entry, default=_UNSET, *, activation_supported=False) -> dict:
    """
    Project one registry entry, optionally replacing its starting default.

    The returned dict is always freshly built; the registry entry is never
    mutated, so a YAML starting value can be surfaced without changing the
    reviewed catalog.
    """
    source = entry.get('source') or {}
    bounds = entry.get('bounds') or {}
    starting_default = entry.get('default') if default is _UNSET else default
    editable = (entry.get('accepts_edits') is True and not entry.get('protected')
                and not entry.get('not_applicable'))
    return {
        'id': entry['id'],
        'label': entry.get('label', entry['id']),
        'group': entry.get('group'),
        'type': entry.get('type'),
        'value_kind': entry.get('value_kind'),
        'unit': entry.get('unit'),
        'default': starting_default,
        'accepts_edits': entry.get('accepts_edits') is True,
        'activatable': editable and activation_supported,
        'protected': entry.get('protected') is True,
        'not_applicable': entry.get('not_applicable') is True,
        'mutability': 'restart_required' if editable else entry.get('mutability'),
        'mutability_fallback': entry.get('mutability_fallback'),
        'live_verified': entry.get('live_verified') is True,
        'restart_scopes': list(entry.get('restart_scopes') or []),
        'bounds': {
            'status': bounds.get('status'),
            'lower': bounds.get('lower'),
            'upper': bounds.get('upper'),
            'lower_inclusive': bounds.get('lower_inclusive'),
            'upper_inclusive': bounds.get('upper_inclusive'),
            'note': bounds.get('note', ''),
        },
        'effective_ceiling_ref': entry.get('effective_ceiling_ref'),
        'exceeds_effective_ceiling': entry.get('exceeds_effective_ceiling') is True,
        'package_version_evidence': entry.get('package_version_evidence'),
        'activation_block': (None if editable and activation_supported
                             else entry.get('activation_block')),
        'dependencies': entry.get('dependencies', []),
        'side_effects': entry.get('side_effects', []),
        'source': {
            'file': source.get('file'),
            'node': source.get('node'),
            'key': source.get('key'),
            'index': source.get('index'),
        },
    }


__all__ = [
    'TuningService',
    'ServiceError',
    'parse_actor',
    'parse_request',
    'strict_json_loads',
    'REQUEST_MAX_BYTES',
    'RESPONSE_MAX_BYTES',
    'READ_CAPABILITY',
    'ROLES',
    'REQUEST_OPERATIONS',
]
