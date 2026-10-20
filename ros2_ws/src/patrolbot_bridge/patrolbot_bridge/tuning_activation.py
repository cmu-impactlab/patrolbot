"""
Offline activation / redeploy / restore state machine.

This is the *safe* path by which a saved desired revision could one day become
the running configuration. It is deliberately inert on a real robot: the only
shipped backend is :class:`UnavailableBackend`, whose ``approved`` is ``False``,
so every activation request is refused with ``activation_unavailable`` until a
reviewed, evidence-backed fixed-operation backend is wired in and verified.

The logic itself is real and unit-tested with an offline fake backend. It
proves the properties the plan requires, without touching ROS:

* admission is gated on fresh stationary raw odometry, hardware/dock/fault
  state, zero command buffers, both physical *and* web deadman channels fresh
  and not held, authoritative idle autonomy, and clear map/undock/lifecycle
  state. Missing evidence is *busy*, never permission.
* the job is journalled durably *before* any service is stopped.
* an ambiguous/timed-out reload keeps the hold and reports ``unknown``; it is
  never blindly retried.
* an unfinished/unknown job survives a restart and holds ALL motion paths,
  including the manual deadman bypass.
* no code path publishes velocity, arms teleop, enables motors, undocks or
  injects a pose.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import hashlib
import json
import threading
import time
import uuid

from .tuning_store import SCOPES, StoreError, StoreUnavailable

# Motion-affecting operations this module owns. Everything else is read-only
# and never enters the state machine.
MUTATING_OPERATIONS = ('apply', 'redeploy', 'restore')

JOB_PENDING = 'pending'
JOB_RUNNING = 'running'
JOB_SUCCEEDED = 'succeeded'
JOB_FAILED = 'failed'
JOB_UNKNOWN = 'unknown'
UNSETTLED_STATES = (JOB_PENDING, JOB_RUNNING, JOB_UNKNOWN)

# Fixed service allowlist for a future host helper. These are the *only*
# operations a helper may ever perform; there is no arbitrary command, path,
# Docker socket or SSH surface. The mapping is data so it can be reviewed and
# unit-tested without a host.
HOST_SERVICES = {
    'bringup': ('patrolbot-bringup',),
    'navigation': ('patrolbot-navigation',),
}

# Evidence fields required before any activation. All are tri-state: ``True``
# passes, ``False`` is busy, ``None`` is unknown (also busy).
REQUIRED_FRESH = (
    'raw_odom_fresh',
    'stationary',
    'hardware_ok',
    'dock_ok',
    'faults_clear',
    'command_buffers_zero',
    'physical_deadman_fresh',
    'web_deadman_fresh',
    'autonomy_idle',
    'maps_clear',
    'undock_clear',
    'lifecycle_clear',
)
REQUIRED_NOT_HELD = ('physical_deadman_held', 'web_deadman_held')


@dataclass(frozen=True)
class Evidence:
    raw_odom_fresh: bool | None = None
    stationary: bool | None = None
    hardware_ok: bool | None = None
    dock_ok: bool | None = None
    faults_clear: bool | None = None
    command_buffers_zero: bool | None = None
    physical_deadman_fresh: bool | None = None
    physical_deadman_held: bool | None = None
    web_deadman_fresh: bool | None = None
    web_deadman_held: bool | None = None
    autonomy_idle: bool | None = None
    maps_clear: bool | None = None
    undock_clear: bool | None = None
    lifecycle_clear: bool | None = None

    def merged(self, other: 'Evidence') -> 'Evidence':
        patch = {}
        for field in self.__dataclass_fields__:
            value = getattr(other, field)
            if value is not None:
                patch[field] = value
        return replace(self, **patch)


def evaluate_evidence(evidence: Evidence):
    """Return ``(allowed, reason)``. Unknown and False are both refusals."""
    if not isinstance(evidence, Evidence):
        return False, 'busy: evidence unavailable'
    for name in REQUIRED_NOT_HELD:
        value = getattr(evidence, name)
        if value is None:
            return False, f'busy: {name} unknown'
        if value is not False:
            return False, f'busy: {name}'
    for name in REQUIRED_FRESH:
        value = getattr(evidence, name)
        if value is None:
            return False, f'busy: {name} unknown'
        if value is not True:
            return False, f'busy: {name}'
    return True, ''


@dataclass(frozen=True)
class ReloadResult:
    status: str
    detail: str = ''


class AdmissionRefused(ValueError):
    """A remote owner definitively refused maintenance before any reload began."""


class UnavailableBackend:
    """Default backend: no verified native mutability, so activation is off."""

    approved = False
    detail = 'native parameter mutability is not verified; restart-required'

    def probe(self) -> Evidence:  # pragma: no cover - never reached when refused
        return Evidence()

    def reload(self, scopes, revision, digest) -> ReloadResult:  # pragma: no cover
        return ReloadResult(JOB_UNKNOWN, self.detail)


class FakeBackend:
    """Deterministic offline backend for tests. ``approved`` must be explicit."""

    def __init__(self, evidence: Evidence, *, approved=True, result=None, raises=None,
                 record=None):
        self._evidence = evidence
        self.approved = approved
        self._result = result
        self._raises = raises
        self._record = record if record is not None else []

    def probe(self) -> Evidence:
        return self._evidence

    def reload(self, scopes, revision, digest) -> ReloadResult:
        self._record.append({
            'scopes': tuple(scopes),
            'revision': revision,
            'digest': digest,
        })
        if self._raises is not None:
            raise self._raises
        if self._result is not None:
            return self._result
        return ReloadResult('ok', 'reloaded')


class FixedOperationPlan:
    """
    A validated, in-memory description of a *fixed* host operation.

    There is no ``sh``/``subprocess`` here and no arbitrary path: a caller can
    only name an allowlisted scope and an exact revision digest. Execution is
    the backend's job; this is the reviewable allowlist check.
    """

    def __init__(self, allowlist=None):
        self.allowlist = dict(allowlist or HOST_SERVICES)

    def services_for(self, scope) -> tuple:
        services = self.allowlist.get(scope)
        if not services:
            raise StoreError('unknown_scope', repr(scope))
        return tuple(services)

    def build(self, *, scope, revision, digest) -> dict:
        if not isinstance(revision, int) or isinstance(revision, bool) or revision < 0:
            raise StoreError('invalid_revision', repr(revision))
        if not isinstance(digest, str) or not digest.startswith('sha256:') or len(digest) != 71:
            raise StoreError('invalid_digest', repr(digest))
        return {
            'scope': scope,
            'services': self.services_for(scope),
            'revision': revision,
            'digest': digest,
        }


def _job_id() -> str:
    return str(uuid.uuid4())


class ActivationCoordinator:
    """Drives one activation job at a time against the existing motion owner."""

    def __init__(self, store, owner, backend, *, clock=None, id_factory=None,
                 plan=None):
        self.store = store
        self.owner = owner
        self.backend = backend
        self.clock = clock or time.time
        self.id_factory = id_factory or _job_id
        self.plan = plan or FixedOperationPlan()
        self._held_job = None
        # Real integrations share EpochRuntime.lock; there is no second ROS
        # lifecycle coordinator. The local lock is for ROS-free test owners.
        self._owner_lock = getattr(owner, 'lock', threading.RLock())

    # ------------------------------------------------------------------
    # Hold integration with the existing EpochRuntime owner
    # ------------------------------------------------------------------
    def _set_hold(self, held, job_id):
        self._held_job = job_id if held else None
        setter = getattr(self.owner, 'set_tuning_hold', None)
        if callable(setter):
            setter(bool(held), job_id or '')

    @property
    def hold(self) -> bool:
        return self._held_job is not None

    @property
    def activation_supported(self) -> bool:
        return getattr(self.backend, 'approved', False) is True

    def _digest_for(self, revision: int) -> str:
        return self.store.load_revision(revision)['meta']['generated_digest']

    # ------------------------------------------------------------------
    # Admission
    # ------------------------------------------------------------------
    def safe_apply_state(self):
        """Return ``{allowed, reason}`` for the current runtime snapshot."""
        if not self.activation_supported:
            return {'allowed': False, 'reason': 'activation_unavailable: ' + (
                getattr(self.backend, 'detail', 'no verified backend'))}
        job = self.store.latest_job()
        if job and job.get('state') in UNSETTLED_STATES:
            return {'allowed': False, 'reason': f'busy: {job["state"]} job {job["job_id"]}'}
        evidence = self.backend.probe()
        allowed, reason = evaluate_evidence(evidence)
        if not allowed:
            return {'allowed': False, 'reason': reason}
        return {'allowed': True, 'reason': 'stationary and idle'}

    # ------------------------------------------------------------------
    # Requests
    # ------------------------------------------------------------------
    def request(self, *, operation, request_id, expected_revision, actor,
                target_revision=None):
        if operation not in MUTATING_OPERATIONS:
            return self._finish(request_id, 'refused', f'unsupported_operation: {operation}',
                                None, None)
        with self.store.writer():
            self.store.require_ready()
            fingerprint = hashlib.sha256(json.dumps({
                'operation': operation, 'expected_revision': expected_revision,
                'target_revision': target_revision, 'actor': actor,
            }, sort_keys=True, allow_nan=False).encode()).hexdigest()
            existing = self.store.get_request(request_id)
            if existing is not None:
                if existing.get('fingerprint') != fingerprint:
                    return {'request_id': request_id, 'outcome': 'refused',
                            'detail': 'request_id belongs to a different or unverifiable intent',
                            'job_id': None, 'revision': None}
                return self.store._dedup_result(existing, request_id)

            self.store.record_request(request_id, {
                'request_id': request_id,
                'operation': operation,
                'state': 'in_progress',
                'started_at': self.clock(),
                'fingerprint': fingerprint,
            })

            manifest = self.store.require_ready()
            if not self.activation_supported:
                return self._finish(
                    request_id, 'refused',
                    'activation_unavailable: ' + getattr(self.backend, 'detail', 'unverified'),
                    None, None)
            if not isinstance(expected_revision, int) or isinstance(expected_revision, bool) \
                    or expected_revision != manifest['desired_revision']:
                return self._finish(
                    request_id, 'conflict',
                    f'expected_revision {expected_revision!r} != {manifest["desired_revision"]}',
                    None, None)

            revision = expected_revision
            if operation == 'restore':
                problems = self._restore_problems(manifest, target_revision)
                if problems:
                    return self._finish(request_id, 'refused', '; '.join(problems), None, None)
                revision = target_revision
            revision_state = self.store.load_revision(revision)
            digest = revision_state['meta']['generated_digest']
            preflight = getattr(self.backend, 'preflight', None)
            if preflight is not None:
                try:
                    preflight(revision, digest)
                except (ValueError, OSError) as exc:
                    return self._finish(request_id, 'refused', str(exc), None, revision)

            job_id = self.id_factory()
            journalled = False
            with self._owner_lock:
                # Refuse, never cancel somebody's existing operation just to
                # create a tuning hold. Fresh backend evidence is mandatory.
                if self.hold or getattr(self.owner, 'tuning_hold', False):
                    allowed, reason = False, 'busy: tuning owner held'
                else:
                    evidence = self.backend.probe()
                    allowed, reason = evaluate_evidence(evidence)
                    if allowed:
                        reserve = getattr(self.backend, 'reserve_navigation', None)
                        if reserve is not None:
                            # Journal BEFORE touching the remote admission
                            # mutex. A crash/timeout cannot orphan an invisible
                            # navigator reservation.
                            self._journal_job(
                                job_id, operation, expected_revision, revision, actor, digest,
                                JOB_PENDING, 'reserving navigation admission')
                            journalled = True
                            try:
                                reserve()
                            except AdmissionRefused as exc:
                                self._update_job(job_id, JOB_FAILED, str(exc))
                                return self._finish(
                                    request_id, 'refused', str(exc), job_id, revision)
                            except Exception:
                                self._set_hold(True, job_id)
                                self._update_job(job_id, JOB_UNKNOWN,
                                                 'navigation reservation outcome unknown')
                                return self._finish(
                                    request_id, 'unknown',
                                    'navigation reservation outcome unknown', job_id, revision)
                            allowed, reason = evaluate_evidence(self.backend.probe())
                            if not allowed:
                                try:
                                    self.backend.release_navigation_reservation()
                                except Exception:
                                    self._set_hold(True, job_id)
                                    self._update_job(job_id, JOB_UNKNOWN,
                                                     'reservation release outcome unknown')
                                    return self._finish(
                                        request_id, 'unknown',
                                        'reservation release outcome unknown', job_id, revision)
                                self._update_job(job_id, JOB_FAILED, reason)
                                return self._finish(
                                    request_id, 'refused', reason, job_id, revision)
                        self._set_hold(True, job_id)
            if not allowed:
                return self._finish(request_id, 'refused', reason, None, revision)
            if not journalled:
                self._journal_job(job_id, operation, expected_revision, revision, actor, digest,
                                  JOB_PENDING, 'queued')

            evidence = self.backend.probe()
            allowed, reason = evaluate_evidence(evidence)
            if not allowed:
                release = getattr(self.backend, 'release_navigation_reservation', None)
                if journalled and release is not None:
                    try:
                        release()
                    except Exception:
                        self._set_hold(True, job_id)
                        self._update_job(job_id, JOB_UNKNOWN,
                                         'reservation release outcome unknown')
                        return self._finish(
                            request_id, 'unknown', 'reservation release outcome unknown',
                            job_id, revision)
                self._update_job(job_id, JOB_FAILED, reason)
                self._set_hold(False, job_id)
                return self._finish(request_id, 'refused', reason, job_id, revision)

            self._update_job(job_id, JOB_RUNNING, 'reloading')
            try:
                # Recheck immediately before invoking the fixed operation.
                allowed, reason = evaluate_evidence(self.backend.probe())
                if not allowed:
                    self._update_job(job_id, JOB_UNKNOWN, reason)
                    return self._finish(
                        request_id, 'unknown', reason + '; hold retained', job_id, revision)
                result = self.backend.reload(sorted(SCOPES), revision, digest)
            except Exception as exc:  # an exception is ambiguous, never success
                self._update_job(job_id, JOB_UNKNOWN,
                                 'reload raised: ' + exc.__class__.__name__)
                return self._finish(request_id, 'unknown',
                                    'reload outcome ambiguous; hold retained', job_id, revision)

            if result.status == 'ok':
                try:
                    self.store.promote(revision=revision, scopes=SCOPES, actor=actor,
                                       detail=f'job {job_id}')
                    if operation == 'restore':
                        self.store._write_manifest(dict(
                            self.store.require_ready(), last_known_good=revision))
                    committed = getattr(self.backend, 'committed', None)
                    if committed is not None:
                        committed(revision, digest)
                    self._update_job(job_id, JOB_SUCCEEDED, result.detail or 'activated')
                except Exception:
                    try:
                        self._update_job(
                            job_id, JOB_UNKNOWN, 'commit outcome unknown; hold retained')
                    except Exception:
                        pass
                    return self._finish(
                        request_id, 'unknown', 'commit outcome unknown; hold retained',
                        job_id, revision)
                self._set_hold(False, job_id)
                return self._finish(request_id, 'ok', result.detail or 'activated',
                                    job_id, revision)
            if result.status == 'failed':
                self._update_job(
                    job_id, JOB_UNKNOWN,
                    result.detail or 'reload failed; runtime is unverified')
                return self._finish(
                    request_id, 'unknown',
                    result.detail or 'reload failed; hold retained', job_id, revision)
            # unknown/timeout: retain the hold, require explicit recovery
            self._update_job(job_id, JOB_UNKNOWN, result.detail or 'reload outcome unknown')
            return self._finish(request_id, 'unknown',
                                result.detail or 'reload outcome unknown', job_id, revision)

    def _restore_problems(self, manifest, target_revision):
        problems = []
        if not isinstance(target_revision, int) or isinstance(target_revision, bool):
            problems.append('restore_revision required')
            return problems
        if target_revision > manifest['desired_revision']:
            problems.append('restore_revision must not exceed the current revision')
        if target_revision != manifest.get('last_known_good'):
            problems.append('restore_revision is not the reviewed last-known-good revision')
        try:
            self.store.load_revision(target_revision)
        except StoreUnavailable as exc:
            problems.append('unknown restore_revision: ' + exc.code)
        return problems

    # ------------------------------------------------------------------
    # Jobs
    # ------------------------------------------------------------------
    def _journal_job(self, job_id, operation, expected_revision, revision, actor, digest,
                     state, detail):
        job = {
            'job_id': job_id,
            'operation': operation,
            'expected_revision': expected_revision,
            'revision': revision,
            'digest': digest,
            'actor': actor,
            'state': state,
            'detail': detail,
            'created_at': self.clock(),
            'updated_at': self.clock(),
        }
        self.store.record_job(job)
        with self.store.writer():
            manifest = self.store.require_ready()
            self.store._write_manifest(dict(manifest, latest_job=job_id))
        self.store._audit({'event': 'job', 'job_id': job_id, 'operation': operation,
                           'state': state, 'revision': revision, 'actor': actor,
                           'at': self.clock()})
        return job

    def _update_job(self, job_id, state, detail):
        job = self.store.get_job(job_id)
        if job is None:
            raise StoreError('unknown_job', job_id)
        job = dict(job, state=state, detail=detail, updated_at=self.clock())
        self.store.record_job(job)
        self.store._audit({'event': 'job', 'job_id': job_id, 'state': state,
                           'revision': job.get('revision'), 'at': self.clock()})
        return job

    def status(self, job_id=None):
        if job_id is None:
            return self.store.latest_job()
        return self.store.get_job(job_id) or self.store.get_request(job_id)

    def _finish(self, request_id, outcome, detail, job_id, revision):
        prior = self.store.get_request(request_id) or {}
        record = {
            'request_id': request_id,
            'state': 'done',
            'outcome': outcome,
            'detail': detail,
            'job_id': job_id,
            'revision': revision,
            'finished_at': self.clock(),
            'fingerprint': prior.get('fingerprint'),
        }
        self.store.record_request(request_id, record)
        return record

    # ------------------------------------------------------------------
    # Restart recovery
    # ------------------------------------------------------------------
    def recover(self):
        """On startup: an unsettled job must hold all motion and stay unknown."""
        job = self.store.latest_job()
        if job and job.get('state') in UNSETTLED_STATES:
            self._set_hold(True, job['job_id'])
            return job
        return None


def evidence_from_runtime(snapshot: dict) -> Evidence:
    """
    Map a plain runtime evidence dict to :class:`Evidence`.

    Unknown/absent keys stay ``None`` (busy). Non-boolean values are coerced
    conservatively: only literal ``True``/``False`` survive; anything else is
    unknown.
    """
    fields = {}
    for field in Evidence.__dataclass_fields__:
        value = snapshot.get(field) if isinstance(snapshot, dict) else None
        fields[field] = value if isinstance(value, bool) else None
    return Evidence(**fields)


__all__ = [
    'ActivationCoordinator',
    'Evidence',
    'FakeBackend',
    'FixedOperationPlan',
    'ReloadResult',
    'UnavailableBackend',
    'evaluate_evidence',
    'evidence_from_runtime',
    'JOB_PENDING',
    'JOB_RUNNING',
    'JOB_SUCCEEDED',
    'JOB_FAILED',
    'JOB_UNKNOWN',
    'UNSETTLED_STATES',
    'MUTATING_OPERATIONS',
    'HOST_SERVICES',
]
