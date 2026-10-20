"""
Robot-owned immutable tuning store with an atomic, fsynced manifest.

Layout under the persistent state base (``/var/lib/patrolbot/config``)::

    initialized                 sentinel: this store was explicitly initialised
    lock                        single-writer flock
    manifest.json               DESIRED revision + ACTIVE component revisions
    startup.json                committed ACTIVE startup artifact pointer
    startup/<scope>.yaml        generated config the launcher reads at boot
    revisions/<n>/overrides.yaml    immutable logical overrides
    revisions/<n>/meta.json          checksum + provenance
    revisions/<n>/generated/<scope>.yaml
    requests/<uuid>.json        durable idempotency / dedup ledger
    jobs/<uuid>.json            durable activation jobs
    audit.log                   append-only JSONL

Safety rules enforced here (and unit-tested without ROS):

* Only a *missing sentinel plus a totally empty directory* is a clean install.
  A present-but-corrupt sentinel/manifest, or a missing sentinel next to real
  state, fails closed. It is never silently reinitialised to faster defaults.
* Every mutation happens under one writer lock. Clients use optimistic
  compare-and-swap (``base_revision``); two admins get a conflict.
* All state writes are atomic: temp file in the same directory, fsync, rename,
  fsync the directory. A crash leaves either the old or the new bytes.
* Revisions are immutable directories. Rollback is a new audited activation.
* Duplicate request ids return the recorded outcome; a request that was
  journalled but never completed reports ``unknown`` and is never retried
  blindly.
* DESIRED and ACTIVE are separate. Boot reads ``startup.json`` (ACTIVE) only.
  A pending desired change never becomes startup config without promotion.

Pure Python + PyYAML. No ROS, no network.
"""

from __future__ import annotations

from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import stat
import tempfile
import threading
import time
import uuid

from . import tuning_spec
from .tuning_baseline import BaselineError, prepare_baseline
from .tuning_json import loads as strict_json_loads
from .tuning_spec import StrictYAMLError
from .tuning_yaml import strict_yaml_dump, strict_yaml_load

SCHEMA_VERSION = 1
UNINITIALIZED = 'uninitialized'
CORRUPT = 'corrupt'
READY = 'ready'

MAX_JSON_BYTES = 512 * 1024
MAX_REVISION = 2 ** 53 - 1
SCOPES = ('bringup', 'navigation')


class StoreError(ValueError):
    """Base for tuning-store failures; carries a stable ``code``."""

    def __init__(self, code: str, message: str = ''):
        super().__init__(message or code)
        self.code = code
        self.message = message or code

    def __str__(self):
        return f'{self.code}: {self.message}'


class StoreUnavailable(StoreError):
    """
    State is missing/corrupt or the durable directory is unusable.

    Callers must fail closed: this is never a licence to reset to defaults.
    """


def _sha256_bytes(data: bytes) -> str:
    return 'sha256:' + hashlib.sha256(data).hexdigest()


def _fsync_dir(path: Path) -> None:
    fd = os.open(path, os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0))
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def _atomic_write(path: Path, data: bytes) -> None:
    directory = path.parent
    fd, temporary = tempfile.mkstemp(dir=directory, prefix='.' + path.name + '.', suffix='.tmp')
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        _fsync_dir(directory)
    except BaseException:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def _read_bytes(path: Path, max_bytes: int) -> bytes:
    flags = os.O_RDONLY | os.O_NONBLOCK | getattr(os, 'O_NOFOLLOW', 0)
    try:
        fd = os.open(path, flags)
    except OSError as exc:
        raise StoreUnavailable('unreadable_state', f'{path.name}: {exc.strerror}') from exc
    try:
        if not stat.S_ISREG(os.fstat(fd).st_mode):
            raise StoreUnavailable('non_regular_state', path.name)
        raw = os.read(fd, max_bytes + 1)
    finally:
        os.close(fd)
    if len(raw) > max_bytes:
        raise StoreUnavailable('oversized_state', path.name)
    return raw


def _read_json(path: Path, max_bytes: int = MAX_JSON_BYTES):
    raw = _read_bytes(path, max_bytes)
    try:
        return strict_json_loads(raw.decode('utf-8'), max_bytes)
    except (ValueError, UnicodeError, RecursionError) as exc:
        raise StoreUnavailable('corrupt_json', f'{path.name}: {exc.__class__.__name__}') from exc


def _is_safe_revision(revision) -> bool:
    return (isinstance(revision, int) and not isinstance(revision, bool)
            and 0 <= revision <= MAX_REVISION)


class RevisionStore:
    def __init__(self, base_dir, registry, *, clock=None, package='patrolbot_bridge'):
        self.base = Path(base_dir)
        if self.base.is_symlink():
            raise StoreUnavailable('symlinked_state_dir', str(self.base))
        self.registry = registry
        self.index = tuning_spec.index_registry(registry)
        self.registry_digest = tuning_spec.registry_digest(registry)
        self.clock = clock or time.time
        self.package = package
        self._thread_lock = threading.RLock()
        self._admission_lock = threading.RLock()
        self._local = threading.local()

    # ------------------------------------------------------------------
    # Paths and low-level guards
    # ------------------------------------------------------------------
    @property
    def initialized_path(self) -> Path:
        return self.base / 'initialized'

    @property
    def manifest_path(self) -> Path:
        return self.base / 'manifest.json'

    @property
    def startup_path(self) -> Path:
        return self.base / 'startup.json'

    @property
    def startup_dir(self) -> Path:
        return self.base / 'startup'

    @property
    def revisions_dir(self) -> Path:
        return self.base / 'revisions'

    @property
    def requests_dir(self) -> Path:
        return self.base / 'requests'

    @property
    def jobs_dir(self) -> Path:
        return self.base / 'jobs'

    @property
    def audit_path(self) -> Path:
        return self.base / 'audit.log'

    @property
    def lock_path(self) -> Path:
        return self.base / 'lock'

    def revision_dir(self, revision: int) -> Path:
        if not _is_safe_revision(revision):
            raise StoreUnavailable('invalid_revision', repr(revision))
        path = self.revisions_dir / str(revision)
        self._reject_links(path)
        return path

    def _reject_links(self, path):
        current = Path(path)
        while current != current.parent:
            if current.is_symlink():
                raise StoreUnavailable('symlinked_state', str(current))
            current = current.parent

    @contextmanager
    def writer(self):
        """
        Single writer/transaction lock (flock + in-process reentrancy guard).

        Re-entrant within one thread so an outer transaction may call helpers
        that also take the writer; other threads and other processes block.
        """
        with self._thread_lock:
            depth = getattr(self._local, 'depth', 0)
            if depth == 0:
                self._reject_links(self.base)
                if not self.base.is_dir():
                    raise StoreUnavailable('durable_directory_unavailable', str(self.base))
                fd = os.open(self.lock_path,
                             os.O_RDWR | os.O_CREAT | os.O_NONBLOCK
                             | getattr(os, 'O_NOFOLLOW', 0), 0o600)
                try:
                    if not stat.S_ISREG(os.fstat(fd).st_mode):
                        raise StoreUnavailable('invalid_writer_lock')
                    fcntl.flock(fd, fcntl.LOCK_EX)
                except BaseException:
                    os.close(fd)
                    raise
                self._local.fd = fd
            self._local.depth = depth + 1
            try:
                yield
            finally:
                self._local.depth -= 1
                if self._local.depth == 0:
                    fd = self._local.fd
                    try:
                        fcntl.flock(fd, fcntl.LOCK_UN)
                    finally:
                        os.close(fd)
                    self._local.fd = None

    def _ensure_base(self) -> None:
        if self.base.exists() and not self.base.is_dir():
            raise StoreUnavailable('durable_directory_unavailable', str(self.base))
        self.base.mkdir(mode=0o700, parents=True, exist_ok=True)

    # ------------------------------------------------------------------
    # State discovery / fail-closed
    # ------------------------------------------------------------------
    def state(self) -> dict:
        """Return ``{status, ...}``; raise :class:`StoreUnavailable` on corruption."""
        self._reject_links(self.base)
        if not self.base.exists():
            # No base at all is a plausible clean install; it is never
            # auto-initialised, so callers still fail closed until an operator
            # explicitly initialises the store.
            return {'status': UNINITIALIZED}
        if not self.base.is_dir():
            raise StoreUnavailable('durable_directory_unavailable', str(self.base))
        sentinel_exists = self.initialized_path.exists() or self.initialized_path.is_symlink()
        manifest_exists = self.manifest_path.exists() or self.manifest_path.is_symlink()
        other_state = any(path.name != 'lock' for path in self.base.iterdir())
        if not sentinel_exists:
            if other_state:
                raise StoreUnavailable(
                    'corrupt_missing_sentinel',
                    'state present without the initialised sentinel; refusing to reset',
                )
            return {'status': UNINITIALIZED}
        sentinel = _read_json(self.initialized_path)
        if not isinstance(sentinel, dict) or sentinel.get('schema_version') != SCHEMA_VERSION:
            raise StoreUnavailable('corrupt_sentinel', 'unrecognized initialised sentinel')
        if sentinel.get('registry_digest') != self.registry_digest:
            raise StoreUnavailable(
                'incompatible_registry', 'explicit schema/registry migration required')
        if not manifest_exists:
            raise StoreUnavailable(
                'corrupt_missing_manifest', 'sentinel present but manifest is gone')
        manifest = _read_json(self.manifest_path)
        if not isinstance(manifest, dict) or manifest.get('schema_version') != SCHEMA_VERSION:
            raise StoreUnavailable('corrupt_manifest', 'unrecognized manifest')
        active = manifest.get('active_revisions')
        if (not _is_safe_revision(manifest.get('desired_revision'))
                or not isinstance(active, dict) or set(active) != set(SCOPES)
                or not all(_is_safe_revision(value) for value in active.values())
                or not _canonical_uuid(manifest.get('generation'))
                or not isinstance(manifest.get('pending_keys'), list)
                or any(key not in self.index for key in manifest['pending_keys'])
                or (manifest.get('latest_job') is not None
                    and not _canonical_uuid(manifest['latest_job']))):
            raise StoreUnavailable('corrupt_manifest', 'invalid revision/ownership metadata')
        return {'status': READY, 'sentinel': sentinel, 'manifest': manifest}

    def require_ready(self) -> dict:
        state = self.state()
        if state['status'] != READY:
            raise StoreUnavailable(UNINITIALIZED, 'tuning store has not been initialised')
        return state['manifest']

    def require_initialized(self) -> dict:
        """Alias used by mutations; same fail-closed contract as :meth:`require_ready`."""
        return self.require_ready()

    # ------------------------------------------------------------------
    # Revision read
    # ------------------------------------------------------------------
    def load_revision(self, revision: int) -> dict:
        directory = self.revision_dir(revision)
        if not directory.is_dir():
            raise StoreUnavailable('unknown_revision', str(revision))
        overrides_raw = _read_bytes(directory / 'overrides.yaml', 256 * 1024)
        try:
            overrides = strict_yaml_load(overrides_raw.decode('utf-8'))
        except (UnicodeDecodeError, StrictYAMLError) as exc:
            raise StoreUnavailable('corrupt_revision', f'{revision}: {exc}') from exc
        if not isinstance(overrides, dict):
            raise StoreUnavailable('corrupt_revision', f'{revision}: overrides is not a mapping')
        meta = _read_json(directory / 'meta.json')
        if not isinstance(meta, dict):
            raise StoreUnavailable('corrupt_revision', 'invalid revision metadata')
        if meta.get('defaults_digest') != tuning_spec.defaults_digest():
            raise StoreUnavailable(
                'incompatible_defaults',
                'reviewed default provenance changed; explicit migration required')
        if meta.get('compiler_digest') != tuning_spec.compiler_digest():
            raise StoreUnavailable(
                'incompatible_compiler', 'artifact compiler changed; explicit migration required')
        generated = {}
        self._reject_links(directory / 'generated')
        for name in sorted((directory / 'generated').glob('*.yaml')):
            generated[name.name] = _read_bytes(name, 256 * 1024).decode('utf-8')
        if set(generated) != {scope + '.yaml' for scope in SCOPES}:
            raise StoreUnavailable('corrupt_revision', 'missing or extra consumer scope')
        for pid, value in overrides.items():
            if tuning_spec.validate_change(self.index, pid, value):
                raise StoreUnavailable('corrupt_revision', 'invalid logical override')
        if overrides_raw != strict_yaml_dump(overrides).encode('utf-8'):
            raise StoreUnavailable('corrupt_revision', f'{revision}: overrides not canonical')
        if meta.get('overrides_sha256') != _sha256_bytes(overrides_raw):
            raise StoreUnavailable('checksum_mismatch', f'revision {revision}')
        expected_generated = {n: _sha256_bytes(t.encode('utf-8'))
                              for n, t in generated.items()}
        if meta.get('generated') != expected_generated:
            raise StoreUnavailable('checksum_mismatch', f'revision {revision} generated')
        if meta.get('generated_digest') != tuning_spec.artifacts_digest(generated):
            raise StoreUnavailable('checksum_mismatch', f'revision {revision} aggregate digest')
        return {'revision': revision, 'overrides': overrides, 'meta': meta, 'generated': generated}

    def desired_overrides(self) -> dict:
        manifest = self.require_ready()
        revision = manifest.get('desired_revision')
        if not _is_safe_revision(revision):
            raise StoreUnavailable('corrupt_manifest', 'desired_revision is invalid')
        return self.load_revision(revision)['overrides']

    def active_overrides(self):
        manifest = self.require_ready()
        active = manifest.get('active_revisions') or {}
        values = set(active.values()) if set(active) == set(SCOPES) else set()
        if not values:
            return None
        # Scopes are activated as one revision; a split is a partial success
        # and therefore no single revision is fully active.
        if len(values) != 1:
            return None
        return self.load_revision(next(iter(values)))['overrides']

    # ------------------------------------------------------------------
    # Clean initialisation
    # ------------------------------------------------------------------
    def initialize(self, *, actor, reason, provenance=None) -> dict:
        """Initialize reviewed defaults for explicitly controlled clean installs/fixtures."""
        return self._initialize(actor=actor, reason=reason, provenance=provenance)

    def initialize_from_baseline(self, bundle, *, actor, reason) -> dict:
        """
        Persist a complete, representable baseline without raising observed settings.

        This is a trusted local foundation API, not an authenticated transport.
        The production ROS boundary remains read-only. Importing a baseline
        records committed startup configuration; it does not certify live
        readback, activate running nodes or authorize motion.
        """
        if (not isinstance(actor, dict) or set(actor) != {'id', 'username', 'role'}
                or type(actor.get('id')) is not int
                or not isinstance(actor.get('username'), str)
                or not 1 <= len(actor['username']) <= 128
                or actor.get('role') != 'administrator'):
            raise StoreError('administrator_required', 'a trusted administrator actor is required')
        if not isinstance(reason, str) or not reason.strip() or len(reason) > 512:
            raise StoreError('invalid_reason', 'initialization needs a bounded explicit reason')
        actor = {key: actor[key] for key in ('id', 'username', 'role')}
        try:
            prepared = prepare_baseline(bundle, self.registry)
        except BaselineError as exc:
            raise StoreError('baseline_' + exc.code, exc.detail) from exc
        problems = tuning_spec.validate_overrides(
            self.index, {}, [{'id': pid, 'value': value}
                             for pid, value in prepared['overrides'].items()])
        if problems:
            raise StoreError(
                'baseline_unrepresentable', '; '.join(problems))
        provenance = {'baseline': prepared['provenance'], 'baseline_digest': prepared['digest']}
        return self._initialize(
            actor=actor, reason=reason, provenance=provenance,
            initial_values=prepared['overrides'], expected_artifacts=prepared['artifacts'],
            expected_defaults_digest=prepared['provenance']['defaults_digest'])

    def _initialize(self, *, actor, reason, provenance=None, initial_values=None,
                    expected_artifacts=None, expected_defaults_digest=None) -> dict:
        if not self.base.parent.is_dir():
            # The durable volume must already exist and be mounted; creating it
            # on the root filesystem would make a missing mount look like a
            # clean install and permit a fast reset.
            raise StoreUnavailable('durable_directory_unavailable',
                                   'state volume parent is not available')
        self._reject_links(self.base)
        self._ensure_base()
        with self.writer():
            if self.state()['status'] != UNINITIALIZED:
                raise StoreUnavailable(
                    'already_initialized', 'refusing to reset existing tuning state')
            # Seed the initial revision from the actual packaged consumer YAML,
            # not the registry catalog: the YAML is the deployment's real
            # starting configuration. Initialization stays unreachable in
            # production; this only changes controlled clean-install fixtures.
            defaults = tuning_spec.yaml_starting_values(self.index)
            if initial_values is not None:
                defaults = dict(initial_values)
            provenance = dict(provenance or {}, package=self.package,
                              registry_digest=self.registry_digest)
            self.revisions_dir.mkdir(mode=0o700, exist_ok=True)
            self.requests_dir.mkdir(mode=0o700, exist_ok=True)
            self.jobs_dir.mkdir(mode=0o700, exist_ok=True)
            revision = 1
            self._write_revision(revision, defaults, parent=None, actor=actor, reason=reason,
                                 provenance=provenance, expected_artifacts=expected_artifacts,
                                 expected_defaults_digest=expected_defaults_digest)
            # Validate immutable contents before recording initialization as
            # durable state. Any interrupted write is still refused on retry.
            validated = self.load_revision(revision)
            sentinel = {
                'schema_version': SCHEMA_VERSION,
                'created_revision': revision,
                'created_at': self.clock(),
                'registry_digest': self.registry_digest,
            }
            _atomic_write(self.initialized_path, json.dumps(sentinel, sort_keys=True).encode())
            manifest = {
                'schema_version': SCHEMA_VERSION,
                'desired_revision': revision,
                'active_revisions': {scope: revision for scope in SCOPES},
                'active_digest': validated['meta']['generated_digest'],
                'last_known_good': revision,
                'pending_keys': [],
                'generation': str(uuid.uuid4()),
                'latest_job': None,
                'updated_at': self.clock(),
                'provenance': provenance,
            }
            self._write_manifest(manifest)
            self._write_startup(validated)
            self._audit({
                'event': 'initialize',
                'actor': actor,
                'reason': reason,
                'revision': revision,
                'at': self.clock(),
            })
            return manifest

    def _write_revision(self, revision, overrides, *, parent, actor, reason, provenance,
                        expected_artifacts=None, expected_defaults_digest=None) -> dict:
        directory = self.revision_dir(revision)
        if directory.exists():
            raise StoreUnavailable('revision_exists', str(revision))
        overrides_text = strict_yaml_dump(overrides)
        overrides_bytes = overrides_text.encode('utf-8')
        artifacts = tuning_spec.generated_artifacts(overrides, self.index)
        defaults_digest = tuning_spec.defaults_digest()
        if expected_artifacts is not None and artifacts != expected_artifacts:
            raise StoreUnavailable(
                'baseline_changed', 'consumer configuration changed before commit')
        if expected_defaults_digest is not None and defaults_digest != expected_defaults_digest:
            raise StoreUnavailable('baseline_changed', 'packaged defaults changed before commit')
        (directory / 'generated').mkdir(mode=0o700, parents=True)
        _atomic_write(directory / 'overrides.yaml', overrides_bytes)
        generated_checksums = {}
        for name, text in sorted(artifacts.items()):
            data = text.encode('utf-8')
            _atomic_write(directory / 'generated' / name, data)
            generated_checksums[name] = _sha256_bytes(data)
        meta = {
            'schema_version': SCHEMA_VERSION,
            'revision': revision,
            'parent': parent,
            'created_at': self.clock(),
            'actor': actor,
            'reason': reason,
            'overrides_sha256': _sha256_bytes(overrides_bytes),
            'generated': generated_checksums,
            'generated_digest': tuning_spec.artifacts_digest(artifacts),
            'defaults_digest': defaults_digest,
            'compiler_digest': tuning_spec.compiler_digest(),
            'provenance': provenance,
        }
        _atomic_write(directory / 'meta.json', json.dumps(meta, sort_keys=True).encode())
        _fsync_dir(directory)
        _fsync_dir(self.revisions_dir)
        return meta

    def _revision_generated_digest(self, revision) -> str:
        return self.load_revision(revision)['meta']['generated_digest']

    # ------------------------------------------------------------------
    # Manifest / startup writes
    # ------------------------------------------------------------------
    def _write_manifest(self, manifest) -> None:
        manifest = dict(manifest, updated_at=self.clock())
        _atomic_write(self.manifest_path, json.dumps(manifest, sort_keys=True).encode())

    def _write_startup(self, revision_state) -> str:
        self._reject_links(self.startup_dir)
        if not self.startup_dir.is_dir():
            self.startup_dir.mkdir(mode=0o700, exist_ok=True)
        checksums = {}
        for name, text in sorted(revision_state['generated'].items()):
            data = text.encode('utf-8')
            _atomic_write(self.startup_dir / name, data)
            checksums[name] = _sha256_bytes(data)
        pointer = {
            'schema_version': SCHEMA_VERSION,
            'revision': revision_state['revision'],
            'generated_digest': revision_state['meta']['generated_digest'],
            'checksums': checksums,
            'written_at': self.clock(),
        }
        _atomic_write(self.startup_path, json.dumps(pointer, sort_keys=True).encode())
        return pointer['generated_digest']

    def read_startup(self) -> dict:
        """Return the committed ACTIVE startup pointer, or fail closed."""
        self.require_ready()
        pointer = _read_json(self.startup_path)
        if not isinstance(pointer, dict) or pointer.get('schema_version') != SCHEMA_VERSION:
            raise StoreUnavailable('corrupt_startup', 'unrecognized startup pointer')
        if not _is_safe_revision(pointer.get('revision')):
            raise StoreUnavailable('corrupt_startup', 'invalid startup revision')
        checksums = pointer.get('checksums')
        if (not isinstance(checksums, dict)
                or set(checksums) != {scope + '.yaml' for scope in SCOPES}):
            raise StoreUnavailable('corrupt_startup', 'missing startup checksums')
        revision = self.load_revision(pointer['revision'])
        if pointer.get('generated_digest') != revision['meta']['generated_digest']:
            raise StoreUnavailable('corrupt_startup', 'startup revision digest mismatch')
        self._reject_links(self.startup_dir)
        for name, expected in checksums.items():
            data = _read_bytes(self.startup_dir / name, 256 * 1024)
            if _sha256_bytes(data) != expected:
                raise StoreUnavailable('startup_checksum_mismatch', name)
        return pointer

    def navigation_admission(self):
        path = self.base / 'navigation-admission.json'
        if not path.exists() and not path.is_symlink():
            return None
        record = _read_json(path)
        if (not isinstance(record, dict) or type(record.get('pending')) is not bool
                or not _is_safe_revision(record.get('revision'))
                or not isinstance(record.get('map_identity'), str)
                or not record['map_identity']):
            raise StoreUnavailable('invalid_navigation_admission')
        revision = self.load_revision(record['revision'])
        if record.get('digest') != revision['meta']['generated_digest']:
            raise StoreUnavailable('navigation_admission_digest_mismatch')
        return record

    def record_navigation_admission(self, record):
        # This post-activation admission journal is written by the existing
        # epoch owner too. Never acquire the main revision writer while holding
        # EpochRuntime.lock (mutations take those locks in the opposite order).
        # Referenced revisions are immutable; manifest replacement is atomic.
        with self._admission_lock:
            self.require_ready()
            revision = self.load_revision(record['revision'])
            if record.get('digest') != revision['meta']['generated_digest']:
                raise StoreUnavailable('navigation_admission_digest_mismatch')
            _atomic_write(self.base / 'navigation-admission.json',
                          json.dumps(record, sort_keys=True).encode())

    # ------------------------------------------------------------------
    # Requests / jobs / audit
    # ------------------------------------------------------------------
    def _request_path(self, request_id) -> Path:
        self._reject_links(self.requests_dir)
        if not isinstance(request_id, str) or not _canonical_uuid(request_id):
            raise StoreError('bad_request_id', repr(request_id))
        return self.requests_dir / (request_id + '.json')

    def _job_path(self, job_id) -> Path:
        self._reject_links(self.jobs_dir)
        if not isinstance(job_id, str) or not _canonical_uuid(job_id):
            raise StoreError('bad_job_id', repr(job_id))
        return self.jobs_dir / (job_id + '.json')

    def get_request(self, request_id):
        path = self._request_path(request_id)
        if not path.exists():
            return None
        return _read_json(path)

    def get_job(self, job_id):
        path = self._job_path(job_id)
        if not path.exists():
            return None
        return _read_json(path)

    def latest_job(self):
        manifest = self.require_ready()
        job_id = manifest.get('latest_job')
        if not isinstance(job_id, str):
            return None
        return self.get_job(job_id)

    def record_request(self, request_id, record) -> None:
        self.requests_dir.mkdir(mode=0o700, exist_ok=True)
        _atomic_write(self._request_path(request_id), json.dumps(record, sort_keys=True).encode())

    def record_job(self, job) -> None:
        self.jobs_dir.mkdir(mode=0o700, exist_ok=True)
        _atomic_write(self._job_path(job['job_id']), json.dumps(job, sort_keys=True).encode())

    def _audit(self, record) -> None:
        line = json.dumps(record, sort_keys=True).encode() + b'\n'
        fd = os.open(self.audit_path, os.O_WRONLY | os.O_CREAT | os.O_APPEND
                     | os.O_NONBLOCK | getattr(os, 'O_NOFOLLOW', 0), 0o600)
        try:
            if not stat.S_ISREG(os.fstat(fd).st_mode):
                raise StoreUnavailable('invalid_audit_file')
            os.write(fd, line)
            os.fsync(fd)
        finally:
            os.close(fd)

    # ------------------------------------------------------------------
    # Save (durable desired revision)
    # ------------------------------------------------------------------
    def save(self, *, request_id, base_revision, changes, actor, reason='', operation='save'):
        with self.writer():
            manifest = self.require_ready()
            existing = self.get_request(request_id)
            fingerprint = hashlib.sha256(json.dumps(
                {'actor': actor, 'operation': operation, 'base_revision': base_revision,
                 'changes': changes}, sort_keys=True, allow_nan=False).encode()).hexdigest()
            if existing is not None:
                if existing.get('fingerprint') is None and existing.get('state') != 'done':
                    return self._dedup_result(existing, request_id)
                if existing.get('fingerprint') != fingerprint:
                    return {'request_id': request_id, 'outcome': 'refused',
                            'detail': 'request_id was used for a different request or actor'}
                return self._dedup_result(existing, request_id)
            record = {
                'request_id': request_id,
                'operation': operation,
                'state': 'in_progress',
                'started_at': self.clock(),
                'fingerprint': fingerprint,
            }
            self.record_request(request_id, record)
            base = self.load_revision(manifest['desired_revision'])['overrides']
            problems = tuning_spec.validate_overrides(self.index, base, changes)
            if problems:
                return self._finish_request(
                    request_id, 'refused', '; '.join(problems), manifest['desired_revision'])
            if (not _is_safe_revision(base_revision)
                    or base_revision != manifest['desired_revision']):
                return self._finish_request(
                    request_id, 'conflict',
                    f"base_revision {base_revision!r} != {manifest['desired_revision']}",
                    manifest['desired_revision'])
            existing_revisions = [int(path.name) for path in self.revisions_dir.iterdir()
                                  if path.name.isdigit()]
            revision = max([manifest['desired_revision'], *existing_revisions]) + 1
            if not _is_safe_revision(revision):
                raise StoreUnavailable('revision_exhausted')
            overrides = tuning_spec.merged_overrides(base, changes)
            provenance = dict(manifest.get('provenance') or {})
            self._write_revision(revision, overrides, parent=manifest['desired_revision'],
                                 actor=actor, reason=reason, provenance=provenance)
            active = self.active_overrides()
            pending = self._pending_keys(overrides, active)
            manifest = dict(manifest, desired_revision=revision, pending_keys=pending)
            self._write_manifest(manifest)
            self._audit({
                'event': 'save',
                'actor': actor,
                'reason': reason,
                'revision': revision,
                'base_revision': base_revision,
                'keys': sorted(c['id'] for c in changes),
                'before_desired': {c['id']: base.get(c['id']) for c in changes},
                'after_desired': {c['id']: overrides[c['id']] for c in changes},
                'committed_effective_limits': (
                    tuning_spec.effective_limits(active, self.index) if active else None),
                'runtime_readback_known': False,
                'at': self.clock(),
            })
            return self._finish_request(request_id, 'ok', 'saved desired revision', revision)

    def _dedup_result(self, existing, request_id):
        if existing.get('state') == 'done':
            return dict(existing, deduplicated=True)
        # Journalled but never completed: a crash may have happened after the
        # mutation began. Report unknown; never blindly retry.
        return {
            'request_id': request_id,
            'state': 'done',
            'outcome': 'unknown',
            'detail': 'request outcome unknown after interruption; query status',
            'revision': None,
            'deduplicated': True,
        }

    def _finish_request(self, request_id, outcome, detail, revision):
        prior = self.get_request(request_id) or {}
        record = {
            'request_id': request_id,
            'state': 'done',
            'outcome': outcome,
            'detail': detail,
            'revision': revision,
            'finished_at': self.clock(),
            'fingerprint': prior.get('fingerprint'),
        }
        self.record_request(request_id, record)
        return dict(record)

    def _pending_keys(self, desired, active):
        if active is None:
            return sorted(desired)
        keys = set(desired) | set(active)
        return sorted(k for k in keys if desired.get(k) != active.get(k))

    # ------------------------------------------------------------------
    # Promotion (activation success)
    # ------------------------------------------------------------------
    def promote(self, *, revision, scopes, actor, detail='') -> dict:
        if set(scopes) != set(SCOPES) or len(scopes) != len(SCOPES):
            raise StoreError(
                'partial_activation_unsupported', 'all consumer scopes must commit together')
        with self.writer():
            manifest = self.require_ready()
            state = self.load_revision(revision)
            previous = manifest.get('active_revisions') or {}
            digest = self._write_startup(state)
            active_revisions = dict(previous)
            for scope in scopes:
                if scope not in SCOPES:
                    raise StoreError('unknown_scope', scope)
                active_revisions[scope] = revision
            pending = self._pending_keys(
                self.load_revision(manifest['desired_revision'])['overrides'],
                state['overrides'],
            )
            last_good = manifest.get('last_known_good')
            if previous:
                candidates = [r for r in previous.values() if _is_safe_revision(r)]
                if candidates:
                    last_good = max(candidates)
            manifest = dict(
                manifest,
                active_revisions=active_revisions,
                active_digest=digest,
                last_known_good=last_good,
                pending_keys=pending,
            )
            self._write_manifest(manifest)
            self._audit({
                'event': 'promote',
                'actor': actor,
                'revision': revision,
                'scopes': sorted(scopes),
                'detail': detail,
                'at': self.clock(),
            })
            return manifest

    # ------------------------------------------------------------------
    # Snapshot helpers
    # ------------------------------------------------------------------
    def snapshot_state(self) -> dict:
        state = self.state()
        if state['status'] != READY:
            return {
                'status': state['status'],
                'desired_revision': None,
                'active_revision': None,
                'last_known_good': None,
                'known': False,
                'pending_keys': [],
                'generation': None,
                'latest_job': None,
            }
        manifest = state['manifest']
        active = manifest.get('active_revisions') or {}
        active_values = {r for r in active.values() if _is_safe_revision(r)}
        active_revision = next(iter(active_values)) if len(active_values) == 1 else None
        return {
            'status': READY,
            'desired_revision': manifest.get('desired_revision'),
            'active_revision': active_revision,
            'last_known_good': manifest.get('last_known_good'),
            'known': active_revision is not None,
            'pending_keys': list(manifest.get('pending_keys') or []),
            'generation': manifest.get('generation'),
            'latest_job': manifest.get('latest_job'),
        }


def _canonical_uuid(value: str) -> bool:
    try:
        parsed = uuid.UUID(value)
    except (ValueError, AttributeError, TypeError):
        return False
    return str(parsed) == value


__all__ = [
    'RevisionStore',
    'StoreError',
    'StoreUnavailable',
    'SCHEMA_VERSION',
    'UNINITIALIZED',
    'CORRUPT',
    'READY',
    'SCOPES',
]
