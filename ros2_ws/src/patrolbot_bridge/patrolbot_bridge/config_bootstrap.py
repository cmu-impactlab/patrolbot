"""
Dry-run bootstrap preflight for persistent robot tuning (read-only).

This module answers one question without touching the robot: *if* an operator were
to initialise the tuning store on this host, which reviewed default revision and
generated config digests would be proposed, and is the state directory in a
truly clean, uninitialised condition? It never initialises the store, creates a
directory, takes a writer lock, promotes startup config, imports ROS or contacts
a node.

This preview never initializes (``initialization_supported: false``). The
separate root-local ``tuning_initialize`` tool requires exact reviewed digests,
clean-install confirmation and stopped consumers. Unknown previous deployments
are not migrated by this diagnostic, and no runtime freshness is inferred.

Run it explicitly (no default paths are invented)::

    python -m patrolbot_bridge.config_bootstrap \
        --registry contracts/robot-tuning.registry.json \
        --state-dir /var/lib/patrolbot/config
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import stat

from . import tuning_spec
from .tuning_json import loads as strict_json_loads
from .tuning_store import READY, RevisionStore, StoreUnavailable

SCHEMA = 'patrolbot-tuning-bootstrap/1'
MAX_REGISTRY_BYTES = 512 * 1024
MAX_REPORT_BYTES = 64 * 1024
MAX_TEXT = 500
CLEAN = 'clean_uninitialized'
BLOCKED_REASON = ('preview is read-only; explicit local initialization requires review, '
                  'clean-install confirmation and all guarded consumers stopped')


class BootstrapRefusal(Exception):
    """An expected, reportable refusal; never a licence to reset state."""

    def __init__(self, code: str, detail: str = '', *, status: str = 'refused'):
        super().__init__(detail or code)
        self.code = code
        self.detail = detail or code
        self.status = status


def _bounded_text(value, limit=MAX_TEXT) -> str:
    text = '' if value is None else str(value)
    return text.replace('\r', ' ').replace('\x00', '')[:limit]


def _read_registry(path):
    """Read the registry within a hard byte budget, never following a symlink."""
    path = Path(path)
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | getattr(os, 'O_NOFOLLOW', 0))
    except OSError as exc:
        raise BootstrapRefusal('registry_unreadable', f'{path}: {exc.strerror}')
    try:
        if not stat.S_ISREG(os.fstat(fd).st_mode):
            raise BootstrapRefusal('registry_not_regular', str(path))
        raw = os.read(fd, MAX_REGISTRY_BYTES + 1)
    finally:
        os.close(fd)
    if len(raw) > MAX_REGISTRY_BYTES:
        raise BootstrapRefusal('registry_oversized', f'{path}: exceeds byte budget')
    try:
        text = raw.decode('utf-8')
    except UnicodeDecodeError:
        raise BootstrapRefusal('registry_not_utf8', str(path))
    try:
        return strict_json_loads(text, MAX_REGISTRY_BYTES)
    except (ValueError, RecursionError) as exc:
        raise BootstrapRefusal('registry_invalid_json', f'{path}: {exc.__class__.__name__}')


def _candidate(index, refusals):
    """
    Describe the (unwritten) revision 1 the store would build, or why not.

    Validation is per-default. The patch dependency check is deliberately not run
    over every default: reviewed producers above the global ceiling are emitted at
    ``min(cap, target)`` and are reported as clamps, not candidate defects.
    """
    # The candidate seed is the actual packaged consumer YAML, not the registry
    # catalog: the YAML is the deployment's real starting configuration.
    defaults = tuning_spec.yaml_starting_values(index)
    result = _empty_candidate()
    result['revision'] = 1
    result['source'] = 'packaged_yaml_starting_values'
    result['editable_parameters'] = len(defaults)
    problems = []
    for pid, value in sorted(defaults.items()):
        problems.extend(f'{pid}: {reason}'
                        for reason in tuning_spec.validate_change(index, pid, value))
    result['validation'] = 'rejected' if problems else 'ok'
    result['reasons'] = [_bounded_text(problem) for problem in problems]
    targets = tuning_spec.speed_producer_targets(defaults, index)
    for pid, effective in sorted(targets.items()):
        reviewed = defaults.get(pid, index[pid]['default'])
        if reviewed > effective + 1e-9:
            result['clamped_producers'].append(
                {'id': pid, 'reviewed': reviewed, 'effective': effective})
    try:
        artifacts = tuning_spec.generated_artifacts(defaults, index)
        result['defaults_digest'] = tuning_spec.defaults_digest()
        result['config_digest'] = tuning_spec.artifacts_digest(artifacts)
        result['artifacts'] = sorted(artifacts)
    except (tuning_spec.SpecError, tuning_spec.StrictYAMLError, OSError) as exc:
        result['validation'] = 'unavailable'
        result['reasons'].append('candidate_artifacts_unavailable: ' + _bounded_text(exc))
        refusals.append('candidate_artifacts_unavailable: packaged defaults unreadable')
    return result


def _summary(report) -> str:
    status = report['bootstrap_status']
    lines = [
        'Persistent tuning bootstrap preflight (dry-run; no state was written).',
        f'Status {status}: initialization_supported=false, activatable=false.',
    ]
    candidate = report['candidate_defaults']
    if status == CLEAN:
        lines.append(f"Proposed revision {candidate['revision']} "
                     f"({candidate['source']}): {candidate['validation']}.")
        if candidate['config_digest']:
            lines.append('Candidate config digest ' + candidate['config_digest'])
    lines.extend('Refusal: ' + refusal for refusal in report['refusals'])
    return _bounded_text('\n'.join(lines))


def _finalize(report) -> dict:
    report['summary'] = _summary(report)
    if len(json.dumps(report, sort_keys=True, allow_nan=False).encode('utf-8')) > MAX_REPORT_BYTES:
        report = _empty_report('', '')
        report['bootstrap_status'] = 'refused'
        report['refusals'] = ['report_too_large: refusing an unbounded candidate report']
        report['summary'] = _summary(report)
    return report


def _empty_candidate() -> dict:
    return {
        'revision': None,
        'source': None,
        'editable_parameters': 0,
        'defaults_digest': None,
        'config_digest': None,
        'artifacts': [],
        'validation': 'unknown',
        'reasons': [],
        'clamped_producers': [],
    }


def _empty_report(registry_file, state_dir) -> dict:
    return {
        'schema': SCHEMA,
        'operation': 'bootstrap_preview',
        'bootstrap_status': 'unavailable',
        'write_performed': False,
        'activatable': False,
        'initialization_supported': False,
        'initialization_blocked_reason': BLOCKED_REASON,
        'live_readback': False,
        'migration_eligible': False,
        'registry_file': _bounded_text(registry_file),
        'state_dir': _bounded_text(state_dir),
        'registry_digest': None,
        'parameter_count': None,
        'state_parent_available': None,
        'candidate_defaults': _empty_candidate(),
        'refusals': [],
        'notes': [
            'No ROS/network contact; no live parameter or odometry readback was performed.',
            'Candidate digests cover reviewed defaults merged onto locally packaged defaults '
            'only; they are provenance hints, not observed runtime state.',
            'This preview never initializes; tuning_initialize is a separate guarded local '
            'administrator operation, not a deployment migration shortcut.',
        ],
    }


def _refuse(report, status, code, detail):
    report['bootstrap_status'] = status
    report['refusals'].append(f'{code}: {_bounded_text(detail)}')
    return _finalize(report)


def build_plan(registry_file, state_dir) -> dict:
    """Return a bounded, JSON-safe report; never creates, resets or promotes state."""
    report = _empty_report(registry_file, state_dir)
    try:
        registry = _read_registry(registry_file)
        try:
            tuning_spec.load_registry(registry)
            index = tuning_spec.index_registry(registry)
        except (tuning_spec.SpecError, KeyError, TypeError, ValueError) as exc:
            raise BootstrapRefusal('registry_invalid', str(exc))
        report['registry_digest'] = tuning_spec.registry_digest(registry)
        report['parameter_count'] = len(index)
        try:
            store = RevisionStore(state_dir, registry)
        except StoreUnavailable as exc:
            raise BootstrapRefusal('state_' + exc.code, exc.message, status='damaged')
        report['state_parent_available'] = store.base.parent.is_dir()
        if not report['state_parent_available']:
            report['notes'].append(
                'State volume parent is missing; initialization is not possible here.')
    except BootstrapRefusal as exc:
        return _refuse(report, exc.status, exc.code, exc.detail)
    try:
        state = store.state()
    except StoreUnavailable as exc:
        return _refuse(report, 'damaged', exc.code, exc.message + '; refusing to reset')
    except OSError as exc:
        return _refuse(report, 'unavailable', 'state_unreadable', str(exc))
    if state['status'] != 'uninitialized':
        report['current'] = {
            'status': READY,
            'desired_revision': state['manifest'].get('desired_revision'),
            'active_revisions': dict(state['manifest'].get('active_revisions') or {}),
        }
        return _refuse(report, 'already_initialized', 'already_initialized',
                       'refusing to reset existing tuning state')
    report['bootstrap_status'] = CLEAN
    try:
        report['candidate_defaults'] = _candidate(index, report['refusals'])
    except (KeyError, TypeError, ValueError, OSError) as exc:
        return _refuse(report, 'refused', 'candidate_invalid', str(exc))
    if report['candidate_defaults']['validation'] != 'ok' or report['refusals']:
        report['bootstrap_status'] = 'refused'
    return _finalize(report)


def _serialize(report) -> str:
    return json.dumps(_finalize(report), sort_keys=True, allow_nan=False)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description='Read-only tuning bootstrap preflight.')
    parser.add_argument('--registry', required=True, help='validated registry JSON file')
    parser.add_argument('--state-dir', required=True, help='persistent tuning state directory')
    args = parser.parse_args(argv)
    report = build_plan(args.registry, args.state_dir)
    print(_serialize(report))
    return 0 if report['bootstrap_status'] == CLEAN else 2


if __name__ == '__main__':
    raise SystemExit(main())


__all__ = ['SCHEMA', 'MAX_REGISTRY_BYTES', 'MAX_REPORT_BYTES', 'CLEAN',
           'BLOCKED_REASON', 'BootstrapRefusal', 'build_plan', 'main']
