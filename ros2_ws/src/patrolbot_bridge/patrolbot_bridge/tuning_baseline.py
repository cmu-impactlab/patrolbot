"""
Pure bounded validator for a supplied tuning baseline bundle.

It turns an operator-supplied capture of the current logical values and
rendered consumer YAML into a deterministic revision candidate, without
writing, contacting or authorising anything. The safety property is exact
representability: a candidate is accepted only when the reviewed generator
reproduces the supplied artifacts from the supplied logical values, compared
as canonical typed documents. A missing value, unknown field or a
source-specific setting the generator cannot reproduce is a refusal, never a
default, widening or clamp.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
import uuid

from . import tuning_spec
from .tuning_json import loads as strict_json_loads
from .tuning_yaml import strict_yaml_dump, strict_yaml_load, StrictYAMLError

CONTRACT = 'patrolbot-tuning-baseline'
VERSION = 1
EVIDENCE_KIND = 'supplied_baseline_bundle'
MAX_BUNDLE_BYTES = 1024 * 1024
ARTIFACT_SCOPES = ('bringup.yaml', 'navigation.yaml')
BUNDLE_FIELDS = frozenset({
    'contract',
    'version',
    'capture_id',
    'source_session_id',
    'registry_digest',
    'defaults_digest',
    'observed_values',
    'observed_artifacts',
})
_SPEED_KINDS = tuning_spec.LINEAR_KINDS | tuning_spec.ANGULAR_KINDS
_CEILINGS = frozenset({tuning_spec.LINEAR_CEILING, tuning_spec.ANGULAR_CEILING})


class BaselineError(ValueError):
    """A supplied baseline was malformed, stale or not exactly representable."""

    def __init__(self, code, detail=''):
        super().__init__(detail or code)
        self.code = code
        self.detail = detail or code

    def __str__(self):
        return f'{self.code}: {self.detail}'


def editable_ids(index) -> list:
    """Logical ids a clean revision stores, using the store's exact filter."""
    return [pid for pid, entry in index.items()
            if entry.get('accepts_edits') is True
            and entry.get('protected') is not True
            and entry.get('not_applicable') is not True]


def _finite(value) -> bool:
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def _canonical_json(value) -> str:
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def _is_canonical_uuid(value) -> bool:
    if not isinstance(value, str):
        return False
    try:
        parsed = uuid.UUID(value)
    except (ValueError, AttributeError, TypeError):
        return False
    return str(parsed) == value


def _coerce(bundle):
    """Return a parsed bundle, rejecting non-JSON and hostile encodings."""
    if isinstance(bundle, str):
        text = bundle
    elif isinstance(bundle, dict):
        try:
            text = json.dumps(bundle, sort_keys=True)
        except (TypeError, ValueError) as exc:
            raise BaselineError('invalid_bundle', 'bundle is not JSON') from exc
    else:
        raise BaselineError('invalid_bundle', 'bundle must be a JSON object or text')
    try:
        return strict_json_loads(text, MAX_BUNDLE_BYTES)
    except (ValueError, UnicodeError, RecursionError) as exc:
        raise BaselineError('invalid_bundle', str(exc)) from exc


def _require_fields(data) -> None:
    if not isinstance(data, dict):
        raise BaselineError('invalid_bundle', 'bundle is not a JSON object')
    keys = set(data)
    if keys != BUNDLE_FIELDS:
        unknown = sorted(keys - BUNDLE_FIELDS)
        missing = sorted(BUNDLE_FIELDS - keys)
        raise BaselineError('invalid_bundle', f'unknown={unknown!r} missing={missing!r}')


def _check_scalars(data: dict, registry) -> None:
    if data.get('contract') != CONTRACT:
        raise BaselineError('unsupported_contract', repr(data.get('contract')))
    version = data.get('version')
    if type(version) is not int or version != VERSION:
        raise BaselineError('unsupported_version', repr(version))
    for field in ('capture_id', 'source_session_id'):
        if not _is_canonical_uuid(data.get(field)):
            raise BaselineError('invalid_' + field, repr(data.get(field)))
    expected = (tuning_spec.registry_digest(registry), tuning_spec.defaults_digest())
    for field, wanted in zip(('registry_digest', 'defaults_digest'), expected):
        if data.get(field) != wanted:
            raise BaselineError(field + '_mismatch', f'expected {wanted}')


def _check_values(index: dict, values):
    if not isinstance(values, dict):
        raise BaselineError('invalid_observed_values', 'not a mapping')
    wanted = set(editable_ids(index))
    if set(values) != wanted:
        missing = sorted(wanted - set(values))
        extra = sorted(set(values) - wanted)
        raise BaselineError('observed_scope_mismatch',
                            f'missing={missing!r} extra={extra!r}')
    problems = []
    for pid in sorted(values):
        problems.extend(f'{pid}: {reason}'
                        for reason in tuning_spec.validate_change(index, pid, values[pid]))
    if problems:
        raise BaselineError('invalid_observed_value', '; '.join(problems))
    return values


def _check_coherence(index: dict, values: dict) -> None:
    problems = []
    for source, target in tuning_spec.dependency_edges(index):
        left = values.get(source, index[source]['default'])
        right = values.get(target, index[target]['default'])
        if not (_finite(left) and _finite(right)):
            problems.append(f'{source} <= {target} is non-numeric')
        elif left > right and not (index[source].get('value_kind') in _SPEED_KINDS
                                   and target in _CEILINGS):
            # The generator intentionally emits min(cap, target) for speed
            # producers, so a producer above its global cap is representable.
            # Any other inversion is not; it is refused, never clamped.
            problems.append(f'{source}={left} > {target}={right}')
    for pid, entry in index.items():
        for dep in entry.get('dependencies', []):
            if dep.get('kind') != 'sum_equals':
                continue
            left = values.get(pid, entry['default'])
            right = values.get(dep['target'], index[dep['target']]['default'])
            tolerance = dep.get('tolerance', 1e-9)
            if (not (_finite(left) and _finite(right))
                    or abs(left + right - dep['equals']) > tolerance):
                problems.append('probability mixture is not coherent')
    if problems:
        raise BaselineError('dependency_violation', '; '.join(problems))


def _first_difference(expected, observed, path='$') -> str:
    if isinstance(expected, dict) and isinstance(observed, dict):
        for key in sorted(set(expected) | set(observed)):
            if key not in expected or key not in observed:
                return f'{path}.{key} keys differ'
            found = _first_difference(expected[key], observed[key], f'{path}.{key}')
            if found:
                return found
    elif isinstance(expected, list) and isinstance(observed, list):
        if len(expected) != len(observed):
            return f'{path} length differs'
        for position, (left, right) in enumerate(zip(expected, observed)):
            found = _first_difference(left, right, f'{path}[{position}]')
            if found:
                return found
    elif type(expected) is not type(observed) or expected != observed:
        return f'{path} expected {expected!r} got {observed!r}'
    return ''


def _check_artifacts(index: dict, overrides: dict, observed):
    generated = tuning_spec.generated_artifacts(overrides, index)
    if not isinstance(observed, dict):
        raise BaselineError('invalid_observed_artifacts', 'not a mapping')
    if set(observed) != set(ARTIFACT_SCOPES):
        raise BaselineError('observed_scope_mismatch',
                            f'expected {sorted(ARTIFACT_SCOPES)!r} '
                            f'got {sorted(observed)!r}')
    for name in ARTIFACT_SCOPES:
        raw = observed[name]
        if not isinstance(raw, str):
            raise BaselineError('invalid_observed_artifact', name + ' is not text')
        try:
            document = strict_yaml_load(raw)
        except StrictYAMLError as exc:
            raise BaselineError('invalid_observed_artifact', f'{name}: {exc}') from exc
        if not isinstance(document, dict):
            raise BaselineError('invalid_observed_artifact', name + ' is not a mapping')
        expected = strict_yaml_load(generated[name])
        if strict_yaml_dump(document) != strict_yaml_dump(expected):
            raise BaselineError('artifact_mismatch',
                                f'{name}: {_first_difference(expected, document)}')
    return generated


def prepare_baseline(bundle, registry) -> dict:
    """
    Validate a supplied baseline and return a deterministic revision candidate.

    Returns ``{'overrides', 'artifacts', 'provenance', 'digest'}``. Raises
    :class:`BaselineError` (with stable ``.code`` and ``.detail``) on a
    malformed field, stale digest, incoherent dependency, missing value or an
    artifact the packaged generator cannot exactly reproduce. The only I/O is
    reading the reviewed packaged defaults; nothing is written or activated.
    """
    index = tuning_spec.index_registry(registry)
    data = _coerce(bundle)
    _require_fields(data)
    _check_scalars(data, registry)
    values = _check_values(index, data['observed_values'])
    _check_coherence(index, values)
    overrides = copy.deepcopy(values)
    artifacts = _check_artifacts(index, overrides, data['observed_artifacts'])
    digest = 'sha256:' + hashlib.sha256(_canonical_json(data).encode('utf-8')).hexdigest()
    provenance = {
        'evidence_kind': EVIDENCE_KIND,
        'capture_id': data['capture_id'],
        'source_session_id': data['source_session_id'],
        'registry_digest': data['registry_digest'],
        'defaults_digest': data['defaults_digest'],
        'baseline_digest': digest,
        'runtime_verified': False,
        'authorized': False,
    }
    return {'overrides': overrides, 'artifacts': artifacts,
            'provenance': provenance, 'digest': digest}


__all__ = [
    'BaselineError',
    'CONTRACT',
    'VERSION',
    'EVIDENCE_KIND',
    'MAX_BUNDLE_BYTES',
    'ARTIFACT_SCOPES',
    'BUNDLE_FIELDS',
    'editable_ids',
    'prepare_baseline',
]
