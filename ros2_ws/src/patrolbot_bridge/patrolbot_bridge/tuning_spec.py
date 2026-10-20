"""
Canonical tuning catalog helpers: validation, speed mapping and generation.

Pure Python. No ROS, no filesystem mutation. This module is the offline
"compiler" between the reviewed registry (``contracts/robot-tuning.registry.json``)
and the deterministic consumer YAML that the robot would load.

The important safety property is *effective <= source target and <= global cap*:
a partially configured producer can never exceed the active policy, and lowering
a global ceiling lowers every dependent consumer without silently raising any
source-specific lower target. Nothing here activates anything; it only computes
values and rejects bad proposals.
"""

from __future__ import annotations

from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path

from .tuning_yaml import strict_yaml_dump, strict_yaml_load, StrictYAMLError

CAPABILITY = 'robot_tuning_v1'
CONTRACT = 'patrolbot-robot-tuning'
CONTRACT_VERSION = 1

LINEAR_CEILING = 'max_linear_m_s'
ANGULAR_CEILING = 'max_angular_rad_s'

# Consumer scopes and the nodes each generated artifact carries. The scope name
# is the restart/reload unit an activation job targets; the source file is the
# packaged reviewed default the overlay is merged onto (RewrittenYaml-style).
CONSUMER_SCOPES = {
    'bringup': {
        'source_file': 'ros2_ws/src/patrolbot-launch/param/defaults/smoother.yaml',
        'nodes': ('teleop_velocity_smoother',),
    },
    'navigation': {
        'source_file': 'ros2_ws/src/patrolbot_navigation/config/nav2_params.yaml',
        'nodes': (
            'amcl',
            'controller_server',
            'velocity_smoother',
            'behavior_server',
            'patrolbot_joy_teleop',
        ),
    },
}

# F710 in-memory trim parameters live in patrolbot_joy_teleop. Their *caps* are
# sourced from the effective ceilings; the trims start clamped to the cap so a
# cached value can never bootstrap a higher speed. ``min_*`` are the smallest
# step, clamped down when the ceiling itself is lowered below them.
F710_NODE = 'patrolbot_joy_teleop'
F710_LINEAR_MIN = 0.05
F710_ANGULAR_MIN = 0.1

# Angular-producer keys that the reviewed source defaults exceed. They are
# always emitted as min(cap, target) so the generated configuration is coherent
# even though the registry records the upstream mismatch.
LINEAR_KINDS = frozenset({'speed'})
ANGULAR_KINDS = frozenset({'angular_speed'})

_INCLUSIVE_TYPE = {'integer': int, 'number': (int, float), 'boolean': bool, 'string': str}


class SpecError(ValueError):
    """A proposal violated the registry contract."""


def load_registry(registry):
    """Return a validated registry mapping, or raise :class:`SpecError`."""
    if not isinstance(registry, dict):
        raise SpecError('registry must be a mapping')
    if registry.get('contract') != CONTRACT or registry.get('version') != CONTRACT_VERSION:
        raise SpecError('unsupported registry contract/version')
    params = registry.get('parameters')
    if not isinstance(params, list):
        raise SpecError('registry parameters must be a list')
    return registry


def registry_digest(registry) -> str:
    """Stable sha256 over the exact registry bytes semantics."""
    canonical = json.dumps(registry, sort_keys=True, separators=(',', ':')).encode()
    return 'sha256:' + hashlib.sha256(canonical).hexdigest()


def index_registry(registry) -> dict:
    load_registry(registry)
    index: dict = {}
    for entry in registry['parameters']:
        pid = entry['id']
        if pid in index:
            raise SpecError('duplicate registry id: ' + pid)
        index[pid] = entry
    for required in (LINEAR_CEILING, ANGULAR_CEILING):
        if required not in index:
            raise SpecError('registry is missing the speed ceiling ' + required)
    return index


def reviewed_defaults(index: dict) -> dict:
    return {pid: entry['default'] for pid, entry in index.items()}


def dependency_edges(index: dict) -> list:
    """All ``source <= target`` edges, from every entry's declared dependencies."""
    edges = []
    for pid, entry in index.items():
        for dep in entry.get('dependencies') or []:
            if not isinstance(dep, dict):
                continue
            target = dep.get('target')
            kind = dep.get('kind')
            if kind == 'less_than_or_equal' and target in index:
                edges.append((pid, target))
    return edges


def _is_strict_int(value) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _finite_number(value) -> bool:
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and abs(value) <= 2**63 - 1 and math.isfinite(value))


def _value_ok(entry: dict, value) -> bool:
    declared = entry.get('type')
    if declared == 'integer':
        return _is_strict_int(value)
    if declared == 'number':
        return _finite_number(value)
    if declared == 'boolean':
        return isinstance(value, bool)
    if declared == 'string':
        return isinstance(value, str)
    if declared == 'enum':
        return isinstance(value, str)
    return False


def _check_bounds(entry: dict, value) -> bool:
    bounds = entry.get('bounds') or {}
    lower, upper = bounds.get('lower'), bounds.get('upper')
    if lower is not None:
        if value < lower or (value == lower and not bounds.get('lower_inclusive', True)):
            return False
    if upper is not None:
        if value > upper or (value == upper and not bounds.get('upper_inclusive', True)):
            return False
    return True


def validate_change(index: dict, pid, value) -> list:
    """Return a list of human-readable refusal reasons for one change."""
    if not isinstance(pid, str) or pid not in index:
        return ['unknown_logical_id: ' + repr(pid)]
    entry = index[pid]
    if entry.get('protected') is True:
        return ['protected_parameter: ' + pid]
    if entry.get('not_applicable') is True:
        return ['not_applicable: ' + pid]
    if entry.get('accepts_edits') is not True:
        return ['not_editable: ' + pid]
    if isinstance(value, bool) and entry.get('type') == 'number':
        return ['bool_as_number: ' + pid]
    if not _value_ok(entry, value):
        return ['bad_type: ' + pid]
    if not _check_bounds(entry, value):
        return ['out_of_bounds: ' + pid]
    return []


def merged_overrides(base: dict, changes: list) -> dict:
    result = dict(base)
    for change in changes:
        result[change['id']] = change['value']
    return result


def validate_overrides(index: dict, base: dict, changes: list) -> list:
    """
    Validate a whole patch against the registry and dependency closure.

    Every dependency edge incident to a changed id (either endpoint) is
    re-evaluated against the merged base+patch, so a one-sided edit cannot
    bypass a constraint. Edges untouched by the patch are left alone.
    """
    problems: list = []
    seen: set = set()
    for change in changes:
        pid = change.get('id') if isinstance(change, dict) else None
        if not isinstance(change, dict) or set(change) != {'id', 'value'}:
            problems.append('bad_type: change must contain id and value')
            continue
        if isinstance(pid, str):
            if pid in seen:
                problems.append('duplicate_change_id: ' + pid)
            seen.add(pid)
        problems.extend(validate_change(index, pid, change.get('value')))
    if problems:
        return problems
    merged = merged_overrides(base, changes)
    touched = {c['id'] for c in changes}
    for source, target in dependency_edges(index):
        if source not in touched and target not in touched:
            continue
        left = merged.get(source, index[source]['default'])
        right = merged.get(target, index[target]['default'])
        if not (_finite_number(left) and _finite_number(right)):
            problems.append(f'dependency_violation: {source} <= {target} (non-numeric)')
        elif left > right:
            # Speed consumers are generated from min(logical target, ceiling),
            # preserving the operator's slower cruise target without forcing
            # unrelated manual changes when a global cap is reduced.
            if not (source not in touched
                    and index[source].get('value_kind') in LINEAR_KINDS | ANGULAR_KINDS):
                problems.append(f'dependency_violation: {source}={left} > {target}={right}')
    for pid, entry in index.items():
        for dep in entry.get('dependencies', []):
            if dep.get('kind') != 'sum_equals' or not ({pid, dep['target']} & touched):
                continue
            left = merged.get(pid, entry['default'])
            right = merged.get(dep['target'], index[dep['target']]['default'])
            if (not (_finite_number(left) and _finite_number(right))
                    or abs(left + right - dep['equals']) > dep.get('tolerance', 1e-9)):
                problems.append('dependency_violation: probability mixture')
    return problems


def effective_limits(overrides: dict, index: dict) -> dict:
    """Global ceilings, bounded by the reviewed hard upper envelope."""
    result = {}
    ceilings = ((LINEAR_CEILING, 'max_linear_m_s'), (ANGULAR_CEILING, 'max_angular_rad_s'))
    for pid, ceiling in ceilings:
        value = overrides.get(pid, index[pid]['default'])
        result[ceiling] = float(value)
    return result


def speed_producer_targets(overrides: dict, index: dict) -> dict:
    """
    Map every speed/angular producer to its *effective* generated value.

    ``effective = min(global_cap, source_target)`` where ``source_target`` is
    the override when present, otherwise the reviewed default. The final
    ``teleop_velocity_smoother`` is a ceiling consumer, not a producer, and is
    handled by the caller as ``cap`` directly.
    """
    limits = effective_limits(overrides, index)
    targets = {}
    for pid, entry in index.items():
        kind = entry.get('value_kind')
        if kind in LINEAR_KINDS:
            cap = limits['max_linear_m_s']
        elif kind in ANGULAR_KINDS:
            cap = limits['max_angular_rad_s']
        else:
            continue
        target = overrides.get(pid, entry['default'])
        targets[pid] = float(min(cap, float(target)))
    return targets


def _nest(flat: dict) -> dict:
    tree: dict = {}
    for dotted, value in flat.items():
        cursor = tree
        parts = dotted.split('.')
        for part in parts[:-1]:
            cursor = cursor.setdefault(part, {})
        cursor[parts[-1]] = value
    return tree


def _node_params(index: dict, overrides: dict, node: str) -> dict:
    """Explicitly overridden, non-indexed params for one node, nested."""
    flat: dict = {}
    for pid, entry in index.items():
        source = entry.get('source') or {}
        if source.get('node') != node:
            continue
        if entry.get('protected') is True or entry.get('not_applicable') is True:
            continue
        if entry.get('accepts_edits') is not True:
            continue
        if source.get('index') is not None:
            continue
        if pid in overrides:
            flat[source['key']] = overrides[pid]
    return _nest(flat)


def _producer_values(index: dict, overrides: dict, node: str) -> dict:
    """
    Every speed/angular producer for one node at min(cap, target).

    Indexed source keys (for example the Nav2 smoother ``max_velocity`` triple)
    are emitted as a full list so the generated YAML is a valid ROS parameter;
    unknown slots stay ``0.0`` (the lateral component must remain zero).
    """
    limits = effective_limits(overrides, index)
    scalar: dict = {}
    indexed: dict = {}
    for pid, entry in index.items():
        value_kind = entry.get('value_kind')
        if value_kind in LINEAR_KINDS:
            cap = limits['max_linear_m_s']
        elif value_kind in ANGULAR_KINDS:
            cap = limits['max_angular_rad_s']
        else:
            continue
        source = entry.get('source') or {}
        if source.get('node') != node:
            continue
        value = float(min(cap, float(overrides.get(pid, entry['default']))))
        if source.get('index') is None:
            scalar[source['key']] = value
        else:
            indexed.setdefault(source['key'], {})[source['index']] = value
    for key, slots in indexed.items():
        values = [0.0] * (max(slots) + 1)
        for position, value in slots.items():
            values[position] = value
        scalar[key] = values
    return _nest(scalar)


def _deep_merge(base: dict, patch: dict) -> dict:
    for key, value in patch.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            base[key] = _deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def bringup_overlay(overrides: dict, index: dict) -> dict:
    """
    Bring-up scope: the final teleop smoother takes the global ceilings.

    Reverse signs are symmetric and the lateral component is zero.
    """
    limits = effective_limits(overrides, index)
    lin, ang = limits['max_linear_m_s'], limits['max_angular_rad_s']
    return {
        'teleop_velocity_smoother': {
            'ros__parameters': {
                'max_velocity': [lin, 0.0, ang],
                'min_velocity': [-lin, 0.0, -ang],
            }
        }
    }


def f710_policy(overrides: dict, index: dict) -> dict:
    """F710 cached trims, clamped to the ceiling. Caps never open a bypass."""
    limits = effective_limits(overrides, index)
    lin, ang = limits['max_linear_m_s'], limits['max_angular_rad_s']
    return {
        'max_linear': lin,
        'max_angular': ang,
        'max_linear_cap': lin,
        'max_angular_cap': ang,
        'min_linear': min(F710_LINEAR_MIN, lin),
        'min_angular': min(F710_ANGULAR_MIN, ang),
    }


def navigation_overlay(overrides: dict, index: dict) -> dict:
    """
    Navigation scope: per-node effective values plus the F710 trim policy.

    Every downstream producer is emitted at min(cap, target) so a partially
    configured producer cannot exceed the active policy.
    """
    overlay: dict = {}
    for node in CONSUMER_SCOPES['navigation']['nodes']:
        if node == F710_NODE:
            continue
        params = _deep_merge(_node_params(index, overrides, node),
                             _producer_values(index, overrides, node))
        if params:
            overlay[node] = {'ros__parameters': params}
    smoother = overlay.setdefault('velocity_smoother', {}).setdefault('ros__parameters', {})
    maximum = smoother.get('max_velocity')
    if maximum:
        smoother['min_velocity'] = [-maximum[0], 0.0, -maximum[2]]
    controller = overlay.setdefault('controller_server', {}).setdefault(
        'ros__parameters', {}).setdefault('FollowPath', {})
    cruise = controller.get(
        'desired_linear_vel',
        min(0.22, effective_limits(overrides, index)['max_linear_m_s']))
    controller['min_approach_linear_velocity'] = min(
        controller.get('min_approach_linear_velocity', 0.08), cruise)
    controller['regulated_linear_scaling_min_speed'] = min(0.10, cruise)
    behavior = overlay.setdefault('behavior_server', {}).setdefault('ros__parameters', {})
    behavior['min_rotational_vel'] = min(0.4, behavior.get('max_rotational_vel', 0.7))
    overlay[F710_NODE] = {'ros__parameters': f710_policy(overrides, index)}
    # These are fixed maintenance invariants, never editable browser fields.
    # Persist the same fresh-seed behavior used during a runtime reload so a
    # subsequent process/container restart cannot revive an old AMCL pose.
    amcl = overlay.setdefault('amcl', {}).setdefault('ros__parameters', {})
    amcl['set_initial_pose'] = False
    amcl['always_reset_initial_pose'] = True
    return overlay


def build_consumer_overlays(overrides: dict, index: dict) -> dict:
    return {
        'bringup': bringup_overlay(overrides, index),
        'navigation': navigation_overlay(overrides, index),
    }


def generated_artifacts(overrides: dict, index: dict) -> dict:
    """
    Deterministic ``filename -> text`` for every consumer scope.

    Filenames are fixed by the scope name; no caller can influence a path.
    """
    overlays = build_consumer_overlays(overrides, index)
    artifacts = {}
    for scope, overlay in overlays.items():
        artifacts[scope + '.yaml'] = strict_yaml_dump(
            _deep_merge(packaged_defaults(scope), overlay))
    return artifacts


def packaged_defaults(scope):
    package, relative = (('patrolbot-launch', 'param/defaults/smoother.yaml')
                         if scope == 'bringup'
                         else ('patrolbot_navigation', 'config/nav2_params.yaml'))
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(Path(get_package_share_directory(package)) / relative)
    except (ImportError, LookupError):
        pass
    for parent in Path(__file__).resolve().parents:
        candidates.extend((parent / 'ros2_ws/src' / package / relative,
                           parent / 'src' / package / relative))
    path = next((candidate for candidate in candidates if candidate.is_file()), None)
    if path is None:
        raise SpecError('reviewed packaged defaults are unavailable for ' + scope)
    document = strict_yaml_load(path.read_text())
    if not isinstance(document, dict):
        raise SpecError('invalid packaged defaults')
    return document


# Fixed source-file -> scope lookup. A logical parameter may only be seeded
# from the reviewed consumer document its own ``source.file`` names; no caller
# can supply a path.
_SCOPE_BY_SOURCE_FILE = {
    scope['source_file']: name for name, scope in CONSUMER_SCOPES.items()
}


def _source_value(document, dotted, what, pid):
    """Follow a dotted/literal path, failing closed on any missing segment."""
    if not isinstance(dotted, str) or not dotted:
        raise SpecError(pid + ': ' + what + ' is not a dotted string')
    cursor = document
    for part in dotted.split('.'):
        if not isinstance(cursor, dict) or part not in cursor:
            raise SpecError(pid + ': missing ' + what + ' ' + repr(dotted))
        cursor = cursor[part]
    return cursor


def yaml_starting_values(index: dict, *, documents=None) -> dict:
    """
    Read the current packaged YAML starting value for every editable parameter.

    The reviewed registry records a parameter's *bounds*; the packaged consumer
    YAML is the deployment's actual starting configuration. This is the single
    read-only bridge between them: for every full, editable, non-protected,
    applicable logical id it follows the entry's allowlisted ``source`` (file to
    scope, dotted node, ``ros__parameters``, dotted/literal key, optional
    non-negative index) into the real document and returns the value found
    there.

    It never substitutes the registry ``default`` when a value is absent or
    invalid: that would silently trust a stale catalog over the shipped YAML. A
    missing key, an unallowlisted source file, a malformed index or a value that
    fails :func:`validate_change` all raise :class:`SpecError`. Callers may
    inject ``documents`` (``scope -> mapping``) for offline tests; otherwise the
    packaged defaults are read. Nothing is written and no global state changes.
    """
    if not isinstance(index, dict):
        raise SpecError('index must be a parameter mapping')
    if documents is None:
        documents = {name: packaged_defaults(name) for name in CONSUMER_SCOPES}
    if not isinstance(documents, dict):
        raise SpecError('documents must be a scope mapping')
    values = {}
    for pid, entry in index.items():
        if (entry.get('accepts_edits') is not True
                or entry.get('protected') is True
                or entry.get('not_applicable') is True):
            continue
        source = entry.get('source')
        if not isinstance(source, dict):
            raise SpecError(pid + ': source is not a mapping')
        scope = _SCOPE_BY_SOURCE_FILE.get(source.get('file'))
        if scope is None:
            raise SpecError(pid + ': source file is not an allowlisted consumer scope')
        document = documents.get(scope)
        if not isinstance(document, dict):
            raise SpecError(pid + ': missing packaged document for scope ' + scope)
        node = _source_value(document, source.get('node'), 'source node', pid)
        if not isinstance(node, dict):
            raise SpecError(pid + ': source node is not a mapping')
        params = node.get('ros__parameters')
        if not isinstance(params, dict):
            raise SpecError(pid + ': source node has no ros__parameters mapping')
        value = _source_value(params, source.get('key'), 'source key', pid)
        position = source.get('index')
        if position is not None:
            if not _is_strict_int(position) or position < 0:
                raise SpecError(pid + ': source index must be a non-negative integer')
            if not isinstance(value, (list, tuple)) or position >= len(value):
                raise SpecError(pid + ': source index is out of range')
            value = value[position]
        problems = validate_change(index, pid, value)
        if problems:
            raise SpecError(pid + ': ' + '; '.join(problems))
        values[pid] = value
    return values


def defaults_digest():
    return artifacts_digest({scope: strict_yaml_dump(packaged_defaults(scope))
                             for scope in CONSUMER_SCOPES})


@lru_cache(maxsize=1)
def compiler_digest():
    """Pin the actual artifact compiler; an image change requires explicit compatibility."""
    return 'sha256:' + hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def artifacts_digest(artifacts: dict) -> str:
    digest = hashlib.sha256()
    for name in sorted(artifacts):
        digest.update(name.encode() + b'\0')
        digest.update(artifacts[name].encode() + b'\0')
    return 'sha256:' + digest.hexdigest()


__all__ = [
    'SpecError',
    'StrictYAMLError',
    'CAPABILITY',
    'CONTRACT',
    'CONTRACT_VERSION',
    'CONSUMER_SCOPES',
    'LINEAR_CEILING',
    'ANGULAR_CEILING',
    'load_registry',
    'registry_digest',
    'index_registry',
    'reviewed_defaults',
    'dependency_edges',
    'validate_change',
    'validate_overrides',
    'merged_overrides',
    'yaml_starting_values',
    'effective_limits',
    'speed_producer_targets',
    'bringup_overlay',
    'f710_policy',
    'navigation_overlay',
    'build_consumer_overlays',
    'generated_artifacts',
    'defaults_digest',
    'compiler_digest',
    'artifacts_digest',
]
