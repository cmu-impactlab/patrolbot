"""Read-only, deterministic activation previews; no ROS or store writes.

Execution support is explicit evidence supplied by a backend, not inferred from
SetParameters service existence or registry restart scopes. The shipped native
backend supplies no verified operation support yet and cannot execute this plan.
"""

import hashlib
import json

from . import tuning_spec
from .tuning_backend import parameter_plan


PLAN_VERSION = 1
ROBOT_SERVICES = ('bringup', 'bridge', 'navigation')
KINDS = frozenset(('parameters', 'lifecycle'))


def canonical_digest(value):
    return 'sha256:' + hashlib.sha256(json.dumps(
        value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def build_plan(*, candidate, active, index, registry_digest, session_id, generation, desired_revision,
               operation='apply', support=None, implementation='unverified', policy_support=False):
    """Compile exact changed-consumer closure and its review hash.

    ``support`` is a fixed node -> proven operation mapping, never browser input.
    Unknown support remains non-executable. A Docker fallback must be a separately
    implemented/proven backend; this planner never silently substitutes one.
    """
    if operation not in ('apply', 'restore'):
        raise ValueError('unallowlisted plan operation')
    support = dict(support or {})
    if any(node not in tuning_spec.CONSUMER_SCOPES['navigation']['nodes']
           + tuning_spec.CONSUMER_SCOPES['bringup']['nodes'] or kind not in KINDS
           for node, kind in support.items()):
        raise ValueError('unallowlisted native support')
    revision = candidate['meta']['revision']
    changes = []
    for pid, entry in sorted(index.items()):
        if (entry.get('accepts_edits') is not True or entry.get('protected')
                or entry.get('not_applicable')):
            continue
        before = active['overrides'].get(pid, entry['default'])
        after = candidate['overrides'].get(pid, entry['default'])
        if before != after:
            changes.append({'id': pid, 'before': before, 'after': after})
    reasons = []
    if generation is None:
        reasons.append({'code': 'hardware_identity_unknown',
                        'detail': 'Current hardware generation is unavailable; preview is not executable.'})
    operations = []
    try:
        previous = parameter_plan(active, index)
        proposed = parameter_plan(candidate, index)
    except (ValueError, KeyError, TypeError) as exc:
        reasons.append({'code': 'runtime_domain_invalid', 'detail': str(exc)[:200]})
    else:
        for node in sorted(proposed):
            values = {key: value for key, value in sorted(proposed[node].items())
                      if value != previous[node].get(key)}
            if not values:
                continue
            # Constructor-only caches cannot be activated by a generic native
            # parameter service, even if another field on that node is dynamic.
            constructor_only = (node == 'behavior_server'
                                or (node == 'controller_server'
                                    and 'FollowPath.approach_velocity_scaling_dist' in values)
                                or node == 'amcl')
            kind = support.get(node)
            if constructor_only and kind == 'parameters':
                kind = None
            if kind is None:
                reasons.append({'code': 'implementation_unverified',
                                'detail': node + ': scoped cache/lifecycle activation is not verified.'})
            operations.append({'node': node, 'kind': kind or (
                'parameters' if node == 'patrolbot_joy_teleop' else 'lifecycle'),
                               'parameters': values})
    caps_changed = any(change['id'] in ('max_linear_m_s', 'max_angular_rad_s')
                       for change in changes)
    # Core/web policies need exact-job staging and committed-binding readback;
    # node parameter availability cannot stand in for that protocol.
    if caps_changed and policy_support is not True:
        reasons.append({'code': 'policy_handshake_unverified',
                        'detail': 'Core/F710/web candidate cache staging and committed policy acknowledgment are not commissioned.'})
    if implementation == 'unverified':
        reasons.append({'code': 'implementation_unverified',
                        'detail': 'The exact native image and scoped ownership/verification protocol are not commissioned.'})
    nodes = [step['node'] for step in operations]
    seed_required = 'amcl' in nodes
    plan = {
        'version': PLAN_VERSION, 'operation': operation, 'revision': revision,
        'desired_revision': desired_revision,
        'digest': candidate['meta']['generated_digest'],
        'base_revision': active['meta']['revision'],
        'base_digest': active['meta']['generated_digest'],
        'session_id': session_id, 'hardware_generation': canonical_digest(generation),
        'compatibility': {'registry': registry_digest,
                          'compiler': candidate['meta']['compiler_digest'],
                          'defaults': candidate['meta']['defaults_digest'],
                          'implementation': implementation},
        'execution_kind': ('scoped_lifecycle' if any(step['kind'] == 'lifecycle'
                                                   for step in operations) else 'scoped_parameters'),
        'nodes': nodes, 'services': [], 'operations': operations,
        'cache_consumers': ['core_clamp', 'web_sessions'] if caps_changed else [],
        'verification_scopes': ['bringup', 'navigation'],
        'state_policy': 'Preserve prior allowed lifecycle state; navigation readiness is separate.',
        'seed_required': seed_required,
        'disruption': ('Only listed consumers may change; input remains disarmed. '
                       + ('AMCL reset requires a new operator seed.' if seed_required
                          else 'AMCL/map state must not be reset.')),
        'executable': not reasons, 'reasons': reasons, 'changes': changes,
    }
    plan['plan_hash'] = canonical_digest(plan)
    return plan
