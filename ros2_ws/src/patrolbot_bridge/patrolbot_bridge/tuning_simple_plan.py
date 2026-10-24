"""Two execution choices: verified hot batch, or one complete Nav2 redeploy.

No per-node lifecycle schedule, silent scope escalation, source/image rebuild,
SBC operation, or coordinator/container handoff is part of this policy.
"""

from .tuning_backend import parameter_plan
from .tuning_mutability import NAVIGATION_NODES, NAVIGATION_SERVICE, classify
from .tuning_plan import canonical_digest


def build_plan(*, candidate, active, index, registry_digest, session_id, generation,
               desired_revision, operation='apply', states=None, hot_support=(),
               nav2_redeploy_supported=False, policy_support=False, implementation='unverified'):
    if operation not in ('apply', 'restore'):
        raise ValueError('unsupported activation operation')
    states = dict(states or {})
    hot_support = frozenset(hot_support)
    changes = []
    reasons = []
    operations = []
    classifications = []
    if generation is None:
        reasons.append({'code': 'hardware_identity_unknown', 'detail': 'Hardware/session identity is unavailable.'})
    for pid, entry in sorted(index.items()):
        if not entry.get('accepts_edits') or entry.get('protected') or entry.get('not_applicable'):
            continue
        before = active['overrides'].get(pid, entry['default'])
        after = candidate['overrides'].get(pid, entry['default'])
        if before != after:
            changes.append({'id': pid, 'before': before, 'after': after})
    try:
        old, new = parameter_plan(active, index), parameter_plan(candidate, index)
    except (ValueError, KeyError, TypeError) as exc:
        reasons.append({'code': 'runtime_domain_invalid', 'detail': str(exc)[:200]})
    else:
        for node, values in sorted(new.items()):
            changed = {key: value for key, value in sorted(values.items()) if value != old[node].get(key)}
            if not changed:
                continue
            kind, explanation = classify(node, changed, states.get(node))
            classifications.append({'node': node, 'mode': kind, 'reason': explanation})
            operations.append({'node': node, 'kind': 'parameters', 'parameters': changed})
            if kind == 'unsupported':
                reasons.append({'code': 'consumer_state_unsupported', 'detail': explanation})
    restart = any(item['mode'] == 'nav2_redeploy' for item in classifications)
    # Constructor/PF reset fields need reconstruction even if runtime state is
    # currently unobservable. Keep the impact truthful while the unsupported
    # state reason above still forbids execution; never label such edits hot.
    restart = restart or any(classify(step['node'], step['parameters'], 3)[0] == 'nav2_redeploy'
                             for step in operations)
    changed_nodes = [step['node'] for step in operations]
    # If one Nav2 field requires reconstruction, all changes inside that fixed
    # container are loaded together at its reviewed boot. Never first hot-write
    # a subset into an old stack that is about to be replaced.
    for step in operations:
        if restart and step['node'] in NAVIGATION_NODES:
            step['kind'] = 'nav2_restart'
        elif step['node'] not in hot_support:
            reasons.append({'code': 'hot_update_unverified',
                            'detail': step['node'] + ': actual hot cache consumption is not verified.'})
    if restart and nav2_redeploy_supported is not True:
        reasons.append({'code': 'nav2_redeploy_unavailable',
                        'detail': 'The fixed navigation-only redeploy and replacement verification are unavailable.'})
    caps_changed = any(item['id'] in ('max_linear_m_s', 'max_angular_rad_s') for item in changes)
    if caps_changed and policy_support is not True:
        reasons.append({'code': 'policy_handshake_unverified',
                        'detail': 'Changed core/F710/web speed policies cannot yet be verified.'})
    if implementation == 'unverified':
        reasons.append({'code': 'implementation_unverified', 'detail': 'The exact native implementation is not verified.'})
    nodes = sorted(set(changed_nodes) | (set(NAVIGATION_NODES) if restart else set()))
    plan = {
        'version': 1, 'strategy': 'hot_or_nav2_redeploy', 'operation': operation,
        'revision': candidate['meta']['revision'], 'desired_revision': desired_revision,
        'digest': candidate['meta']['generated_digest'], 'base_revision': active['meta']['revision'],
        'base_digest': active['meta']['generated_digest'], 'session_id': session_id,
        'hardware_generation': canonical_digest(generation),
        'compatibility': {'registry': registry_digest, 'compiler': candidate['meta']['compiler_digest'],
                          'defaults': candidate['meta']['defaults_digest'], 'implementation': implementation},
        'execution_kind': 'nav2_redeploy' if restart else 'scoped_parameters',
        'nodes': nodes, 'changed_nodes': changed_nodes,
        'services': [NAVIGATION_SERVICE] if restart else [], 'operations': operations,
        'classifications': classifications,
        'runtime_states': {node: states.get(node) for node in changed_nodes},
        'cache_consumers': ['core_clamp', 'web_sessions'] if caps_changed else [],
        'verification_scopes': ['bringup', 'navigation'], 'seed_required': restart,
        'input_policy': 'Block commands during maintenance; discard input, never queue/replay or auto-arm.',
        'state_policy': 'Navigation readiness is separate from parameter activation; ordinary restart uses committed ACTIVE.',
        'disruption': ('Restart the complete navigation container, including AMCL, Nav2, F710 and its watchdog. '
                       'Core bridge, bringup, dashboard and SBC are not restarted. A fresh operator seed is required.'
                       if restart else 'No container or lifecycle restart. Apply only verified hot values; '
                       'preserve localization. Inputs remain disarmed until explicitly started again.'),
        'image_rebuild': False, 'executable': not reasons, 'reasons': reasons, 'changes': changes,
    }
    plan['plan_hash'] = canonical_digest(plan)
    return plan
