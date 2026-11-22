"""Strict Runtime Controls v1 contract, mirrored across trusted boundaries."""
import json
import math
from uuid import UUID

CONTRACT = 'patrolbot-runtime-controls'
READ_CAPABILITY = 'runtime_controls_read_v1'
MANUAL_CAPABILITY = 'runtime_controls_manual_v1'
POLICY_CAPABILITY = 'runtime_controls_policy_v1'
MAX_BYTES = 8192
PRESETS = {'low': .50, 'medium': .75, 'high': 1.0}
MANUAL_REASON = ('Runtime manual updates unavailable: consumers lack a shared '
                 'idle reservation and generation-bound effective trim readback.')
NAV_REASON = ('Navigation updates unavailable: no exclusive speed-limit arbiter '
              'retains independent safety restrictions; Nav2 has no application acknowledgement.')


def uuid(value):
    try:
        return isinstance(value, str) and str(UUID(value)) == value
    except (ValueError, TypeError, AttributeError):
        return False


def number(value, lower, upper):
    # Check magnitude before isfinite so arbitrarily large JSON integers fail safely.
    return (type(value) in (int, float) and lower <= value <= upper
            and math.isfinite(value))


def loads(raw):
    if not isinstance(raw, str) or len(raw.encode('utf-8')) > MAX_BYTES:
        raise ValueError('runtime payload too large or not text')
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError('duplicate runtime field')
            result[key] = value
        return result
    def constant(value):
        raise ValueError('nonfinite runtime number')
    try:
        return json.loads(raw, object_pairs_hook=pairs, parse_constant=constant)
    except RecursionError as exc:
        raise ValueError('nested runtime payload') from exc


def request(value):
    common = {'contract', 'version', 'operation', 'request_id'}
    if not isinstance(value, dict) or value.get('operation') not in ('read', 'set', 'reconcile'):
        raise ValueError('invalid runtime operation')
    fields = common if value['operation'] == 'read' else common | {
        'state_generation', 'target', 'values'}
    if (set(value) != fields or value['contract'] != CONTRACT
            or type(value['version']) is not int or value['version'] != 1
            or not uuid(value['request_id'])):
        raise ValueError('invalid runtime request')
    if value['operation'] == 'reconcile':
        if (not uuid(value['state_generation']) or value['target'] != 'manual' or value['values'] != {}):
            raise ValueError('Invalid bounded ownership reconciliation')
    if value['operation'] == 'set':
        if not uuid(value['state_generation']):
            raise ValueError('invalid runtime generation')
        values = value['values']
        if not isinstance(values, dict):
            raise ValueError('invalid runtime values')
        if value['target'] == 'manual':
            if (set(values) != {'linear_m_s', 'angular_rad_s'}
                    or not number(values['linear_m_s'], .05, .30)
                    or not number(values['angular_rad_s'], .1, .70)):
                raise ValueError('invalid manual ceilings')
        elif value['target'] == 'navigation':
            if set(values) == {'profile'}:
                if not isinstance(values['profile'], str) or values['profile'] not in PRESETS:
                    raise ValueError('invalid navigation profile')
            elif set(values) != {'custom_m_s'} or not number(values['custom_m_s'], .08, .30):
                raise ValueError('invalid navigation ceiling')
        else:
            raise ValueError('invalid runtime target')
    return value


def navigation_speed(values, baseline, upper, lower=.08):
    if (not number(baseline, .08, .30) or not number(upper, .08, baseline)
            or not number(lower, .08, upper)):
        raise ValueError('unvalidated commissioned navigation bounds')
    if not isinstance(values, dict):
        raise ValueError('invalid navigation values')
    if set(values) == {'profile'} and isinstance(values['profile'], str) and values['profile'] in PRESETS:
        speed = baseline * PRESETS[values['profile']]
    elif set(values) == {'custom_m_s'}:
        speed = values['custom_m_s']
    else:
        raise ValueError('invalid navigation values')
    if not number(speed, lower, upper):
        raise ValueError('navigation speed outside commissioned bounds')
    return {'m_s': speed, 'percentage': speed / baseline * 100.0}


def snapshot(robot_id, generation, observation=None):
    # Preserve source age/generation. A cached parameter is not effective trim readback.
    observed = None
    if (isinstance(observation, dict) and observation.get('source') == 'ros_parameters'
            and number(observation.get('age_s'), 0, 3)
            and type(observation.get('hardware_session_generation')) is int
            and 0 <= observation['hardware_session_generation'] < 2**53
            and isinstance(observation.get('values'), dict)):
        values = observation.get('values', {})
        names = {'nav2.controller.desired_linear_vel': 'nav_cruise_m_s',
                 'max_linear_m_s': 'max_linear_m_s',
                 'max_angular_rad_s': 'max_angular_rad_s'}
        observed = {label: values[key] for key, label in names.items()
                    if number(values.get(key), 0, 10)}
        observed = {'source': 'ros_parameters', 'age_s': observation['age_s'],
                    'hardware_session_generation': observation.get('hardware_session_generation'),
                    'values': observed}
    return {'contract': CONTRACT, 'version': 1, 'robot_id': robot_id,
            'state_generation': generation, 'consumer_generation': None,
            'pending': None, 'status': 'unavailable', 'observed': observed,
            'manual': {'requested': None, 'confirmed': None, 'reason': MANUAL_REASON,
                       'linear_m_s': {'min': .05, 'max': .30, 'step': .05},
                       'angular_rad_s': {'min': .1, 'max': .70, 'step': .1}},
            'navigation': {'requested': None, 'confirmed': None, 'reason': NAV_REASON,
                           'baseline_m_s': None, 'min_m_s': .08, 'max_m_s': None,
                           'presets': {name: {'fraction': fraction, 'm_s': None}
                                       for name, fraction in PRESETS.items()}}}


def response(value):
    if (not isinstance(value, dict) or set(value) != {'request_id', 'outcome', 'detail', 'snapshot'}
            or not uuid(value['request_id']) or value['outcome'] not in ('ok', 'unavailable', 'confirmed', 'rejected', 'unknown', 'partial')
            or not isinstance(value['detail'], str) or len(value['detail']) > 512):
        raise ValueError('invalid runtime response')
    state = value['snapshot']
    if state is not None:
        if not isinstance(state, dict) or not uuid(state.get('state_generation')):
            raise ValueError('invalid runtime snapshot')
        expected = snapshot(state.get('robot_id'), state['state_generation'])
        observed = state.get('observed')
        if observed is not None:
            if (not isinstance(observed, dict) or set(observed) != {
                    'source', 'age_s', 'hardware_session_generation', 'values'}
                    or observed['source'] != 'ros_parameters'
                    or not number(observed['age_s'], 0, 3)
                    or type(observed['hardware_session_generation']) is not int
                    or not 0 <= observed['hardware_session_generation'] < 2**53
                    or not isinstance(observed['values'], dict)
                    or set(observed['values']) - {'f710_linear_m_s', 'f710_angular_rad_s',
                        'nav_cruise_m_s', 'max_linear_m_s', 'max_angular_rad_s'}
                    or not all(number(v, 0, 10) for v in observed['values'].values())):
                raise ValueError('invalid runtime observation')
        expected['observed'] = observed
        if state.get('status') not in ('unavailable', 'default', 'pending', 'confirmed', 'rejected', 'partial', 'unknown'):
            raise ValueError('invalid runtime status')
        expected['status'] = state['status']
        pending = state.get('pending')
        if pending is not None and not uuid(pending):
            raise ValueError('invalid pending reservation')
        expected['pending'] = pending
        manual = state.get('manual')
        if not isinstance(manual, dict) or set(manual) != set(expected['manual']):
            raise ValueError('invalid manual state')
        if not isinstance(manual['reason'], str) or len(manual['reason']) > 512:
            raise ValueError('invalid manual reason')
        for name, lower, upper, step in (('linear_m_s', .05, .3, .05), ('angular_rad_s', .1, .7, .1)):
            bounds = manual[name]
            if (not isinstance(bounds, dict) or set(bounds) != {'min', 'max', 'step'}
                    or bounds['min'] != lower or bounds['step'] != step
                    or not number(bounds['max'], 0, upper)):
                raise ValueError('invalid commissioned manual bounds')
        requested = manual['requested']
        if requested is not None:
            request(dict(contract=CONTRACT, version=1, operation='set', request_id=state['state_generation'],
                         state_generation=state['state_generation'], target='manual', values=requested))
        consumers = manual['confirmed']
        if state['status'] != 'unavailable' and consumers is None:
            raise ValueError('missing effective consumer evidence')
        if consumers is not None:
            if not isinstance(consumers, dict) or set(consumers) != {'web', 'f710'} or not uuid(state.get('consumer_generation')):
                raise ValueError('invalid manual consumer evidence')
            fields = {'available', 'generation', 'hardware_generation', 'reservation', 'linear_m_s', 'angular_rad_s',
                      'effective_linear_m_s', 'effective_angular_rad_s', 'upper_linear_m_s', 'upper_angular_rad_s'}
            for evidence in consumers.values():
                if (not isinstance(evidence, dict) or set(evidence) != fields
                        or type(evidence['available']) is not bool or not uuid(evidence['generation'])
                        or type(evidence['hardware_generation']) is not int
                        or not 0 <= evidence['hardware_generation'] < 2**53
                        or not isinstance(evidence['reservation'], str)
                        or (evidence['reservation'] and not uuid(evidence['reservation']))):
                    raise ValueError('invalid consumer identity')
                for key in fields - {'available', 'generation', 'hardware_generation', 'reservation'}:
                    if not number(evidence[key], 0, .3 if 'linear' in key else .7):
                        raise ValueError('invalid effective consumer speed')
                if (evidence['effective_linear_m_s'] > evidence['linear_m_s']
                        or evidence['effective_angular_rad_s'] > evidence['angular_rad_s']):
                    raise ValueError('effective trim exceeds ceiling')
            if len({e['hardware_generation'] for e in consumers.values()}) != 1:
                raise ValueError('mixed consumer hardware generations')
            expected['consumer_generation'] = state['consumer_generation']
        expected['manual'] = manual
        if (json.dumps(state, sort_keys=True, allow_nan=False) != json.dumps(expected, sort_keys=True, allow_nan=False)
                or not isinstance(state['robot_id'], str) or not 1 <= len(state['robot_id']) <= 64):
            raise ValueError('unsupported runtime evidence or capability')
    if value['outcome'] in ('ok', 'confirmed') and state is None:
        raise ValueError('runtime read lacks state')
    return value
