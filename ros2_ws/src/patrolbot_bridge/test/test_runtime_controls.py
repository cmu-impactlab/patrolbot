"""Portable contract and no-effect regressions; runnable with unittest or pytest."""
import copy
import asyncio
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from uuid import uuid4

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from patrolbot_bridge import runtime_contract as c
from patrolbot_bridge.runtime_controls import RuntimeControls

ROOT = Path(__file__).resolve().parents[4]


class RuntimeControlsTests(unittest.TestCase):
    def setUp(self):
        self.read = dict(contract=c.CONTRACT, version=1, operation='read', request_id=str(uuid4()))
        self.manual = dict(self.read, operation='set', state_generation=str(uuid4()),
                           target='manual', values={'linear_m_s': .1, 'angular_rad_s': .3})

    def test_fixtures(self):
        data = json.loads((ROOT / 'contracts/runtime-controls.fixtures.json').read_text())
        for item in data['valid']:
            with self.subTest(item=item):
                self.assertEqual(c.request(item), item)
        for item in data['invalid']:
            with self.subTest(item=item), self.assertRaises(ValueError):
                c.request(item)

    def test_nonfinite_overflow_and_bounds(self):
        for field, low, high in [('linear_m_s', .05, .3), ('angular_rad_s', .1, .7)]:
            for value in [low, high]:
                payload = copy.deepcopy(self.manual); payload['values'][field] = value
                self.assertEqual(c.request(payload), payload)
            for value in [low-1e-9, high+1e-9, 0, -1, True, '0.1', float('nan'),
                          float('inf'), -float('inf'), 10**10000]:
                payload = copy.deepcopy(self.manual); payload['values'][field] = value
                with self.subTest(field=field, kind=type(value).__name__), self.assertRaises(ValueError):
                    c.request(payload)

    def test_strict_json(self):
        for raw in ['{"a":1,"a":2}', '{"a":NaN}', '{"a":Infinity}', ' '*8193, '['*10000]:
            with self.assertRaises(ValueError): c.loads(raw)

    def test_presets_and_custom_do_not_drift(self):
        for _ in range(10):
            for name, speed in [('low', .11), ('medium', .165), ('high', .22)]:
                preset = c.navigation_speed({'profile': name}, .22, .22)
                custom = c.navigation_speed({'custom_m_s': speed}, .22, .22)
                self.assertAlmostEqual(preset['m_s'], custom['m_s'])
                self.assertAlmostEqual(preset['percentage'], c.PRESETS[name]*100)

    def test_stricter_navigation_bounds_and_unavailable_baseline(self):
        for baseline, upper, value in [(None, .22, {'profile':'low'}),
                (.22, .10, {'profile':'low'}), (.08, .08, {'profile':'low'}),
                (.22, .22, {'custom_m_s':0}), (.22,.22,{'profile':[]})]:
            with self.assertRaises(ValueError): c.navigation_speed(value, baseline, upper)
        self.assertEqual(c.navigation_speed({'profile':'low'}, .22, .12)['m_s'], .11)

    def test_reads_have_no_effect_and_never_confirm_parameters(self):
        reads = []
        service = RuntimeControls('fixture', lambda: reads.append(1) or {
            'source':'ros_parameters', 'age_s':.2, 'hardware_session_generation':3,
            'values':{'max_linear_m_s':.1, 'nav2.controller.desired_linear_vel':.22}})
        result = asyncio.run(service.run(json.dumps(self.read)))
        self.assertEqual(c.response(result), result)
        self.assertEqual(reads, [1])
        state = result['snapshot']
        self.assertIsNone(state['manual']['confirmed'])
        self.assertIsNone(state['navigation']['baseline_m_s'])
        self.assertEqual(state['observed']['values']['max_linear_m_s'], .1)
        self.assertEqual(state['observed']['values']['nav_cruise_m_s'], .22)
        self.assertEqual(asyncio.run(service.run(json.dumps(self.read)))['snapshot']['state_generation'], state['state_generation'])

    def test_unsigned_mutations_rejected_without_any_consumer_call(self):
        def forbidden(): raise AssertionError('unsigned mutation must never call consumer')
        service = RuntimeControls('fixture', forbidden)
        for target, values in [('manual', self.manual['values']), ('navigation', {'profile':'low'})]:
            for generation in [str(uuid4()), service.generation]:
                payload = dict(self.manual, target=target, values=values, state_generation=generation)
                for _ in range(2):
                    with self.assertRaises(ValueError):
                        asyncio.run(service.run(json.dumps(payload)))

    def test_restart_and_stale_evidence(self):
        a, b = RuntimeControls('fixture', lambda:None), RuntimeControls('fixture', lambda:None)
        self.assertNotEqual(a.generation,b.generation)
        for age in [-1, 3.001, float('nan')]:
            state=c.snapshot('fixture',a.generation, {'age_s':age,'values':{}})
            self.assertIsNone(state['observed'])

    def test_false_confirmation_or_boolean_version_rejected(self):
        valid = asyncio.run(RuntimeControls('fixture', lambda:None).run(json.dumps(self.read)))
        for mutate in [lambda s:s.update(version=True), lambda s:s.update(status='confirmed'),
                       lambda s:s['manual'].update(confirmed={'linear_m_s':.1}),
                       lambda s:s['navigation']['presets']['high'].update(fraction=True)]:
            result=copy.deepcopy(valid); mutate(result['snapshot'])
            with self.assertRaises(ValueError): c.response(result)

    def test_callback_invalid_is_bounded_and_no_read(self):
        def forbidden(): raise AssertionError('invalid request must not read')
        service=RuntimeControls('fixture',forbidden)
        result=asyncio.run(service.callback(NS(payload_json='{"operation":"save"}'),NS()))
        self.assertIsNone(json.loads(result.result_json)['snapshot'])


if __name__ == '__main__': unittest.main()
