"""Acceptance evidence must include independent motion and stop observations."""

import ast
import math
from pathlib import Path
import tempfile
import unittest


def validator(name='validate_capture'):
    source = Path(__file__).with_name('integration_undock_turn.py')
    function = next(node for node in ast.parse(source.read_text()).body
                    if isinstance(node, ast.FunctionDef) and node.name == name)
    namespace = {'math': math, 'Path': Path}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), namespace)
    return namespace[name]


class UndockTurnCaptureTests(unittest.TestCase):
    def evidence(self, **overrides):
        capture = dict(scenario='reach', no_amcl_or_seed=True,
            native_libraries={str(index): ['/opt/patrolbot_nav2/install/lib/native.so']
                              for index in range(4)},
            normal_goals_held=True, hardware_clear_handoff=True,
            token_correlated=True, replay_rejected=True, stopped=True,
            final_drive=[0., 0.], drive_allowed_at_end=False,
            turn_drives=[[1., 0., .5]], status=4,
            result={'success': True, 'code': 0}, plant_yaw_delta=math.pi,
            plant_translation_m=0., active_scoped_state_seen=True,
            all_navigation_inactive=True, revocation_reason='')
        return dict(capture, **overrides)

    def test_action_success_alone_cannot_accept_a_turn(self):
        check = validator()
        check(self.evidence())
        for changes in ({'turn_drives': []}, {'plant_yaw_delta': 0.},
                        {'plant_translation_m': .02}, {'stopped': False},
                        {'status': 6}, {'token_correlated': False},
                        {'active_scoped_state_seen': False}, {'normal_goals_held': False},
                        {'replay_rejected': False}, {'no_amcl_or_seed': False},
                        {'revocation_reason': 'turn direction or distance exceeded'}):
            with self.subTest(changes=changes), self.assertRaises(AssertionError):
                check(self.evidence(**changes))

    def test_package_prefix_alone_cannot_prove_loaded_native_libraries(self):
        check = validator()
        for paths in ({}, {'lib.so': []},
                      {'lib.so': ['/opt/ros/jazzy/lib/lib.so']},
                      {'lib.so': ['/opt/patrolbot_nav2/install/lib/lib.so',
                                  '/opt/ros/jazzy/lib/lib.so']}):
            with self.subTest(paths=paths), self.assertRaises(AssertionError):
                check(self.evidence(native_libraries=paths))
        with tempfile.TemporaryDirectory() as temporary:
            proc = Path(temporary)
            (proc / '1').mkdir()
            (proc / '1/maps').write_text(
                '0 0 0 0 0 /opt/patrolbot_nav2/install/lib/libbehavior_server_core.so\n'
                '0 0 0 0 0 /opt/ros/jazzy/lib/libbehavior_server_core.so\n')
            libraries = validator('loaded_nav2_libraries')(proc)
            self.assertEqual(len(libraries['libbehavior_server_core.so']), 2)
            self.assertEqual(libraries['libcontroller_server_core.so'], [])

    def test_fault_requires_real_zero_and_lease_revocation(self):
        check = validator()
        capture = self.evidence(scenario='cancel', status=5,
            result={'success': False, 'code': 4}, zero_after_fault_s=.05,
            fault_requested_while_live=True, fault_recent_nonzero=True,
            revocation_reason='operator canceled undock turn')
        check(capture)
        for changes in ({'final_drive': [0., .2]}, {'drive_allowed_at_end': True},
                        {'turn_drives': [[1., .1, .2]]}, {'zero_after_fault_s': .61},
                        {'all_navigation_inactive': False}, {'status': 6},
                        {'result': {'success': False, 'code': 3}},
                        {'revocation_reason': 'hardware or odometry is stale'},
                        {'fault_requested_while_live': False}, {'fault_recent_nonzero': False}):
            with self.subTest(changes=changes), self.assertRaises(AssertionError):
                check(dict(capture, **changes))

    def test_late_mutation_is_not_accepted_as_a_restored_turn(self):
        check = validator()
        capture = self.evidence(scenario='lifecycle-late', status=6,
            result={'success': False, 'code': 2}, turn_drives=[],
            unknown_mutation_held=True, late_reply_received=True)
        check(capture)
        for changes in ({'unknown_mutation_held': False}, {'late_reply_received': False},
                        {'turn_drives': [[1., 0., .2]]}, {'status': 4}):
            with self.subTest(changes=changes), self.assertRaises(AssertionError):
                check(dict(capture, **changes))

    def test_restored_source_cannot_revive_an_expired_turn(self):
        check = validator()
        capture = self.evidence(scenario='source-expiry', status=6,
            result={'success': False, 'code': 3}, zero_after_fault_s=.3,
            fault_requested_while_live=True, fault_recent_nonzero=True,
            source_restored_after_revocation=True, revocation_reason='undock velocity source expired',
            no_nonzero_after_expiry=True, expiry_result_s=.5)
        check(capture)
        for changes in ({'source_restored_after_revocation': False},
                        {'no_nonzero_after_expiry': False}, {'expiry_result_s': 20.}):
            with self.subTest(changes=changes), self.assertRaises(AssertionError):
                check(dict(capture, **changes))

    def test_foreign_publisher_and_reconnect_need_the_injected_cause(self):
        check = validator()
        common = dict(status=6, result={'success': False, 'code': 3},
                      zero_after_fault_s=.05, fault_requested_while_live=True,
                      fault_recent_nonzero=True)
        foreign = self.evidence(scenario='foreign-publisher', **common,
            foreign_endpoint_count=2, revocation_reason='undock command publisher changed')
        reconnect = self.evidence(scenario='reconnect', **common,
            session_changed=True, new_session_drives=[[1., 0., 0.]],
            revocation_reason='hardware session disconnected')
        check(foreign)
        check(reconnect)
        for capture, changes in ((foreign, {'foreign_endpoint_count': 1}),
                (foreign, {'revocation_reason': 'odometry is stale'}),
                (reconnect, {'session_changed': False}),
                (reconnect, {'revocation_reason': 'odometry is stale'}),
                (reconnect, {'new_session_drives': [[1., 0., .1]]})):
            with self.subTest(changes=changes), self.assertRaises(AssertionError):
                check(dict(capture, **changes))


if __name__ == '__main__':
    unittest.main()
