"""Contract for prompt injected action responses through real production BTs."""
def validate(capture):
    assert capture['elapsed_s'] <= 120., 'original case budget exceeded'
    assert capture.get('navigator_exit_code') == 0, 'navigator teardown crashed or required termination'
    if capture['mode'] == 'planner-exhaust':
        assert capture['status'] == 6, 'planner exhaustion did not abort'
        assert capture['follow_calls'] == 0, 'failed planning reached FollowPath'
        assert capture['plan_calls'] == 2, 'planner exceeded its single contextual retry'
        assert capture['planner_contextual_clear_calls'] == 1, 'planner contextual clearing missing'
        expected = []
    elif capture['mode'] == 'exhaust':
        assert capture['status'] == 6, 'recovery exhaustion did not abort'
        assert capture['follow_calls'] == 20, 'contextual/outer retry count changed'
        expected = ['backup', 'clear', 'spin', 'wait'] * 2 + ['backup']
    else:
        assert capture['mode'] == 'fault-clears'
        assert capture['status'] == 4, 'cleared fault did not permit navigation success'
        assert capture['follow_calls'] == 9, 'fault cleared before full recovery cycle'
        expected = ['backup', 'clear', 'spin', 'wait']
    if capture['mode'] != 'planner-exhaust':
        assert capture['contextual_clear_calls'] == capture['follow_calls'] // 2, 'contextual recovery clearing missing'
    assert capture['recovery_actions'] == expected, 'production recovery sequence incomplete'
