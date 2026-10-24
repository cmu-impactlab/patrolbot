"""Candidate core clamp never authorizes driving or survives as startup intent."""

import pytest
import threading

from patrolbot_bridge.tuning_policy import PolicyCache

JOB = {'job_id': '11111111-1111-4111-8111-111111111111',
       'digest': 'sha256:' + 'a' * 64, 'plan_hash': 'sha256:' + 'b' * 64}
ACTIVE = {'max_linear_m_s': .3, 'max_angular_rad_s': .7}
CANDIDATE = {'max_linear_m_s': .2, 'max_angular_rad_s': .5}


def test_candidate_is_verified_but_commands_stay_zero_until_committed_active_binding():
    active = {'generation': '1:1', 'limits': ACTIVE}
    cache = PolicyCache(lambda: (active['generation'], active['limits']), required=True)
    cache.refresh()
    assert cache.stage(JOB, CANDIDATE, maintenance_owned=True)['limits'] == CANDIDATE
    assert cache.clamp(.3, .7)[:2] == (0., 0.)
    assert not cache.refresh()
    assert cache.staged_evidence(JOB)['drive_blocked']
    with pytest.raises(ValueError):
        cache.commit_staged(JOB, generation='1:2', committed_limits=CANDIDATE, maintenance_owned=True)
    assert cache.clamp(.3, .7)[:2] == (0., 0.)
    active.update(generation='1:2', limits=CANDIDATE)
    assert cache.commit_staged(JOB, generation='1:2', committed_limits=CANDIDATE,
                              maintenance_owned=True)['limits'] == CANDIDATE
    assert cache.clamp(.3, .7)[:2] == (.2, .5)


@pytest.mark.parametrize('limits,owned', [(CANDIDATE, False), ({'max_linear_m_s': .31, 'max_angular_rad_s': .7}, True),
                                      ({'max_linear_m_s': float('nan'), 'max_angular_rad_s': .7}, True)])
def test_policy_staging_never_sanitizes_unsafe_intent_into_a_false_confirmation(limits, owned):
    cache = PolicyCache(lambda: ('1:1', ACTIVE), required=True)
    with pytest.raises(ValueError):
        cache.stage(JOB, limits, maintenance_owned=owned)


def test_foreign_job_cannot_replace_or_commit_staged_policy():
    cache = PolicyCache(lambda: ('1:1', ACTIVE), required=True)
    cache.stage(JOB, CANDIDATE, maintenance_owned=True)
    foreign = dict(JOB, job_id='22222222-2222-4222-8222-222222222222')
    with pytest.raises(ValueError):
        cache.stage(foreign, CANDIDATE, maintenance_owned=True)
    with pytest.raises(ValueError):
        cache.commit_staged(foreign, generation='1:2', committed_limits=CANDIDATE, maintenance_owned=True)
    assert cache.limits() == {'max_linear_m_s': 0., 'max_angular_rad_s': 0.}


def test_new_process_cache_reads_only_committed_active_not_abandoned_candidate():
    cache = PolicyCache(lambda: ('1:1', ACTIVE), required=True)
    cache.stage(JOB, CANDIDATE, maintenance_owned=True)
    restarted = PolicyCache(lambda: ('1:1', ACTIVE), required=True)
    restarted.refresh()
    assert restarted.limits() == ACTIVE


def test_background_active_probe_cannot_deadlock_or_overwrite_a_new_candidate_commit():
    probing, finish = threading.Event(), threading.Event()
    active = {'generation': '1:1', 'limits': ACTIVE}
    def probe():
        if threading.current_thread().name == 'background-policy-read':
            snapshot = (active['generation'], active['limits'])
            probing.set()
            assert finish.wait(2.)
            return snapshot
        return active['generation'], active['limits']
    cache = PolicyCache(probe, required=True)
    thread = threading.Thread(target=cache.refresh, name='background-policy-read')
    thread.start()
    assert probing.wait(2.)
    try:
        # Would deadlock here if the worker held cache mutex while waiting for
        # its store reader; the explicit commit owns writer -> cache ordering.
        cache.stage(JOB, CANDIDATE, maintenance_owned=True)
        active.update(generation='1:2', limits=CANDIDATE)
        cache.commit_staged(JOB, generation='1:2', committed_limits=CANDIDATE, maintenance_owned=True)
    finally:
        finish.set()
        thread.join(2.)
    assert not thread.is_alive()
    assert cache.generation == '1:2' and cache.limits() == CANDIDATE
