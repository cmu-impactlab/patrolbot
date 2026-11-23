"""Portable tests for the offline apply contract; no ROS or hardware."""

from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
import sys
from pathlib import Path
from threading import Event, Lock

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))

from autonomous_speed_apply import (  # noqa: E402
    ApplyResult,
    ApplyStatus,
    AutonomousSpeedApply,
    CommissionedCaps,
    Fence,
    UsedCaps,
    WriteOutcome,
    WriteReceipt,
)


class FakePathFollowing:
    """In-process gate shared by goal admission and every cap operation."""

    def __init__(self, fault=None, before_write=None, bounds=(0.26, 0.70)):
        self.fault = fault
        self.before_write = before_write
        self.gate = Lock()
        self.goal = False
        self.fence = None
        self.generation = 0
        self.bounds = bounds
        self.configured = self.used = (0.22, 0.70)
        self.calls = []

    def admit_goal(self):
        with self.gate:
            if self.goal or self.fence is not None:
                return False
            self.goal = True
            return True

    def finish_goal(self):
        with self.gate:
            self.goal = False

    def try_own_idle(self, owner):
        with self.gate:
            self.calls.append('own')
            if self.goal or self.fence is not None:
                return None
            if self.fault != 'reused_generation':
                self.generation += 1
            self.fence = Fence(owner, self.generation)
            if self.fault == 'lost_admission':
                raise TimeoutError
            if self.fault == 'wrong_admission_owner':
                return replace(self.fence, owner=object())
            return self.fence

    def _check(self, fence):
        if (self.fence is None or fence.owner is not self.fence.owner
                or fence.generation != self.fence.generation):
            raise RuntimeError('not the current fence owner')

    def read_commissioned_caps(self, fence):
        with self.gate:
            self.calls.append('bounds')
            self._check(fence)
            if self.fault == 'no_bounds':
                return None
            if self.fault == 'stale_bounds':
                fence = replace(fence, generation=fence.generation - 1)
            return CommissionedCaps(fence, self.bounds)

    def write_caps(self, fence, caps):
        if self.before_write is not None:
            self.before_write()
        with self.gate:
            self.calls.append('write')
            self._check(fence)
            if self.fault in ('rejected', 'wrong_rejection_owner'):
                reply = fence
                if self.fault == 'wrong_rejection_owner':
                    reply = replace(fence, owner=object())
                return WriteReceipt(reply, WriteOutcome.REJECTED_UNCHANGED)
            if self.fault == 'lost_before_write':
                raise TimeoutError
            self.configured = caps
            if self.fault not in ('ignored_linear', 'ignored_angular'):
                self.used = caps
            elif self.fault == 'ignored_linear':
                self.used = (self.used[0], caps[1])
            else:
                self.used = (caps[0], self.used[1])
            if self.fault == 'lost_after_write':
                raise TimeoutError
            if self.fault == 'ambiguous_write':
                return None
            if self.fault == 'wrong_write_generation':
                fence = replace(fence, generation=fence.generation + 1)
            return WriteReceipt(fence, WriteOutcome.APPLIED)

    def read_used_caps(self, fence):
        with self.gate:
            self.calls.append('read')
            self._check(fence)
            if self.fault == 'failed_read':
                raise OSError
            if self.fault == 'missing_read':
                return None
            if self.fault == 'wrong_read_generation':
                fence = replace(fence, generation=fence.generation - 1)
            if self.fault == 'wrong_read_owner':
                fence = replace(fence, owner=object())
            return UsedCaps(fence, self.used[:1] if self.fault == 'missing_axis' else self.used)

    def release(self, fence):
        with self.gate:
            self.calls.append('release')
            self._check(fence)
            # An unconfirmed release is not allowed to open this in-process gate.
            if self.fault == 'lost_release':
                raise TimeoutError
            if self.fault == 'ambiguous_release':
                return None
            if self.fault == 'wrong_release_generation':
                return replace(fence, generation=fence.generation + 1)
            if self.fault == 'wrong_release_owner':
                return replace(fence, owner=object())
            self.fence = None
            return fence


@pytest.mark.parametrize(('profile', 'kwargs'), [
    ('Really Fast', {}),
    ('Custom', {'custom_linear_mps': 2.0, 'custom_angular_radps': 0.7}),
    ('Custom', {'custom_linear_mps': 0.26, 'custom_angular_radps': 0.701}),
])
def test_policy_rejection_never_touches_the_gate(profile, kwargs):
    fake = FakePathFollowing()
    assert AutonomousSpeedApply(fake).apply(profile, **kwargs) == ApplyResult(
        ApplyStatus.INVALID_POLICY,
    )
    assert fake.calls == []
    assert fake.configured == fake.used == (0.22, 0.70)


def test_goal_that_won_admission_refuses_apply_without_mutation():
    fake = FakePathFollowing()
    apply = AutonomousSpeedApply(fake)
    assert fake.admit_goal()
    assert apply.apply('Slow') == ApplyResult(ApplyStatus.BUSY)
    assert fake.calls == ['own']
    assert fake.fence is None
    assert fake.configured == fake.used == (0.22, 0.70)
    fake.finish_goal()
    assert apply.apply('Slow') == ApplyResult(ApplyStatus.APPLIED, (0.12, 0.40))


def test_apply_that_won_admission_excludes_goal_and_concurrent_applies():
    entered, resume = Event(), Event()

    def pause_before_write():
        entered.set()
        assert resume.wait(5)

    fake = FakePathFollowing(before_write=pause_before_write)
    apply = AutonomousSpeedApply(fake)
    competing_apply = AutonomousSpeedApply(fake)
    with ThreadPoolExecutor(max_workers=1) as pool:
        first = pool.submit(
            apply.apply, 'Custom', custom_linear_mps=0.26, custom_angular_radps=0.70,
        )
        try:
            assert entered.wait(5)
            assert not first.done()
            assert not fake.admit_goal()
            assert apply.apply('Slow') == ApplyResult(ApplyStatus.BUSY)
            assert fake.calls == ['own', 'bounds']  # Same-instance refusal is local.
            assert competing_apply.apply('Slow') == ApplyResult(ApplyStatus.BUSY)
            assert fake.calls == ['own', 'bounds', 'own']
            assert fake.configured == fake.used == (0.22, 0.70)
        finally:
            resume.set()
        assert first.result(timeout=5) == ApplyResult(ApplyStatus.APPLIED, (0.26, 0.70))
    assert fake.calls == ['own', 'bounds', 'own', 'write', 'read', 'release']
    assert fake.admit_goal()


def test_known_unchanged_rejection_can_release_but_does_not_report_used_caps():
    fake = FakePathFollowing(fault='rejected')
    apply = AutonomousSpeedApply(fake)
    assert apply.apply('Slow') == ApplyResult(ApplyStatus.REJECTED_UNCHANGED)
    assert fake.calls == ['own', 'bounds', 'write', 'release']
    assert fake.configured == fake.used == (0.22, 0.70)
    assert fake.fence is None
    fake.fault = None
    assert apply.apply('Slow') == ApplyResult(ApplyStatus.APPLIED, (0.12, 0.40))
    assert fake.generation == 2


@pytest.mark.parametrize(('fault', 'calls', 'actual'), [
    ('lost_admission', ['own'], (0.22, 0.70)),
    ('wrong_admission_owner', ['own'], (0.22, 0.70)),
    ('wrong_rejection_owner', ['own', 'write'], (0.22, 0.70)),
    ('lost_before_write', ['own', 'write'], (0.22, 0.70)),
    ('lost_after_write', ['own', 'write'], (0.12, 0.40)),
    ('ambiguous_write', ['own', 'write'], (0.12, 0.40)),
    ('wrong_write_generation', ['own', 'write'], (0.12, 0.40)),
    ('failed_read', ['own', 'write', 'read'], (0.12, 0.40)),
    ('missing_read', ['own', 'write', 'read'], (0.12, 0.40)),
    ('missing_axis', ['own', 'write', 'read'], (0.12, 0.40)),
    ('ignored_linear', ['own', 'write', 'read'], (0.22, 0.40)),
    ('ignored_angular', ['own', 'write', 'read'], (0.12, 0.70)),
    ('wrong_read_generation', ['own', 'write', 'read'], (0.12, 0.40)),
    ('wrong_read_owner', ['own', 'write', 'read'], (0.12, 0.40)),
    ('lost_release', ['own', 'write', 'read', 'release'], (0.12, 0.40)),
    ('ambiguous_release', ['own', 'write', 'read', 'release'], (0.12, 0.40)),
    ('wrong_release_generation', ['own', 'write', 'read', 'release'], (0.12, 0.40)),
    ('wrong_release_owner', ['own', 'write', 'read', 'release'], (0.12, 0.40)),
])
def test_uncertainty_holds_capacity_and_never_publishes_or_retries(fault, calls, actual):
    fake = FakePathFollowing(fault=fault)
    apply = AutonomousSpeedApply(fake)
    if 'write' in calls:
        calls = ['own', 'bounds', *calls[1:]]
    assert apply.apply('Slow') == ApplyResult(ApplyStatus.BLOCKED)
    assert fake.calls == calls
    assert fake.used == actual
    if fault in ('ignored_linear', 'ignored_angular'):
        assert fake.configured == (0.12, 0.40) != fake.used
    fence = fake.fence
    assert fence is not None

    # A seemingly healthy adapter on the next call is not a reconciliation.
    fake.fault = None
    assert apply.apply('Fast') == ApplyResult(ApplyStatus.BLOCKED)
    assert fake.calls == calls
    assert not fake.admit_goal()
    assert AutonomousSpeedApply(fake).apply('Fast') == ApplyResult(ApplyStatus.BUSY)
    assert fake.calls == [*calls, 'own']
    assert fake.fence is fence
    assert fake.used == actual


def test_reused_generation_does_not_reach_write_even_with_the_new_owner():
    fake = FakePathFollowing()
    apply = AutonomousSpeedApply(fake)
    assert apply.apply('Slow').status is ApplyStatus.APPLIED
    fake.fault = 'reused_generation'
    assert apply.apply('Fast') == ApplyResult(ApplyStatus.BLOCKED)
    assert fake.calls == ['own', 'bounds', 'write', 'read', 'release', 'own']
    assert fake.used == (0.12, 0.40)
    assert not fake.admit_goal()


def test_tighter_authority_read_under_fence_rejects_without_writing():
    fake = FakePathFollowing(bounds=(0.11, 0.70))
    apply = AutonomousSpeedApply(fake)
    assert apply.apply('Slow') == ApplyResult(ApplyStatus.INVALID_POLICY)
    assert fake.calls == ['own', 'bounds', 'release']
    assert fake.configured == (0.22, 0.70)
    assert fake.fence is None
    assert apply.apply('Custom', custom_linear_mps=0.10, custom_angular_radps=0.40) == (
        ApplyResult(ApplyStatus.APPLIED, (0.10, 0.40)))


@pytest.mark.parametrize(('fault', 'bounds'), [
    ('no_bounds', (0.26, 0.70)),
    ('stale_bounds', (0.26, 0.70)),
    (None, (2.0, 0.70)),
    (None, (0.26, True)),
])
def test_missing_stale_or_invalid_authority_keeps_fence(fault, bounds):
    fake = FakePathFollowing(fault=fault, bounds=bounds)
    apply = AutonomousSpeedApply(fake)
    assert apply.apply('Slow') == ApplyResult(ApplyStatus.BLOCKED)
    assert fake.calls == ['own', 'bounds']
    assert not fake.admit_goal()
