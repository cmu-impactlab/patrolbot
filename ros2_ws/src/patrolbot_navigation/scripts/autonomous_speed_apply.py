"""Offline-only contract for applying ordinary autonomous path-following caps."""

from dataclasses import dataclass
from enum import Enum, auto
from threading import Lock
from typing import Protocol

from autonomous_speed_policy import resolve_autonomous_speed


Caps = tuple[float, float]  # linear m/s, angular rad/s


@dataclass(frozen=True)
class Fence:
    owner: object
    generation: int


class WriteOutcome(Enum):
    APPLIED = auto()
    REJECTED_UNCHANGED = auto()


@dataclass(frozen=True)
class WriteReceipt:
    fence: Fence
    outcome: WriteOutcome


@dataclass(frozen=True)
class CommissionedCaps:
    fence: Fence
    caps: Caps


@dataclass(frozen=True)
class UsedCaps:
    fence: Fence
    caps: Caps


class OfflinePathFollowingAdapter(Protocol):
    """Required fake-adapter semantics, not evidence that Nav2 supports them.

    One gate must serialize idle acquisition with all ordinary path-following
    goal admission and cap writes. The fence remains closed to both other
    writers and new goals until confirmed release; it never expires here.
    Every operation must check owner identity and generation at the gate.
    This gate does not govern stop, cancel, manual, or undock.
    """

    def try_own_idle(self, owner: object) -> Fence | None:
        """Atomically claim idle with a new increasing generation, or return None
        for a definitive refusal with no claim or cap mutation. If an acquired
        claim cannot be confirmed, it remains held.
        """
        ...

    def write_caps(self, fence: Fence, caps: Caps) -> WriteReceipt | None:
        """Write only this pair, atomically. REJECTED_UNCHANGED proves no mutation
        and no deferred write. None or an exception is not proof either way.
        """
        ...

    def read_commissioned_caps(self, fence: Fence) -> CommissionedCaps | None:
        """Read the current authoritative bounds under the same writer fence.

        Caller-selected caps and a stale read before acquiring idle are not
        authority. These bounds must remain stable until release.
        """
        ...

    def read_used_caps(self, fence: Fence) -> UsedCaps | None:
        """Read both caps in use as one fence-bound snapshot, not a write echo."""
        ...

    def release(self, fence: Fence) -> Fence | None:
        """Check ownership and open only as one confirmed in-process operation.

        An exception or unconfirmed result must leave goal admission fenced.
        An RPC that can open the gate before losing its reply is not sufficient.
        """
        ...


class ApplyStatus(Enum):
    APPLIED = auto()
    BUSY = auto()
    INVALID_POLICY = auto()
    REJECTED_UNCHANGED = auto()
    BLOCKED = auto()


@dataclass(frozen=True)
class ApplyResult:
    status: ApplyStatus
    used_caps: Caps | None = None


def _matches(candidate: object, expected: Fence) -> bool:
    return (
        type(candidate) is Fence
        and candidate.owner is expected.owner
        and type(candidate.generation) is int
        and candidate.generation == expected.generation
    )


class AutonomousSpeedApply:
    """Single-flight coordinator. No retries, rollback, or recovery path."""

    def __init__(self, adapter: OfflinePathFollowingAdapter):
        self._adapter = adapter
        self._flight = Lock()
        self._blocked = False
        self._last_generation = -1

    def apply(
        self,
        profile: str = 'Fast',
        *,
        custom_linear_mps: float | None = None,
        custom_angular_radps: float | None = None,
    ) -> ApplyResult:
        if not self._flight.acquire(blocking=False):
            return ApplyResult(ApplyStatus.BUSY)
        try:
            if self._blocked:
                return ApplyResult(ApplyStatus.BLOCKED)
            try:
                # Reject malformed or out-of-envelope requests even before
                # claiming capacity. The trusted ceiling is read after that.
                resolve_autonomous_speed(
                    profile,
                    custom_linear_mps=custom_linear_mps,
                    custom_angular_radps=custom_angular_radps,
                )
            except ValueError:
                return ApplyResult(ApplyStatus.INVALID_POLICY)

            # Latch before the first operation that could leave an owned fence.
            # Clear only on definitive refusal or matching release confirmation.
            self._blocked = True
            return self._apply_owned(profile, custom_linear_mps, custom_angular_radps)
        finally:
            self._flight.release()

    def _apply_owned(self, profile, linear, angular) -> ApplyResult:
        blocked = ApplyResult(ApplyStatus.BLOCKED)
        owner = object()
        try:
            fence = self._adapter.try_own_idle(owner)
            if fence is None:
                self._blocked = False
                return ApplyResult(ApplyStatus.BUSY)
            if (type(fence) is not Fence or fence.owner is not owner
                    or type(fence.generation) is not int
                    or fence.generation <= self._last_generation):
                return blocked
            self._last_generation = fence.generation

            allowed = self._adapter.read_commissioned_caps(fence)
            if (type(allowed) is not CommissionedCaps or not _matches(allowed.fence, fence)
                    or type(allowed.caps) is not tuple or len(allowed.caps) != 2):
                return blocked
            max_linear, max_angular = allowed.caps
            # Invalid authority and a valid but narrower authority have
            # different outcomes. Validate the authority before the request.
            resolve_autonomous_speed(
                'Custom', custom_linear_mps=0.0, custom_angular_radps=0.0,
                commissioned_linear_mps=max_linear, commissioned_angular_radps=max_angular,
            )
            try:
                caps = resolve_autonomous_speed(
                    profile, custom_linear_mps=linear, custom_angular_radps=angular,
                    commissioned_linear_mps=max_linear, commissioned_angular_radps=max_angular,
                )
            except ValueError:
                if not _matches(self._adapter.release(fence), fence):
                    return blocked
                self._blocked = False
                return ApplyResult(ApplyStatus.INVALID_POLICY)

            write = self._adapter.write_caps(fence, caps)
            if type(write) is not WriteReceipt or not _matches(write.fence, fence):
                return blocked
            if write.outcome is WriteOutcome.REJECTED_UNCHANGED:
                result = ApplyResult(ApplyStatus.REJECTED_UNCHANGED)
            elif write.outcome is WriteOutcome.APPLIED:
                used = self._adapter.read_used_caps(fence)
                if (type(used) is not UsedCaps or not _matches(used.fence, fence)
                        or type(used.caps) is not tuple or len(used.caps) != 2
                        or any(type(value) not in (int, float) for value in used.caps)
                        or used.caps != caps):
                    return blocked
                result = ApplyResult(ApplyStatus.APPLIED, used.caps)
            else:
                return blocked

            if not _matches(self._adapter.release(fence), fence):
                return blocked
            self._blocked = False
            return result
        except Exception:
            # The adapter must retain the gate on uncertainty. Do not attempt
            # anything else (including release) based on an ambiguous reply.
            return blocked
