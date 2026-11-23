# Proposal: commission higher autonomous speed on the physical PatrolBot

**Review only.** The requested target is up to **2.00 m/s forward** with an
angular ceiling of **0.70 rad/s**. This proposal changes no limit and authorizes
no motion or deployment. The target is not an acceptance criterion: the outcome
may be a lower cap, a restricted operating area, or no increase.

## What limits the robot today

| Layer | Current evidence | What it means |
| --- | --- | --- |
| Chassis model | The [upstream ARIA PatrolBot-SH profile](https://github.com/qcr/libaria/blob/master/params/patrolbot-sh.p) and the [vendored copy](../ros2_ws/src/patrolbot-launch/patrolbot-sh.p) list absolute bounds of 2.20 m/s and 500°/s (8.73 rad/s). The desired velocity overrides are zero, so ARIA uses robot flash settings. | The file does not establish the installed firmware limits, measured performance, or safe operating speeds. |
| Nominal autonomous cruise | `FollowPath.desired_linear_vel` is 0.22 m/s in [Nav2 configuration](../ros2_ws/src/patrolbot_navigation/config/nav2_params.yaml). | A controller target, not a guaranteed achieved speed. Curves, approach and safety checks can slow the robot further. |
| Autonomous command envelope | The Nav2 smoother limits linear speed to 0.26 m/s; the final smoother and bridge policy cap ordinary angular motion at 0.70 rad/s. The [safety chain](../docs/safety/safety-chain.md#velocity-limits) describes both smoothers. | The lowest applicable limit wins. This is the current envelope, not approval for a higher one. |
| SBC backstop | [`classifyDrive`](../hardware/patrolbot_hw_server/protocol.cpp) rejects ordinary nonzero `DRIVE` commands above 0.30 m/s linear or 1.20 rad/s angular. | The looser angular rejection threshold does not override the upstream 0.70 rad/s cap. The current protocol cannot deliver 2.00 m/s through the ordinary path. |
| Collision monitoring | Forward velocity polygons end at 0.26 m/s; the current forward stop box ends 0.30 m ahead of `base_link`. Laser and sonar both have 0.50 s freshness timeouts. See the [configuration](../ros2_ws/src/patrolbot_navigation/config/nav2_params.yaml) and [sensor limitations](../docs/reference/limitations.md#planning-sees-one-horizontal-plane-the-rear-is-sonar-only). | Changing only a controller target, smoother, or SBC threshold does not establish protection beyond this envelope. |

At 2.00 m/s, **0.50 s corresponds to 1.00 m of travel before braking** if
the robot keeps moving during that interval. That is an illustration using one
configured timeout, not a bound on fault detection or total stopping distance.
With identical deceleration, the braking-distance term scales with speed
squared: it is about 59 times larger at 2.00 m/s than at 0.26 m/s.
Do not extrapolate the current stop box or assume software deceleration equals
measured minimum braking performance.

## Boundaries for implementation

- Apply any new envelope only to **ordinary autonomous path following**. Preserve
  the manual drive limits, autonomous reverse and recovery limits, and the
  complete guarded undock sequence: SBC reverse, owned turn, speed, acceleration,
  maneuvers, and refusal conditions. Shared smoother or gate changes must not
  widen, clip, or reroute those commands.
- Keep the watchdog, sensor freshness checks, multiplexer ownership, collision
  protection, and physical E-stop in the path. Do not grant a higher limit by
  changing the global `DRIVE` ceiling alone. Review how the downstream gate
  verifies autonomous ownership and fails closed for missing or stale
  authorization, including untrusted ROS publishers and mixed software versions.
- Enforce a combined linear/angular envelope derived from measured braking,
  visibility, curvature, and swept body clearance. A 0.70 rad/s angular cap
  does not imply that turning at that rate while traveling at 2.00 m/s is safe.
  No higher autonomous reverse speed is in scope.
- Treat lane or site restrictions as part of the commissioned envelope. Do not
  authorize high speed in shared corridors or around bystanders on the strength
  of a closed-course result. An absent, stale, or mismatched commissioning
  record must never widen the current envelope.

## Commissioning gates

| Gate | Evidence required before advancing |
| --- | --- |
| 1. Baseline and hazard review | Record immutable Pi/SBC versions, active and startup tuning, base firmware/flash settings, tires, braking condition, approved load and floor conditions, and the intended test area. Use read-only checks and existing evidence first. Verify which commands reach the base, which paths share smoothers, and how ordinary autonomy can be distinguished at the SBC without trusting UI claims or an unauthenticated velocity topic. Identify a test lead and an independent spotter with a verified physical stop, and obtain facility approval for an exclusion zone. |
| 2. Offline design and replay | Before any new-speed hardware trial, prove an explicit stop for commands outside the fully covered velocity bands and for missing/unmatched sensor, owner or policy evidence. Replay straight/turning and mixed-sign cases, obstacles at the edge of each protection zone, stale/lost/delayed sensor data, dropped or late zero commands, and reconnect/restart cases. Show that arbitration, watchdogs and collision gating still work and requests cannot raise limits during an active goal. Reject mixed revisions or unknown outcomes. No robot access is needed for this gate. |
| 3. Supervised measurement | In a closed course with no bystanders, compare baseline behavior first. Only after review of prior data should the test lead authorize each incremental speed. Use inert targets and independent position/speed measurement. For each reviewed condition record worst observed detection-to-brake latency and distance to a verified stop, with a documented allowance for sampling, timestamp uncertainty, and repeated-run variability. Include obstacle detection across the swept path, curved travel, different approved loads and floors, sensor loss, communications loss, and the independent emergency stop. Attribute sensor, Pi, SBC, and actual motion delays; do not subtract unsynchronized Pi and SBC clock readings. Abort on unexpected motion, regression, missed obstacle, insufficient clearance or uncertain evidence. |
| 4. Approval and release | For every allowed combination of speed and direction, show reliable detection range along the swept footprint exceeds the conservative worst-case stopping distance **plus** the agreed uncertainty and clearance margins. Require no unprotected point in the allowed envelope, bounded stop after failures, and successful baseline manual and undock regression. Record the approved maximum, area/conditions, evidence, approvers, and revocation criteria. Deploy high-speed support only through a separate reviewed release with an explicit default-off commissioning control. Revoke it on a material change to sensors, braking, software, or operating area. |

The stopping-distance budget includes survey/sensor error, reaction travel,
braking displacement, overshoot and a clearance margin. A useful starting model
for straight travel is `v × latency + v² / (2 × minimum deceleration)`; it is
not a substitute for measured stops, turning sweeps, or sensor coverage.
If a credible minimum deceleration or latency bound cannot be established,
or if known sensor blind spots cannot be controlled in the intended area, the
proposed speed is not approved.

## Future profile and dashboard behavior

Offer **Slow**, **Fast**, **Really Fast**, and **Custom** for ordinary autonomous
navigation. Keep today's nominal cruise as the startup behavior unless a
separate default change is reviewed. Slow should be lower than Fast but high
enough to make progress reliably; Really Fast can target at most the approved
commissioned ceiling, never an unverified chassis rating. Final preset values
other than the baseline are commissioning outputs. Custom accepts a linear and
angular limit in SI units, rejects out-of-range inputs at the robot, and displays
the confirmed effective limits rather than just the requested numbers.

Apply a change only while navigation is idle and owner state is known. A profile
change does not start motion, resume a goal, enable motors, or modify manual or
undock settings. A lost response is not success; reconcile the effective policy
before admitting another goal. Stop, cancel, fault recovery, lowering an
administrative ceiling, and manual takeover must still work regardless of the
selected profile.

Replacing the general Advanced tuning UI is separate from migrating active
configuration. Preserve the commissioned limits and non-speed overrides across
the transition, do not implicitly apply pending drafts, and keep read-only
rollback evidence. A paired dashboard change should advertise only features the
running core confirms; it must not expose the higher range to a robot that has
not passed commissioning.

<!-- SPDX-License-Identifier: Apache-2.0 -->
