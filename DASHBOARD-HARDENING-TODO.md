# PatrolBot Dashboard Hardening Backlog

Status: deferred from the Pi/SBC joystick and Nav2 command-path work.

Dashboard source lives in the separate `patrolbot-dashboard` repository. This
file records the work to review there; it does not authorize dashboard
deployment or robot motion.

## Command boundary

- Keep continuous velocity control out of the dashboard. The physical Logitech
  F710 on the Pi remains the commissioned manual-control channel.
- Keep the ROS bridge outbound-only and loopback-discovered. Never expose DDS,
  `/cmd_vel`, `/initialpose`, or Nav2 actions directly to the LAN or VPN.
- Separate telemetry permission from command permission. Authenticated viewers
  must remain read-only; only an explicitly authorized operator role may send
  navigation, stop, or pose commands.
- Add a single-operator command lease. A second browser may observe but cannot
  race, replace, resume, or cancel the active operator's command without an
  explicit takeover recorded in the audit log.
- Reject motion commands unless the robot connection and base state are fresh,
  hardware state is valid, faults and E-stop are clear, motors are enabled,
  charging is inactive, localization is valid, and required navigation
  lifecycle nodes are active.
- Treat joystick/manual takeover on the Pi as final: a canceled Nav2 goal must
  not be automatically resumed by the dashboard.

## Authentication and transport

- Make production startup fail closed unless OIDC, a non-default session
  secret, a non-empty username allowlist, TLS, and a non-development robot
  credential are configured.
- Enforce command authorization server-side from the verified session role;
  hiding buttons in the frontend is not authorization.
- Validate browser WebSocket `Origin` against an explicit allowlist and reject
  cross-origin command sockets.
- Set session and OIDC cookies `Secure`, `HttpOnly`, and an appropriate
  `SameSite` policy in TLS deployments; rotate the session after login.
- Move the robot credential out of the WebSocket query string so reverse-proxy
  access logs cannot capture it. Use an authorization header or a first-frame
  challenge with constant-time comparison and a documented rotation process.
- Add strict request-size, frame-rate, and per-user command-rate limits at both
  nginx and the FastAPI gateway.

## Protocol and auditability

- Validate command UUIDs, envelope sequence monotonicity, robot identity,
  timestamps, finite numeric coordinates, and configured map bounds.
- Record requesting user, session, source IP, robot session generation,
  command payload, acknowledgment, result, rejection reason, and takeover
  events in the command audit.
- Persist duplicate protection across server restarts for the audit retention
  window instead of relying only on an in-memory 512-entry cache.
- Define reconnection behavior explicitly: browser or robot reconnects must
  never replay a goal, pose, resume, or stop request.
- Make stop/cancel completion depend on Nav2's terminal result and confirmed
  zero robot velocity, not only acceptance of a cancel request.

## Navigation short-movement investigation

- Reproduce the reported “moves briefly, then stops” symptom while correlating
  the dashboard command lifecycle with Nav2 action status and the Pi's bounded
  command-path diagnostic capture.
- If the same goal fails when sent directly on the Pi, fix the Pi/Nav2/SBC
  blocker in the core repository. If direct Nav2 succeeds, fix the dashboard
  broker/bridge lifecycle without changing collision or watchdog thresholds.
- Surface the exact terminal reason in the UI: safety hold, collision stop,
  localization loss, controller abort, cancel, timeout, or robot disconnect.

## Charging, motor power, and undock controls

- Add distinct operator controls for `Release charging`, `Enable motors`, and
  `Undock`. Do not label any of these as merely “unlock wheels” and do not
  collapse them into one unauditable command: charge release must remain
  zero-motion and motor-disabled; motor enable must remain a separate explicit
  request; undock is the later guarded reverse operation.
- Back `Release charging` with `/patrolbot/charge_release` and a fresh UUID.
  Display the accepted/result state, response code and message, request UUID,
  charge state, motor/E-stop state, and session generation. Disable the control
  unless hardware telemetry is fresh and valid, faults are clear, motors are
  disabled, and the robot is stationary.
- Back `Enable motors` with `/patrolbot/motor_enable` and a fresh UUID. Require
  a deliberate confirmation and reject it in the UI unless charge state is
  confirmed zero, E-stop is clear, hardware state is valid and fresh, faults
  are zero, odometry is stationary, and no velocity source is active.
- Expose the existing `/patrolbot/undock` action only after dock pose,
  localization, rear-clearance, bumper, E-stop, motor-enable, and physical
  acceptance work is commissioned. Show action feedback and cancellation, and
  never implement undock by publishing browser velocity.
- After release or motor enable, do not replay a prior goal or joystick value.
  Require a new, explicit operator action before motion and show a persistent
  banner whenever the robot is released from charging with motors enabled.

## Verification and rollout

- Add server tests for roles, Origin rejection, rate limits, reconnect/replay,
  duplicate persistence, stale telemetry, invalid robot state, and complete
  audit identity.
- Add bridge tests proving command mode is off by default and that the bridge
  never publishes `/cmd_vel` or opens an SBC connection.
- Make `make test` deterministic: split optional external-service tests from
  unit tests, print progress, and give every external dependency an explicit
  timeout. The 2026-07-23 baseline did not complete within the bounded
  inspection window.
- Deploy behind an explicit command-mode flag, validate telemetry-only behavior
  first, then commission pose and goal commands with an operator physically at
  the robot. Keep a checksum-backed rollback version.
