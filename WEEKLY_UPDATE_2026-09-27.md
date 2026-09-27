# PatrolBot — Weekly Progress Update
**27 September 2026 · Issue #60**

## Goal
Make localization reliable across the SBC, Raspberry Pi, Nav2 and dashboard, including recovery after connection loss or restarts. Close issue #60 once software and real-robot acceptance checks are complete.

## Completed work and why
- **Improved hardware-state and odometry handling.** Deployed the SBC origin-identification update so the Pi can distinguish a connection interruption from a real odometry reset and hold navigation until localization is valid.
- **Fixed an AMCL initialization problem.** The candidate now rejects scans older than the operator’s pose estimate, allowing a stationary robot to produce a properly timed localization update.
- **Repaired several Nav2 and Python shutdown failures.** Isolated component, launch and end-to-end recovery tests pass, improving restart and recovery reliability. One intermittent managed-shutdown timeout still needs investigation.
- **Reconciled source code with the deployed robot.** Published existing safety safeguards and configuration to GitHub, preventing future builds from accidentally restoring older behavior.
- **Built and verified a combined candidate.** Checked source/image provenance, installed files, AMCL behavior and 42 DockManager tests. Short sensor-outage tests confirm a zero-command hold, recovery without restart and clean shutdown.
- **Prepared deployment and rollback.** Identified the exact three source-file changes, pinned the candidate image and tested backup/restore failure scenarios. Progress is recorded in GitHub and the project audits.

## Planned next steps
1. Complete the eight-hour outage test and investigate the intermittent shutdown timeout; a successful diagnostic rerun alone does not resolve it.
2. Deploy the validated combined Pi candidate with verified rollback and fresh hardware preflight.
3. With the SBC available, verify live sensor/TF timing, operator-controlled pose initialization and at least 90 minutes of valid stationary localization.
4. Check wall/map alignment when measurements are available, then perform separately authorized, supervised navigation and undocking tests.

## Current limits
The combined candidate is **not deployed yet**, and physical localization accuracy is **not accepted**. The SBC was offline at the last operator report. Wall geometry measurements remain pending; dashboard frontend/server deployment is deferred to the operator.

*Meeting snapshot. Authoritative ongoing status: [audits/OPEN.md](audits/OPEN.md).*
