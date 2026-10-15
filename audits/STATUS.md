# PatrolBot — current status

Updated October 15, 2026. Current source/runtime evidence overrides older
snapshots; GitHub issue state is recorded separately and is not hardware proof.
See [open work](OPEN.md), [accepted scope](COMPLETED.md), and the
[cleanup/review inventory](2026-10-clean-state/REPORT.md).

**UI restoration candidate, not deployed:** missing uncertainty display and
independent motor-disable control restored over the exact deployed baseline;
305 frontend, 12 focused Chromium/WebKit cases and isolated installed-image/HTTP
checks pass. A 60-case responsive-layout probe under a constrained local
container finished 44 passed / 16 failed on timeouts/slow loads only; a
single-worker rerun is testing the resource hypothesis. Candidate `9a9235d3`
changes static assets only; 49 backend/map files unchanged. Layout/release
review and fresh rollback/admission/approval remain. Operator reports SBC down;
no SBC access or robot command. Counts unchanged.
See [readiness evidence](2026-10-clean-state/UI-RESTORATION-20261015.md).

The running robot and dashboard have dedicated `deployed/*` source branches and
read-only hash manifests. Unfinished behavior remains on `review/*` branches.
No robot behavior was changed by this cleanup. Current dashboard UI regressions,
map mismatch, joystick defects and incomplete physical acceptance remain open.

## Counts

| State | PB findings | Other workstreams |
|---|---:|---:|
| ✅ Done | 8 | 22 |
| 🟨 In progress | 18 | 14 |
| 🔲 Not started | 0 | 2 |
| Total | 26 | 38 |

Corrections to previous counts: PB020 reopens for build/publication provenance;
documentation reconciliation reopens; gimbal moves from not-started to
in-progress because implemented code already exists. No GitHub issue was closed.

## PB findings

| ID | State | Current scope / remaining |
|---|---|---|
| 001 | ✅ | Generic docking excluded; single final velocity path historically verified. |
| 002 | 🟨 | Historical successful undocks retained; current admission regression/software repair and supervised acceptance remain. |
| 003 | 🟨 | Both-tree physical reverse/recovery acceptance; pending native guard changes are separate. |
| 004 | 🟨 | Sonar consumers exist; physical obstacle/dropout acceptance remains. |
| 005 | 🟨 | Low-obstacle sensing scaffold; hardware and commissioning remain. |
| 006 | 🟨 | Panel mapping/FLASH readback accepted; empirical motion cutoff remains. |
| 007 | 🟨 | Explicit map-aware seed flow exists; physical alignment/navigation remain. |
| 008 | 🟨 | Bumper-inclusive footprint measurement and doorway acceptance remain. |
| 009 | 🟨 | Auto-enable removed; enabled-motor E-stop power removal remains. |
| 010 | 🟨 | Clock synchronization exists; RTC battery/cold boot remain. |
| 011 | ✅ | Charging-state interface and historical transition verified. |
| 012 | 🟨 | Voltage/fault holds exist; real power-fail input/shutdown remain. |
| 013 | ✅ | SBC journaling/autostart historically verified. |
| 014 | ✅ | Jazzy smoother parameters and speed cap historically verified. |
| 015 | 🟨 | Installed manual-drive defect; handshake candidate not deployment-ready. |
| 016 | ✅ | Historical unclean-power-loss cause established. |
| 017 | 🟨 | Unauthenticated SUBNET DDS remains explicitly exposed. |
| 018 | 🟨 | 0.10 m cutoff installed; reflection/costmap/collision acceptance remains. |
| 019 | ✅ | Historical dead-artifact review; new snapshots are separately inventoried. |
| 020 | 🟨 | Observed source hashes captured; immutable clean rebuild and approved publication/rollout remain. |
| 021 | 🟨 | Laser height datum and physical measurement remain. |
| 022 | 🟨 | Hardened but unsupported i686 SBC still needs replacement. |
| 023 | 🟨 | Gyro firmware evidence exists; physical contribution proof remains. |
| 024 | ✅ | Protocol design review accepted for its original scope. |
| 025 | ✅ | Historical interface-drift fix; future releases must reverify ABI. |
| 026 | 🟨 | Parked endurance accepted; current guarded-undock/moving accuracy remain. |

## Other workstreams

| Item | State | Scope |
|---|---|---|
| GitHub organization | ✅ | Historical tracker cleanup; new corrections proposed, not applied. |
| SBC-owned undock mode | ✅ | Original mode accepted; later admission regression is PB002. |
| Joystick speed caps | ✅ | Historical alignment retained. |
| Joystick USB hotplug | ✅ | Historical receiver replug accepted. |
| Controller haptics | ✅ | Historical alert-only physical vibration accepted. |
| Web image derivation | ✅ | Existing derivation retained; exact release ABI still verified separately. |
| Interface hash gate | ✅ | Implemented readiness check. |
| Battery unsupported SoC | ✅ | Measured voltage/unknown percentage behavior accepted. |
| Scan timestamp / laser frame | 🟨 | Acquisition path exists; geometry/latency acceptance remains. |
| Compose web bridge | ✅ | Active service, commands deliberately enabled. |
| Dashboard DNS | ✅ | Public HTTPS/WSS endpoint working. |
| Single-identity historical cutover | ✅ | Historical scope only; current per-role releases differ deliberately. |
| Provenance regression | ✅ | October 3 closure retained; new regression tracked under PB020. |
| Undock hardware profile | 🟨 | Actual profile captured; all physical commissioning criteria remain. |
| SBC source/binary divergence | ✅ | Historical closure; current executable/source captured, not rebuilt here. |
| Malformed dashboard requests | ✅ | Historical deployed negative-ACK/audit acceptance. |
| Dashboard hardening | 🟨 | Current UI regressions plus remaining command-boundary criteria. |
| Override logger crash | 🟨 | Logger fix exists; authorized actual-path acceptance remains. |
| Computer power controls | 🔲 | Deferred; software reset is not power-on/off. |
| Gimbal integration | 🟨 | July driver/joy/mux/protocol/tests exist; integration/hardware deferred. |
| LiDAR power | 🟨 | Software/FLASH substeps accepted; electrical/boot/controller acceptance remains. |
| Backup consolidation | ✅ | October 15: 36 exact snapshots archived/verified off-device and retired. |
| Documentation reconciliation | 🟨 | Current tracked index corrected; historical architecture references remain. |
| Parked pose validity | 🟨 | 90-minute docked substep accepted; moving/clear-of-dock criteria remain. |
| Odometry covariance | ✅ | Historical deployed covariance accepted. |
| AMCL covariance root cause | 🟨 | Measured initialization exists; geometry/moving/patrol/undock acceptance remains. |
| Localization gate consistency | 🟨 | Per-axis implementation exists; full end-to-end age/motion matrix remains. |
| Convergence margin | 🔲 | Superseded criteria retained under #48/#47; no duplicate implementation. |
| Bridge reconnect log | ✅ | Historical evidence limitation accepted; no fabricated reconstruction. |
| Uncertainty display | 🟨 | Merged implementation absent from current frontend; restoration/usability remain. |
| Package metadata | 🟨 | Metadata repaired; rosaria2 MIT/GPLv2 provenance unresolved. |
| Odometry validation | ✅ | Historical malformed/finite-telemetry acceptance retained. |
| Reconnect epochs | ✅ | October 11 link-only continuity accepted. |
| Pi-only readiness | ✅ | Existing bounded degraded-state report accepted. |
| Outage warning storm | ✅ | Historical soak/recovery acceptance retained. |
| Pi power integrity | 🟨 | Representative electrical/thermal load acceptance remains. |
| Physical localization | 🟨 | Stationary wall relocation failures and current map mismatch remain. |
| First-floor map software | ✅ | Original #66 software acceptance retained; later revision mismatch is current PB020/#65 work. |
