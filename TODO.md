# PatrolBot TODO

Updated October 15, 2026. [Status](audits/STATUS.md) · [Open](audits/OPEN.md) ·
[Accepted scope](audits/COMPLETED.md) · [Review/deployment inventory](audits/2026-10-clean-state/REPORT.md).
[UI restoration readiness](audits/2026-10-clean-state/UI-RESTORATION-20261015.md).

Counts: PB **8 done / 18 in progress / 0 not started**; other workstreams
**22 done / 14 in progress / 2 not started** (38 total).

Current artifacts: core `2d3df1c3`, navigation `607d3524`, web `0da4b7b6`,
dashboard `04a464df`, SBC `1767110d`. Source preservation is not motion acceptance.

Next three:
1. Finish UI-only restoration release review/rollback/approval; 305 frontend, 12 focused and 28/28 layout-rerun cases pass, installed image verified.
2. Keep map/joystick/undock/override/planning repairs separate on review branches.
3. SBC is reported down; defer fresh physical/telemetry acceptance until available and separately authorized.

No goal, pose, velocity, motor-enable, charge-release or undock is authorized by
repository cleanup. Undock validation mode is **not** a zero-motion dry run.
