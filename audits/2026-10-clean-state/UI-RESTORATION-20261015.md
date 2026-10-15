# First deployment problem: dashboard UI restoration

State: 🟨 candidate tested, **not deployed**. Operator reports SBC down; no SBC
access, pose/goal/velocity/motor/charge/undock request or production restart.

## What existed before this continuation

Dashboard `d4925f8` implements filter spread/signed variance margins and stale
AMCL presentation; `4ffc514` completes independent motor-disable visibility for
fresh/default-off and invalid telemetry. Both are merged in main. GitHub runs
38428091701 (CI) and38428091766 (Mobile browser acceptance) are success on
211fc8a, with historical304 frontend/live-asset evidence retained in the October13
clean-baseline record. Current served asset92453189 lacks those fixes.

The running backend and Pi web already provide the optional uncertainty fields.
Therefore this restoration requires no ROS/interface/backend/map change, no
database migration and no Pi/SBC restart. It is not an override or motion fix.

## Work prepared now

Branch `review/dashboard-ui-restoration-20261015` starts at actual deployed source
749cb18, not a blanket main checkout. Four runtime/type files are byte-identical
to the previously reviewed merged fixes: localizationUncertainty.ts, protocol.ts,
RobotStatusWidget.tsx and NavControlsWidget.tsx. Existing server/web/map source
and command gates remain unchanged. Added tests preserve account, connection,
capability and duplicate-request boundaries in both displayed motor states.

Candidate source commit: `fa94998559ec06af5b26550097f526d38a144fad`.
Subsequent candidate-branch changes are verification/documentation only.

## Fresh evidence

| Check | Result and scope |
|---|---|
| Deployed widgets with the same focused contracts | 11 fail /61 pass; confirms both UI regressions. |
| Restored widget/helper contracts | All72 pass after restoration. |
| Full frontend suite after extra refusal tests | All305 pass (297 existing +8 additional guards). |
| TypeScript build, browser types, bundle budget | Pass; initial static JS155.3kB gzip below175kB budget. |
| Restored built entry | SHA2568299aab51de57519e19ac86b94e1c3b878052216f95ced92a0d6cf27e20b029e, identical to previously accepted October13 bundle. |
| Focused browser checks | All12 pass across Chromium/WebKit desktop/phone/tablet projects in pinned isolated container. |
| Production-derived candidate | Immutable image9a9235d31278d9959855b693cfced11336ac23a0277e70f7bcb45840bef96529. |
| Installed preservation | Base layers, environment/command/user retained; all49 application/map files unchanged; all15 static files match reviewed build. |
| Actual isolated HTTP | Health and all15 assets pass; robot_connected=false, no live credentials/data or external network. Application cleanup completes. |

Browser image: `mcr.microsoft.com/playwright@sha256:eff16c30e6f3f4af0a03fa4b706120d5e9b0891c344a27d64559aff5900a4a27`.
Application base: `sha256:04a464df6668b2197795ed965f4c1704f180a7b08bf733dcd2086af47e9240bc`.
Frontend tree: `13845994ba5e6ffb996f82683c23bb45c45155d8b82e129eb2ee1d95acfd40c6`.

Commands run in the isolated dashboard worktree: `npm test -- --run`,
`npm run build`, `npm run check:e2e`, `npm run check-bundle`, focused Playwright
localization-status cases, and
`python3 -B scripts/check_ui_image.py infrastructure/releases/ui-restoration-20261015.candidate.json`.
The image checker creates only network-none, cap-dropped, read-only temporary
test containers with tmpfs data. Browser API/WebSocket routes are mocked; fake
motor-disable clicks do not reach a real robot.

Private logs/build context/image inputs:
`patrolbot-dashboard/backups/20261015T145435Z-ui-restoration/`.

### Failed harness/environment attempts, not erased

- Initial host WebKit6 cases failed before launch: missing libicudata.so.74.
  Pinned Ubuntu browser container resolves the host mismatch.
- First container attempt could not write owner-private paths as cap-dropped
  root. Running as the workstation owner passes with the same test/isolation.
- First isolated smoke required exit0 after SIGTERM. Installed Uvicorn0.52.1
  intentionally re-raises captured signals after application cleanup (verified
  from installed source). Corrected check accepts -SIGTERM only with complete
  application-shutdown and finished-process logs. No product behavior changed.

## Before deployment

1. Finish selected responsive/layout checks and review the final narrow diff/image.
2. Verify current production image/source/assets have not drifted. Prepare fresh
   verified off-device rollback/config/SQLite preservation and previous-image
   retrieval; no durable device snapshot is needed.
3. Obtain explicit approval for server-only image recreation. Do not run a full
   mutable dependency rebuild or restart nginx/Pi/SBC as a shortcut.
4. After actual activation, verify public assets/authentication/health, unchanged
   app/maps/env/data, source branch and release manifest. No robot command is
   needed for UI/read-only checks.
5. Fresh live AMCL display and actual motor response remain separate acceptance
   when SBC is available and the operator authorizes it. An unavailable/unknown
   readback must remain truthful; UI software tests are not physical acceptance.

Other map/joystick/undock/override/planning issues stay open. No issue/count
closure, production activation or source-parity claim for this candidate yet.

Do not widen `max_position_variance` — it also gates general navigation; do not
flip `allow_reversing` — mutually exclusive with enabled `use_rotate_to_heading`;
do not install graceful shutdown without real power-fail input; do not change
footprint/inflation together; do not restore DDS to SUBNET; do not raise
SCAN_RANGE_MIN to make the stop box see nearer. Do not bypass unknown outcomes,
guess physical values, auto-enable motors or resume old goals.
