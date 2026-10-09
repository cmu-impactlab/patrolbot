# Coordinated floor-map candidate (#66)

This changes the robot/dashboard interface together. Hardware acceptance remains
open under [#66](https://github.com/cmu-impactlab/patrolbot/issues/66) and #65.
Do not deploy one half independently. No automatic floor detection, elevator
handling, or between-floor navigation is implemented.

## Behavior and interface

- Browsing Floor 1 or Floor 2 sends no robot command. Set location sends a
  `LocalizeOnMap` action via the existing dashboard command.
- `map_context_v1`, catalog map ID, and content revision are required for spatial
  commands. Old clients and bridges are explicitly rejected.
- Map-server readback and AMCL consumption confirm activation separately from
  fresh localization. Only the confirmed active floor is red.
- The existing bridge motion guard holds the shared `/cmd_vel` path through
  lifecycle reset, map loading, one pose seed, localization, and costmap refresh.
  Navigation starts configured but inactive to avoid waiting for a seed while
  holding the lifecycle-manager service needed by that seed transaction.
- A bounded no-motion scan update can refresh localization after slow costmap
  activation. The transaction never republishes its pose seed.
- Cancellation, disconnected transport, stale/moving base telemetry, timeout,
  or partial failure retain the hold. No automatic rollback or old-goal resume.
- The durable `patrolbot-state` volume stores `active-map.json` atomically. An
  absent selection in an available volume defaults to Floor 2. Corrupt state,
  missing volume, unknown maps and changed revisions fail closed. Restart always
  requires fresh localization. Never delete state to bypass a failed readback.
- Dock coordinates and guarded initialization belong to Floor 2 only.

## Reproducible offline checks

`docker/Dockerfile.floor-validation` builds pinned Nav2 1.3.12 with the complete
local patch series, typed interfaces, bridge, dock manager/plugin and navigation.
Run it only as an isolated validation image, not on the production robot.
`docker/tests/integration_floor_maps.py` asserts network-none isolation and uses
synthetic scan rays against the canonical rasters with a loopback fake SBC.
Its poses and covariance are test fixtures, not commissioning measurements.

From the robot repository, package both current source trees with:

```sh
python3 docker/package_floor_release.py \
  --dashboard ../patrolbot-dashboard \
  --output audits/2026-10-floor-maps/artifacts/candidate
```

The two archives and manifest include source HEADs, dirty-worktree indicators,
per-file SHA-256 hashes, archive hashes, map revisions and source rollback
references. Packaging fails on a robot/dashboard catalog mismatch. These are
reviewable source artifacts; they do not assert a production image is deployed.

## Deployment and rollback gates

Before a separately authorized deployment, capture the actually running robot
and dashboard image digests, compose configuration, and a backup of the durable
state volume. Record those concrete rollback references in `audits/OPEN.md`.
Build both images from the manifest sources with matching interfaces and verify
catalog parity during dashboard packaging. The robot candidate build must
include the updated docking plugin as well as the bridge/interface packages.

On a failed rollout, retain the hold. An authorized rollback restores the
coordinated robot/dashboard image pair and the matching catalog/state snapshot;
it does not assert physical localization survived. Require fresh localization
again. Source backups from this implementation are under each repository's
`backups/20261007-floor-maps/` and `backups/20261008-floor-followup/`.

Hardware acceptance, once the SBC is available and the operator authorizes it:
read back each map, check physical scan/map alignment, verify fresh localization,
prove the stationary hold with motors enabled, and test same-floor navigation.
Offline success cannot close this acceptance.
