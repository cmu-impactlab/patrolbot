# Hardware server LiDAR power control

`LIDAR_ON:<canonical UUID>` and `LIDAR_OFF:<canonical UUID>` use the existing
ACK/RESULT protocol and replay cache. `HELLO FEATURES=LIDAR_POWER` advertises
protocol support; the comma-separated feature list also retains `UNDOCK`.
`AUX` includes `LIDAR=STATE,supported,scan_age_ms` at 10 Hz. States are
`UNKNOWN`, `OFF`, `STARTING`, `READY`, and `FAULT`; age `-1` means unavailable.
Only a complete measured scan aged at most 500 ms permits motion. Scanner
readiness remains independent of whether power control is commissioned.

`OFF` means **commanded off**, not measured electrical power. An OK result
means the mapped power command was sent; ON remains STARTING until fresh
post-command scans arrive. Failed power commands remain FAULT until an explicit
operator request. A scan-loss fault may recover when fresh scans return; the Pi
still owns localization and explicit re-arm requirements.

Power commands are refused unless
`PATROLBOT_LIDAR_POWER_COMMISSIONED=1`, the scanner is configured, fresh base
telemetry proves the robot stationary, and no local hardware/undock operation
is pending. The Pi additionally serializes navigation and map operations.
Readiness gates ordinary DRIVE, UNDOCK, and CHARGE_RELEASE. Motor-power
requests remain available independently of dock, localization and LiDAR state;
enabling preserves E-stop and confirmed hardware readback. Zero
DRIVE and UNDOCK_CANCEL retain their existing paths. Losing scans clears drive
targets and aborts active undock; ON never restores an old motion target.

Before enabling the commissioning flag, an operator must verify the actual
firmware LRFPORT/wiring mapping, command 96 (`POWER_LRF`) behavior, electrical
readback availability, and firmware `LRFPOWER=1` for real boot-on. No direct
power port is assumed. The server does not send power commands at startup and
does not use `ArLaserConnector::connectLasers()`, whose defaults power on and
may power-cycle the scanner. ARIA configures the existing serial owner through
`parseArgs()` and reconnects asynchronously through its own scanner thread.
The existing scanner loss detector closes/reset its serial session after OFF;
ON permits serial reconnect. Quick OFF/ON baud negotiation needs real testing.

Operator OFF survives Pi/TCP and hardware-server restarts. The boot-ID-scoped
record is `/run/user/<server uid>/patrolbot_lidar_power_state`, under the
deployed systemd user's runtime directory. It is written as FAULT before power
control, then OFF or ON after a successful send. Interrupted/failed persistence
holds FAULT; a missing/unwritable runtime directory refuses control. A new SBC
boot ignores the previous boot's record. Do not delete this file to bypass a
fault; use a supervised explicit power request after diagnosing the failure.

Offline checks: `make test`. Build on the matching SBC ARIA installation with
`make`; a host-only compile against archived ARIA headers does not establish
binary compatibility with the deployed library. Never run a test server on the
live base connection. Deployment requires a binary/config backup and rollback,
then supervised stationary OFF/ON, scan-age/10 Hz telemetry checks, blocked
motion/pose/goal checks, reconnect/process-restart OFF retention, boot-on, and
fresh localization/re-arm acceptance.
