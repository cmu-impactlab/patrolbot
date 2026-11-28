# Offline autonomous speed commissioning analysis

`autonomous_stop_budget.py` checks one **straight-line distance budget** using
recorded bounds. It reads a local file only and cannot change robot settings.
It does not approve a speed, verify source evidence, or account for turning,
swept footprint, blind spots, changes to floor/load, or fault-mode coverage.

```bash
python3 diagnostics/navigation/autonomous_stop_budget.py measurements.json
```

Supply these positive finite numbers for one speed and one set of conditions:

| JSON field | Units | Input |
| --- | --- | --- |
| `speed_m_s` | m/s | Forward speed under evaluation |
| `reaction_upper_s` | s | Conservative upper bound from detection to onset of braking |
| `deceleration_lower_m_s2` | m/s² | Conservative lower bound on braking deceleration |
| `measured_stop_upper_m` | m | Upper bound from detection to verified stop, not brake onset to stop |
| `uncertainty_m` | m | Additional measurement and variability allowance |
| `clearance_margin_m` | m | Required residual clearance |
| `detection_clearance_lower_m` | m | Reliable detection clearance from the leading swept footprint, not range from `base_link` |

Use the larger of the measured stopping bound and the reaction-plus-braking
model, then add uncertainty and clearance margin. Exact equality with available
clearance does not pass. Exit codes: `0` comparison met, `1` insufficient
clearance, `2` invalid or incomplete data. Output always reports
`commissioning_approval: false`. There is intentionally no default measurement
set or integration with runtime configuration. A review of all conditions,
hardware fault evidence, ownership and site approval is still required.

## Forward speed zones (Nav2 Jazzy study)

`forward_speed_zones.py` reads a PatrolBot baseline YAML and a local JSON file
of measured bounds and proposed front edges. It neither writes config nor uses
ROS, and returns no launchable parameters. There are no example production
distances. The 1.8 m/s command limit in this tool is a **study bound only**.
Run from the repository root with Python and PyYAML:

```bash
python3 diagnostics/navigation/forward_speed_zones.py \
  ros2_ws/src/patrolbot_navigation/config/nav2_params.yaml local-evidence.json
```

| JSON location | Input |
| --- | --- |
| `leading_edge_x_m` | Measured foremost footprint x (including attachments), from `base_link` |
| `rotation_or_stopped`, `forward` | Evidence objects for the retained front stop bands, restricted to straight, nonnegative commands; include zero-command coasting |
| `extensions` | Nonempty list in increasing command order. Each entry has `command_max_m_s`, `front_edge_x_m` (a proposed +x reach from `base_link`, not clearance), and the evidence fields below. Edges may stay equal but must not shrink. |
| Each evidence object or extension | `evidence_ref` (nonempty local record identifier), `observed_speed_upper_m_s` (measured ground-speed upper bound), and `stop_budget` (exactly the seven fields above, with `speed_m_s` covering both that observed bound and the band's upper command speed) |

The study derives contiguous command bands above the baseline forward maximum.
[Jazzy selects the first band](https://api.nav2.org/nav2-jazzy/html/velocity__polygon_8cpp_source.html)
whose **commanded** x and yaw rate are both inside inclusive bounds. The
higher band wins at shared forward endpoints in the study; the baseline
stopped band still wins at the near-zero boundary. Extensions are considered
only at exactly zero commanded yaw, which does not prove the robot moves
straight. This is not a proposed runtime turn strategy.
A miss retains the previous polygon (or has none at startup); it does not
select a safe fallback. A lower or zero command can immediately select a
shorter field while the robot is still fast. Native velocity polygons do not
solve this lag.

Each check requires both the measured detection clearance and the simple
`front_edge_x_m - leading_edge_x_m` **straight geometry** to strictly exceed the
same required stop distance from `autonomous_stop_budget.py`. Geometry alone
says nothing about sufficient laser returns, lateral coverage, or swept
width. Exit `0` means only that these comparisons met the supplied bounds;
`1` means at least one did not, and `2` means incomplete or invalid input.
The tool does not verify the provenance or completeness of measurements.

For each band, evidence must cover the whole command range and all reachable
entries, including drops from faster commands into the low and stopped bands.
Use synchronized monitor-input commands and measured ground speed, not command
speed as a proxy. Worst-case stopping bounds must include acquisition,
transport, processing, downstream command and actuator delay, and braking over
relevant surfaces, loads, grades and battery conditions. Validate reliable
detection at the deployed scan height, angles and point threshold across
relevant targets, occlusions and blind spots; vendor range is not dependable
system detection clearance. Turns require separate measured swept-footprint,
detection and stopping coverage. [SICK's field-design guidance](https://www.sick.com/ag/en/the-perfect-size-of-the-protective-field-on-an-industrial-autonomous-vehicle/w/blog-size-protective-field-industrial-autonomous-vehicle)
explains why braking, latency, supplements and swept geometry matter.

The [installed LMS-200](../../docs/devices/sensors.md) is not a machine-safety
component ([SICK datasheet](https://www.sick.com/media/pdf/3/43/843/dataSheet_LMS200-30106_1015850_en.pdf)).
[Nav2 Collision Monitor](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_collision_monitor/README.md)
is not hard real-time or safety certified; this study does not establish
personnel protection. On PatrolBot the monitor filters the Nav2 branch upstream
of the final motion mux; manual commands bypass it. Both configured source
timeouts are 0.5 s, not measured reaction bounds. The laser transform uses
latest TF rather than scan-time correction because of a prior TF reconnect
crash; motion during scan and TF age still need to be accounted for. Reverse,
rear sensing, undock, manual motion, hardware ceilings and safety gates are
unchanged and outside this study. The rear velocity polygon is not extended
or checked for higher commands, so the study does not cover the whole monitor.

### Published comparison: MiR250

The [MiR250 Technical Guide, “Field set when driving forward”](https://download.astor.com.pl/dokumentacja/Mobile%20Industrial%20Robots/Roboty%20MIR/MiR250/Podr%C4%99czniki/mir250-technical-guide-10-en.pdf)
reports a **2.85 m** forward protective field for **1.70–2.10 m/s**, the band
that includes the PatrolBot study target. This is a comparison, not PatrolBot
evidence or a Nav2 parameter. The MiR250 switches fields using encoder speed
and has safety laser scanners and a safety PLC that monitors braking. PatrolBot
uses commanded velocity for field selection and does not have that protection
chain. Do not enter the MiR field length as PatrolBot detection clearance or
stopping distance; those inputs describe measurements of the tested system.

**PatrolBot study candidate:** 2.80 m of forward reach measured from the
leading edge of the robot, at the 1.80 m/s study target. This is a hypothesis
to check, not a value for `nav2_params.yaml`. Nav2 polygon coordinates are
relative to `base_link`, so an x coordinate of 2.80 m would give less than
2.80 m of clearance ahead of the robot. Use the actual leading edge,
including any attachments, rather than assuming the configured footprint
matches the measured robot. Score the candidate against PatrolBot's
measured detection and stopping bounds, including deceleration into the
shorter fields. Until those bounds are supplied, the candidate is unevaluated.

## Shared forward-speed transition model

`shared_forward_speed.py` models navigation, dashboard manual drive, and the
physical controller as three independent selections and one owner at a time.
It has no hardware or ROS adapter, does not estimate braking or sensor coverage,
and is not installed as a robot node. The 1.8 m/s and 2.8 m values are scenario
inputs, not authorization. Its two-zone crossover is the lowest current
mode default, chosen conservatively so a handoff cannot shrink the zone.
The simulated short zone and receipt timeout are explicit fixture inputs.

`test_shared_forward_speed.py` exercises the first acceptance scenario in all
three modes: faster selection while driving, a lower turning limit, then a
lower operator selection or a stop while measured speed is still high. It
also exercises independent selections at takeover, reconnect reset, and
stale samples. A single zero limit in the simulation really is a stop; a
future Nav2 adapter must translate a zero `SpeedLimit` message as **no route
restriction**, not a stop or permission to override another cap. Manual
obstacle stops and invalid samples require new motion authority. Navigation
obstacle auto-resume is tested as a policy option, not a safety sign-off.

The output is a **bounded command intent**, not a guaranteed actual speed or
an acceleration setpoint. Later integration must prove the source and age
of samples and preserve the wider zone through both smoothers and the hardware
response. It must also preserve existing reverse, recovery and undock routes,
and verify independent personnel protection before shared-hallway release.

### Read-only trace replay

`forward_speed_shadow.py` replays normalized JSON lines and prints shadow
reports; it cannot connect to ROS or the SBC. Use a common monotonic clock for
`at` (event time) and `received_at` (original receipt time). Record `owner`
events with `mode` (`navigation`, `dashboard`, `controller`) and a strictly
increasing integer `epoch`; record `selection` with its `epoch` and
`selected_m_s`; record `reset` on connection replacement. A `frame` carries
`command`, `measured`, and `clearance` samples as `{value, received_at}` or
`null`, plus `route_cap`, `turn_cap`, and `robot_cap` as numbers or `null` if
unknown. The command must also carry the original `epoch`. Every record includes
`event` and `at`. Caps are normalized (do not pass Nav2's zero sentinel as a
route cap). Reports separate the received measurement from `would_apply`;
neither a report nor a captured owner label is authorization to drive.

PatrolBot's final `Twist` and SBC `DRIVE` line do not identify the producer.
The separate opt-in `observe_forward_speed.py` tool reads `/odom`,
`/patrolbot/base_state`, and `/cmd_vel` and writes bounded JSONL output to stdout
only. It is not installed with bringup or started by any launch file. In a
sourced, isolated Jazzy test workspace, invoke it with `--duration`,
`--short-zone-m`, and `--max-age-s` for the test scenario. It never attributes
the downstream command to a drive owner or claims an effective cap. The
reported hardware generation is observed link context, not a control lease.
Sample ages are measured from local observation; they do not certify
acquisition age, odometry accuracy, or physical stopping distance. Candidate
zone output is from the study only, not a sensor field or configuration.
Unknown or stale observations are not filled in with a fabricated zero.

Use the [review and rollback checklist](forward-speed-review.md) to keep local
results separate from any later hardware acceptance.
