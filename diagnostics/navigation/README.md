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
