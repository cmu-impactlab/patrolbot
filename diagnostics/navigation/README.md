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
