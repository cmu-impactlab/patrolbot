# Session 1 — pre-change baseline

Captured 2026-08-11, robot docked throughout, no motion command issued.

All of this was taken **before** any container restart, so the AMCL evidence below is
the original degenerate state (the navigation container had been up 12 days).

## Topic rates (20 s windows, nav container)

| Topic | Rate | min / max period | std dev |
|---|---|---|---|
| `/odom` | 22.71 Hz | 0.024 / 0.064 s | 0.0036 s |
| `/scan` | 22.76 Hz | 0.014 / 0.074 s | 0.0041 s |
| `/tf` | 69.6 Hz | 0.000 / 0.070 s | 0.0093 s |
| `/amcl_pose` | 3.13 Hz | 0.165 / 0.438 s | 0.099 s |

## TF chain

`map -> odom` present:

```
- Translation: [99.062, 126.274, 0.000]
- Rotation: in RPY (radian) [0.000, 0.000, -1.852]   (-106.084 deg)
```

`odom -> base_link` present:

```
- Translation: [0.141, 0.016, 0.000]
- Rotation: in RPY (radian) [0.000, -0.000, 0.222]   (12.711 deg)
```

## AMCL covariance — degenerate, confirmed

`amcl_cov_before.csv`, 60 s capture, **188 samples, 188 of which have a
non-positive diagonal entry**. Every sample is identical; the filter is frozen.

```
x=99.038192  y=126.133962  yaw=-1.629663
cov[0]  (xx)     =  8.253264e-10
cov[7]  (yy)     = -6.219768e-10     <- negative
cov[35] (yaw,yaw)= -6.128431e-14     <- negative
cov[1]/cov[6] (xy) = 3.306845e-10
cov[5]/cov[30]     = 0.000000e+00
```

A covariance matrix with negative diagonal entries is not merely small, it is not a
covariance matrix at all. Step 1's acceptance criterion is met with no ambiguity: the
filter did **not** self-recover.

Note that `/patrolbot/dock_readiness` reported `localization_valid: true` and
`ready_for_undock: true` against exactly this covariance, which is the defect change 1b
addresses.

## `/odom` covariance

Both the pose and twist covariance blocks in `/odom` are **all zeros**. The SBC bridge
publishes no uncertainty estimate at all. Recorded here as a fact for later sessions;
not changed in this session.

## AMCL parameters

Full dump in `amcl_params_before.yaml`. The values this session cares about:

```
alpha1: 0.1          alpha2: 0.2      alpha3: 0.2
alpha4: 0.1          alpha5: 0.2
recovery_alpha_slow: 0.0             recovery_alpha_fast: 0.0
set_initial_pose: false              always_reset_initial_pose: false
resample_interval: 1                 transform_tolerance: 1.0
min_particles: 500                   max_particles: 1500
update_min_d: 0.1                    update_min_a: 0.1
```

## Base state at capture time

`dock_state: DOCKED_CONFIRMED`, `dock_phase_name: HANDOFF_READY`,
`charge_state: 3`, `front_bumper_pressed: true` (docked against the charger),
`minimum_rear_range: 4.331 m`, `session_generation: 3`, `fault_flags: 0`.
