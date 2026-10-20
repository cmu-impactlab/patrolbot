# Tuning inputs: current YAML first, documented domains second

Initial values are the robot repository's current configuration, **not** the
example defaults from a ROS web page:

- `ros2_ws/src/patrolbot_navigation/config/nav2_params.yaml` — AMCL, RPP,
  navigation smoother and behavior server.
- `ros2_ws/src/patrolbot-launch/param/defaults/smoother.yaml` — final ordinary
  drive ceilings.

Examples: `alpha1=0.05`, particles `1000–4000`, `sigma_hit=0.2`, desired cruise
`0.22 m/s`, final-approach minimum `0.08 m/s`, final ceilings `0.30 m/s` and
`0.70 rad/s`. Source values do not imply live readback, saved changes or activation.
Existing saved desired/active revisions are never replaced on refresh.

## Validation policy

An input must satisfy the intersection of the parameter's documented domain,
type and dependencies **and** this robot's narrower reviewed edit policy. A
Nav2 example/default is not an upper bound. No arbitrary ROS parameter editor,
automatic safety-cap increase or silent repair of invalid input is allowed.

| Family | Documented domain / relationships | Additional robot policy |
| --- | --- | --- |
| AMCL motion noise, update thresholds | Finite values with nonnegative noise/movement semantics; docs do not publish universal upper bounds | Existing reviewed ranges remain local tuning budgets, not ROS limits |
| Particles / beams | Integer counts; particle minimum cannot exceed maximum; beam sampling needs multiple beams | Existing CPU/resource budgets are retained and labeled as robot policy |
| `sigma_hit` | Positive Gaussian-model standard deviation; zero produces a degenerate denominator | Upper edit limit `1.0` is local policy |
| `z_hit`, `z_rand` | Mixture weights; used likelihood-field weights sum to 1 | Both weights are validated as a coherent group |
| `pf_err`, `pf_z` | Positive population error/quantile multiplier, not arbitrary motion controls | `pf_z` has no invented upper bound; documented `2.33` is accepted while YAML starts at `0.99` |
| RPP approach minimum | Nav2 Jazzy explicitly requires `min_approach_linear_velocity > 0.01` | Must also fit the selected cruise/robot speed policy |
| RPP lookahead | Correct distance/time units and ordered minimum/maximum | Existing finite reviewed ranges remain local budgets |
| RPP approach scaling distance | Docs recommend less than half the local costmap width | Robot policy enforces `< 1.5 m` for the current YAML's `3 m` costmap; a source-consistency test binds the geometry |
| Speed/angular targets | Respect producer types and dependency groups | Ordinary drive never exceeds commissioned `0.30 m/s`, `0.70 rad/s`; slower source settings are retained |

The registry records inclusive/exclusive endpoints. UI, server, core and portable
validator all use those endpoints; the UI must not invent an upper bound by
doubling the current value or mislabel an exclusive endpoint as valid.

Some existing upstream YAML angular targets (`0.9`, `1.0 rad/s`) exceed the final
`0.70 rad/s` ceiling. They remain visible as source values; the final ceiling
does not increase. Observing a legacy target is not permission to activate it or
accept an incoherent edit. The source YAML itself is unchanged by this slice.

## Official references

The stack uses ROS 2 Jazzy/Nav2, so use its version-specific docs rather than
ROS 1 `wiki.ros.org` parameter names or rolling-only features:

- [AMCL](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/others/configuring_amcl/)
- [Regulated Pure Pursuit](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/)
- [Velocity smoother](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_velocity_smoother/)
- [Behavior server](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_behavior_server/)

Docs reviewed 2026-10-18. These are source/documentation and unit-test checks,
not deployed-environment validation or proof of hot-reload support.

Source cross-checks use the pinned [Nav2 1.3.12 implementation](https://github.com/ros-navigation/navigation2/tree/1.3.12),
especially `nav2_amcl/src/pf/pf.c`, the laser models' `2*sigma_hit*sigma_hit`
denominator, RPP's `parameter_handler.cpp` and `std::clamp` lookahead, and
`nav2_velocity_smoother/src/velocity_smoother.cpp` configure-time sign checks.
Those checks do not establish live mutability or justify changing safety gates.
