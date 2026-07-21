---
title: ROS 2 Services
description: PatrolBot hardware-control, dock-pose, lifecycle, and management services.
---

# ROS 2 Services

Most services in the graph are the **standard Nav2 lifecycle and management services** exposed by
the composed nodes. PatrolBot also provides typed wrappers around the existing SBC socket and an
operator-authorized dock-pose initialization service.

## Custom to PatrolBot

### Hardware control

| Service | Type | Server | Safety contract |
|---|---|---|---|
| `/patrolbot/charge_release` | `patrolbot_interfaces/ChargeRelease` | `patrolbot_bridge` | stationary/fresh/fault-free; releases charging but leaves motors disabled and sends no velocity |
| `/patrolbot/motor_enable` | `patrolbot_interfaces/MotorEnable` | `patrolbot_bridge` | stationary, charge state zero, E-stop clear, fresh state, and fault-free; never implied by `DRIVE` |

Each request carries a caller-generated lowercase canonical UUID. Responses preserve acceptance,
success, code, message, request ID, charge/motor/E-stop state, elapsed time, and the bridge's
process-local `session_generation`. Reusing a completed UUID replays the cached result without a
second hardware action.

!!! warning "These are hardware operations"
    Do not call either service as a connectivity probe. Charge release changes charger hardware;
    motor enable changes actuator power state. Follow the commissioned operator procedure and
    obtain the required physical approvals first.

### Dock-pose initialization

| | |
|---|---|
| **Service** | `/patrolbot/initialize_dock_pose` |
| **Type** | `patrolbot_interfaces/InitializeDockPose` |
| **Server** | `patrolbot_dock_manager` |
| **Purpose** | Publish the configured, operator-authorized dock pose and return only after AMCL convergence |

The service remains fail-closed while `dock_pose_configured=false`; the active configuration does
not contain a surveyed pose. It replaced the unsafe assumption that every restart happened at map
origin.

### Velocity-smoother lifecycle transition

| | |
|---|---|
| **Service** | `/teleop_velocity_smoother/change_state` |
| **Type** | `lifecycle_msgs/ChangeState` |
| **Server** | `teleop_velocity_smoother` (the mobile-base `nav2_velocity_smoother`) |
| **Client** | `lifecycle_mgr.py` (`lifecycle_manager_script`) |
| **Purpose** | At startup, the client calls `configure` then `activate` so the smoother begins publishing `/cmd_vel`. Without this, navigation output never reaches the bridge. |

```bash
# What lifecycle_mgr.py automates, by hand:
ros2 service call /teleop_velocity_smoother/change_state \
  lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"   # configure
ros2 service call /teleop_velocity_smoother/change_state \
  lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"   # activate
```

See [Nodes → lifecycle_mgr.py](nodes.md#lifecycle_mgrpy) and
[State Machines](../internals/state-machines.md).

## Inherited from Nav2

Every composed lifecycle node exposes the standard lifecycle service set, and several Nav2 servers
expose management services. The ones you are most likely to use:

| Service | Type | Server | Use |
|---|---|---|---|
| `/<node>/change_state` | `lifecycle_msgs/ChangeState` | every lifecycle node | drive a node through its [lifecycle](../internals/state-machines.md) |
| `/<node>/get_state` | `lifecycle_msgs/GetState` | every lifecycle node | query current state |
| `/global_costmap/clear_entirely_global_costmap` | `nav2_msgs/ClearEntireCostmap` | `global_costmap` | clear stale obstacles after a false mark |
| `/local_costmap/clear_entirely_local_costmap` | `nav2_msgs/ClearEntireCostmap` | `local_costmap` | clear the rolling window |
| `/<lifecycle_manager>/manage_nodes` | `nav2_msgs/ManageLifecycleNodes` | lifecycle managers | startup/shutdown/pause/resume the managed set |
| `/<lifecycle_manager>/is_active` | `std_srvs/Trigger` | lifecycle managers | check whether the managed set is active |

```bash
ros2 service list                      # see everything currently advertised
ros2 service type /<service-name>      # confirm a service's type
```

## Legacy services (`rosaria2`, not active)

The legacy [`rosaria2`](../packages/rosaria2.md) driver advertised motor-control services. They are
**not present** in the production stack because `rosaria2` is not launched:

| Service | Type | Server |
|---|---|---|
| `enable_motors` | `std_srvs/Empty` | `rosaria2_node` |
| `disable_motors` | `std_srvs/Empty` | `rosaria2_node` |

In the current architecture, sonar is enabled on the SBC at startup, while motors are explicitly
disabled. The active motor-enable interface is `/patrolbot/motor_enable`, not either legacy
`rosaria2` service.
