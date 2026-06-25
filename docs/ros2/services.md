---
title: ROS 2 Services
description: Services in the PatrolBot graph — the Nav2 lifecycle and clear-costmap services, the smoother state-change client, and the legacy motor-enable services.
---

# ROS 2 Services

PatrolBot is light on custom services. Most services in the graph are the **standard Nav2
lifecycle and management services** exposed by the composed nodes; PatrolBot adds one client
(`lifecycle_mgr.py`) and inherits the rest. The custom bridge defines **no** services — the SBC
link is a raw socket, not a service interface.

## Custom to PatrolBot

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

In the current architecture, motor enable/sonar enable happen on the **SBC** inside
`patrolbot_server` at startup, not via a ROS service.
