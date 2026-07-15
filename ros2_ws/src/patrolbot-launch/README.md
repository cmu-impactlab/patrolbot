# patrolbot-launch (source)

Source package for the mobile base launch architecture.

## Important

This is a standard colcon package. It is built/installed into
`~/ros2_ws/install/patrolbot-launch/` and launched **by package name**:
```bash
ros2 launch patrolbot-launch bringup.xml
```
That is exactly what the main Pi 5 `patrolbot-bringup` container and Pi 4
rollback systemd unit run.

Changes to launch/param files here take effect after `colcon build` (the install
tree symlinks back to this source, so XML/YAML edits are usually live immediately;
rebuild after adding new files).

> Historical note: this used to be run from a stale colcon build directory
> (`~/build_backup/patrolbot-launch/launch/bringup.xml`). That backup was removed
> 2026-06-28 and the service repointed to the installed package — never reintroduce
> a launch path that points into a build/backup directory.

## What bringup.xml starts

1. **twist_mux** — velocity multiplexer; reads `mux.yaml` for topic priorities
2. **nav2_velocity_smoother** — smooths velocity commands; remaps
   `/cmd_vel → /cmd_vel_out` and `cmd_vel_smoothed → cmd_vel`
3. **lifecycle_mgr.py** — configures and activates the velocity smoother

rosaria2 and joy teleop are commented out in `bringup.xml`
("NOT NEEDED IF PATROLBOT LIDAR SERVER IS RAN").

## Dependencies

`twist_mux`, `nav2_velocity_smoother`, `rclpy`
