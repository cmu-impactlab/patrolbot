#!/usr/bin/env python3
"""
Persistent lifecycle manager for the teleop_velocity_smoother.

Originally a one-shot (configure+activate then exit). Made PERSISTENT so the
smoother self-heals: if the velocity_smoother node crashes and is respawned by
the launch (respawn="true"), it comes back UNCONFIGURED — this manager detects
that and re-configures+activates it within a couple of seconds, restoring the
cmd_vel path without a full bringup restart.
"""
import time
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State

SMOOTHER = '/teleop_velocity_smoother'


class LifecycleManager(Node):
    def __init__(self):
        super().__init__('lifecycle_manager_script')
        self.change_cli = self.create_client(ChangeState, SMOOTHER + '/change_state')
        self.get_cli = self.create_client(GetState, SMOOTHER + '/get_state')

    def _call(self, client, req, timeout=5.0):
        if not client.wait_for_service(timeout_sec=2.0):
            return None
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def get_state(self):
        r = self._call(self.get_cli, GetState.Request())
        return r.current_state.id if r else None

    def change(self, tid, label):
        req = ChangeState.Request()
        req.transition.id = tid
        req.transition.label = label
        r = self._call(self.change_cli, req)
        ok = bool(r and r.success)
        self.get_logger().info(f'smoother {label}: {"ok" if ok else "FAILED"}')
        return ok

    def ensure_active(self):
        sid = self.get_state()
        if sid is None or sid == State.PRIMARY_STATE_ACTIVE:
            return  # service absent (mid-respawn) or already active -> nothing to do
        if sid == State.PRIMARY_STATE_UNCONFIGURED:
            self.get_logger().info('smoother UNCONFIGURED (fresh/respawned) -> bringing up')
            if self.change(Transition.TRANSITION_CONFIGURE, 'configure'):
                self.change(Transition.TRANSITION_ACTIVATE, 'activate')
        elif sid == State.PRIMARY_STATE_INACTIVE:
            self.change(Transition.TRANSITION_ACTIVATE, 'activate')


def main(args=None):
    rclpy.init(args=args)
    node = LifecycleManager()
    node.get_logger().info('Lifecycle manager up — keeping smoother ACTIVE (self-heals respawns).')
    try:
        while rclpy.ok():
            node.ensure_active()
            time.sleep(2.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
