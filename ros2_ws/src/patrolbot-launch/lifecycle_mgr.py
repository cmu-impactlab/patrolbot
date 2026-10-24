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
import json
import os
from pathlib import Path
import uuid

from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from patrolbot_interfaces.srv import TuningMaintenance
from patrolbot_bridge.tuning_lease import MaintenanceLease, serve

SMOOTHER = '/teleop_velocity_smoother'


class LifecycleManager(Node):
    def __init__(self):
        super().__init__('lifecycle_manager_script')
        self.change_cli = self.create_client(
            ChangeState, SMOOTHER + '/change_state')
        self.get_cli = self.create_client(GetState, SMOOTHER + '/get_state')
        self._maintenance = False
        self._legacy_maintenance = False
        self._tuning_lease = MaintenanceLease()
        self._transition_busy = False
        self.create_service(SetBool, '~/tuning_maintenance', self._set_maintenance)
        self.create_service(TuningMaintenance, '~/tuning_maintenance_v2', self._set_maintenance_v2)

    def _set_maintenance(self, request, response):
        self._legacy_maintenance = bool(request.data)
        self._maintenance = self._legacy_maintenance or self._tuning_lease.held
        response.success = not self._transition_busy
        response.message = ('Lifecycle transition is still in flight' if self._transition_busy
                            else 'Tuning maintenance acknowledged')
        return response

    def _set_maintenance_v2(self, request, response):
        # Runs on this manager's existing callback owner. A configure/activate
        # operation in flight refuses reservation; no other job can release it.
        response = serve(self._tuning_lease, request, response,
                         idle=lambda: self._transition_busy is False,
                         evidence=lambda: {'idle': not self._transition_busy,
                                           'managed_nodes': ['teleop_velocity_smoother']})
        self._maintenance = self._legacy_maintenance or self._tuning_lease.held
        return response

    def _durable_tuning_hold(self):
        default = Path(os.environ.get('PATROLBOT_STATE_DIR', '/var/lib/patrolbot')) / 'config'
        base = Path(os.environ.get('PATROLBOT_TUNING_DIR', str(default)))
        if base.is_symlink():
            return True
        if not base.exists():
            marker = base.parent / 'tuning-initialized.json'
            return (marker.exists() or marker.is_symlink()
                    or os.environ.get('PATROLBOT_TUNING_ENABLED', '0') == '1')
        manifest = base / 'manifest.json'
        try:
            if manifest.is_symlink() or manifest.stat().st_size > 512 * 1024:
                return True
            state = json.loads(manifest.read_text())
            job_id = state.get('latest_job')
            if job_id is None:
                return False
            if str(uuid.UUID(job_id)) != job_id:
                return True
            job_path = base / 'jobs' / (job_id + '.json')
            if job_path.is_symlink() or job_path.stat().st_size > 512 * 1024:
                return True
            job = json.loads(job_path.read_text())
            return job.get('state') not in ('succeeded', 'failed', 'canceled')
        except (OSError, ValueError, TypeError, AttributeError):
            # Present but unknown tuning state must never cause automatic
            # lifecycle activation over an unresolved configuration job.
            try:
                return any(path.name != 'lock' for path in base.iterdir())
            except OSError:
                return True

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
        if self._maintenance or self._durable_tuning_hold():
            return False
        req = ChangeState.Request()
        req.transition.id = tid
        req.transition.label = label
        self._transition_busy = True
        try:
            r = self._call(self.change_cli, req)
        finally:
            self._transition_busy = False
        ok = bool(r and r.success)
        self.get_logger().info(f'smoother {label}: {"ok" if ok else "FAILED"}')
        return ok

    def ensure_active(self):
        if self._maintenance or self._durable_tuning_hold():
            return
        sid = self.get_state()
        if sid is None or sid == State.PRIMARY_STATE_ACTIVE:
            # Service absent (mid-respawn) or already active: nothing to do.
            return
        if sid == State.PRIMARY_STATE_UNCONFIGURED:
            self.get_logger().info(
                'smoother UNCONFIGURED (fresh/respawned) -> bringing up')
            if self.change(Transition.TRANSITION_CONFIGURE, 'configure'):
                self.change(Transition.TRANSITION_ACTIVATE, 'activate')
        elif sid == State.PRIMARY_STATE_INACTIVE:
            self.change(Transition.TRANSITION_ACTIVATE, 'activate')


def main(args=None):
    rclpy.init(args=args)
    node = LifecycleManager()
    node.get_logger().info(
        'Lifecycle manager up — keeping smoother ACTIVE (self-heals respawns).')
    try:
        while rclpy.ok():
            node.ensure_active()
            time.sleep(2.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
