"""Fixed navigation-container restart protocol; no Docker access in ROS nodes.

The core already authenticates Apply and journals its exact reviewed job. A
host-only helper validates that journal, restarts ONE existing container and
hands the candidate to ONE replacement boot. Ordinary boots still use ACTIVE.
No second signing system, coordinator handoff, build, pull or SBC operation.
"""

import json
import socket

from .tuning_lease import context
from .tuning_plan import canonical_digest
from .tuning_store import StoreUnavailable

CORE_SOCKET = '/run/patrolbot-tuning/core/supervisor.sock'
BOOT_SOCKET = '/run/patrolbot-tuning/boot/supervisor.sock'
MAX_REPLY = 512 * 1024


def request(path, value, *, timeout=15.):
    """One bounded local request. A timeout is unknown, never retried here."""
    from .tuning_json import loads
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(timeout)
        connection.connect(str(path))
        connection.sendall(json.dumps(value, allow_nan=False).encode() + b'\n')
        with connection.makefile('rb') as stream:
            data = stream.readline(MAX_REPLY + 1)
    if len(data) > MAX_REPLY or not data.endswith(b'\n'):
        raise ValueError('invalid navigation helper reply')
    result = loads(data.decode(), MAX_REPLY)
    if not isinstance(result, dict) or result.get('ok') is not True:
        raise RuntimeError('navigation helper operation refused or unavailable')
    return result


def candidate_for_job(store, value):
    """Revalidate immutable files without waiting for Apply's writer lock.

    The live coordinator retains that lock while awaiting restart. The helper
    only reads atomic journals and immutable artifacts; it never promotes or
    writes the robot store. A second read must still bind the same running job.
    """
    value = context(value)
    manifest = store.require_ready()
    job = store.get_job(value['job_id'])
    if (manifest.get('latest_job') != value['job_id'] or not isinstance(job, dict)
            or job.get('state') != 'running' or job.get('mutation_started') is not True
            or job.get('input_gate_attempted') is not True
            or any(job.get(key) != item for key, item in value.items())):
        raise StoreUnavailable('nav2_restart_requires_running_exact_job')
    plan = job.get('execution_plan')
    if (not isinstance(plan, dict) or plan.get('strategy') != 'hot_or_nav2_redeploy'
            or plan.get('execution_kind') != 'nav2_redeploy'
            or plan.get('services') != ['navigation'] or plan.get('image_rebuild') is not False
            or plan.get('seed_required') is not True or plan.get('executable') is not True
            or plan.get('reasons') != [] or plan.get('plan_hash') != value['plan_hash']
            or plan['plan_hash'] != canonical_digest({key: item for key, item in plan.items() if key != 'plan_hash'})
            or plan.get('revision') != job.get('revision') or plan.get('digest') != value['digest']):
        raise StoreUnavailable('nav2_restart_requires_reviewed_navigation_only_plan')
    pointer = store.read_startup()
    revision = store.load_revision(job['revision'])
    if (plan['base_revision'] != pointer['revision'] or plan['base_digest'] != pointer['generated_digest']
            or plan['desired_revision'] != manifest['desired_revision']
            or revision['meta']['generated_digest'] != value['digest']
            or store.get_job(value['job_id']) != job):
        raise StoreUnavailable('nav2_restart_revision_changed')
    return dict(value, revision=job['revision'], artifact=revision['generated']['navigation.yaml'])
