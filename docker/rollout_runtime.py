"""Coordinate file rollback with container rollback through checked adapters.

Adapters must verify the pinned candidate, capture fresh read-only telemetry,
recreate only the manifest's services, and check image/source/health readback.
This module does not authorize deployment or send robot commands.
"""

from rollout_files import digest, install


class RollbackFailed(RuntimeError):
    def __init__(self, deployment_error, rollback_error):
        super().__init__('deployment failed and runtime rollback could not be verified')
        self.deployment_error = deployment_error
        self.rollback_error = rollback_error


def require_files(root, changes, image, hash_key):
    for change in changes:
        if digest(root / change['path']) != change[hash_key]:
            raise RuntimeError('source verification failed: ' + change['path'])
    image_lines = [line for line in (root / 'docker/.env').read_text().splitlines()
                   if line.startswith('PATROLBOT_IMAGE=')]
    if image_lines != ['PATROLBOT_IMAGE=' + image]:
        raise RuntimeError('environment image verification failed')


def rollout(root, payload, backup, manifest, *, verify_candidate, preflight,
            restart, postflight):
    """Apply an already-authorized rollout; failures remain failures after rollback.

    verify_candidate(manifest) and postflight(image) must return exactly True.
    preflight() must return a capture dictionary with safe exactly True.
    restart(image) must raise on any command failure. It is called again for
    the previous image if the candidate restart was attempted, even partially.
    All adapters are required; there are no permissive defaults.
    """
    old = manifest['expected_previous_image']
    new = manifest['candidate_image']
    changes = manifest['source_changes']
    if verify_candidate(manifest) is not True:
        raise RuntimeError('candidate verification refused deployment')
    capture = preflight()
    if not isinstance(capture, dict) or capture.get('safe') is not True:
        raise RuntimeError('stationary preflight refused deployment')

    restart_attempted = False

    def verify_installation():
        nonlocal restart_attempted
        require_files(root, changes, new, 'after')
        restart_attempted = True
        restart(new)
        if postflight(new) is not True:
            raise RuntimeError('candidate postflight failed')

    try:
        install(root, payload, backup, changes, old, new, verify_installation)
    except BaseException as deployment_error:
        # install restores the complete file snapshot before raising. If its
        # restoration failed, refuse to recreate containers from mixed files.
        if restart_attempted:
            try:
                require_files(root, changes, old, 'before')
                restart(old)
                if postflight(old) is not True:
                    raise RuntimeError('previous-image postflight failed')
            except BaseException as rollback_error:
                raise RollbackFailed(deployment_error, rollback_error) from deployment_error
        raise
    return {'image': new, 'backup': str(backup), 'preflight': capture}
