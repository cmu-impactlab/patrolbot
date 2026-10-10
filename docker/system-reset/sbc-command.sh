#!/bin/sh
# Install as the forced command of a dedicated Pi reset SSH key on the SBC.
# No caller-controlled arguments, paths, environment or shell text are executed.
set -eu
export XDG_RUNTIME_DIR="/run/user/$(/usr/bin/id -u)"
case "${SSH_ORIGINAL_COMMAND:-}" in
    status) exec /bin/systemctl --user show patrolbot-server.service --property=LoadState ;;
    restart) exec /bin/systemctl --user restart patrolbot-server.service ;;
    *) echo 'Only reset service status and restart are permitted' >&2; exit 64 ;;
esac
