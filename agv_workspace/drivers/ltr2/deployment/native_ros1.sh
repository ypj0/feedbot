#!/bin/bash
set -eo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd)"
unit=feedbot-ltr2-managed.service
case "${1:-}" in
start)
    [[ "${2:-}" =~ ^[0-9a-f]{48}$ ]]
    test -x "$root/build/ltr2_managed_ros1"
    systemd-run --unit="$unit" --property=Restart=no --property=KillMode=control-group \
        --property=TimeoutStopSec=10 --property=NoNewPrivileges=yes \
        /bin/bash -c 'source /opt/ros/melodic/setup.bash; exec "$1" "_owner_token:=$2"' \
        _ "$root/build/ltr2_managed_ros1" "$2"
    ;;
stop)
    if systemctl show "$unit" -p LoadState | grep -q '^LoadState=not-found$'; then
        exit 0
    fi
    systemctl stop "$unit"
    state="$(systemctl show "$unit" -p ActiveState --value)"
    [[ "$state" == inactive || "$state" == failed ]]
    ;;
transition)
    source /opt/ros/melodic/setup.bash
    timeout 15 /usr/bin/python2 "$root/source/deployment/transition_ros1.py" "${2:-}"
    ;;
*) exit 2 ;;
esac
