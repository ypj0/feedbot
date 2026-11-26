#!/bin/bash
set -euo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
exec systemd-run --collect --unit=feedbot-map-layer --property=Type=simple --property=Restart=no \
    --property=BindsTo=feedbot-base.service --property=After=feedbot-base.service \
    --property="WorkingDirectory=$PWD" --property=TimeoutStopSec=40 \
    --property="ExecStop=/opt/feedbot-nav2/docker/docker -H unix:///run/feedbot-nav2-docker.sock stop -t 20 feedbot-mapping-mode" \
    /bin/bash "$script_dir/run.sh" "$@"
