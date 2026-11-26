#!/bin/bash
set -euo pipefail
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
exec systemd-run --collect --unit=feedbot-localization-verifier --property=Restart=no \
    --property="BindsTo=feedbot-base.service feedbot-map-layer.service" \
    --property="After=feedbot-base.service feedbot-map-layer.service" \
    --property=TimeoutStopSec=25 \
    --property="ExecStop=/opt/feedbot-nav2/docker/docker -H unix:///run/feedbot-nav2-docker.sock stop -t 15 feedbot-localization-verifier" \
    /bin/bash "$script_dir/run.sh"
