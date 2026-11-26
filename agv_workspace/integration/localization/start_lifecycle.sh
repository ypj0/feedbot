#!/bin/bash
set -euo pipefail
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
exec systemd-run --collect --unit=feedbot-localization-lifecycle --property=Restart=no \
    --property="BindsTo=feedbot-base.service feedbot-map-layer.service feedbot-localization-verifier.service" \
    --property="After=feedbot-base.service feedbot-map-layer.service feedbot-localization-verifier.service" \
    --property=TimeoutStopSec=25 \
    --property="ExecStop=/opt/feedbot-nav2/docker/docker -H unix:///run/feedbot-nav2-docker.sock stop -t 15 feedbot-localization-lifecycle" \
    /bin/bash "$script_dir/run_lifecycle.sh"
