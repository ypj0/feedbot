#!/bin/bash
set -euo pipefail
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
docker=/opt/feedbot-nav2/docker/docker
exec "$docker" -H unix:///run/feedbot-nav2-docker.sock run --rm \
    --name feedbot-localization-verifier --stop-signal SIGINT --network host \
    -e ROS_DOMAIN_ID=89 -e ROS_LOCALHOST_ONLY=0 \
    -e ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e RMW_FASTRTPS_PUBLICATION_MODE=SYNCHRONOUS \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/work/agv_workspace/integration/mapping/config/dds_loopback.xml \
    -v /opt/feedbot-integration/releases/20261123-base-v7:/work:ro \
    -v "$script_dir:/observer:ro" --entrypoint /bin/bash feedbot-navigation:jazzy-20261122 \
    -c 'source /opt/ros/jazzy/setup.bash; exec python3 -B /observer/observer.py'
