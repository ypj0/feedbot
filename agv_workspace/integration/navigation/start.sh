#!/bin/bash
set -euo pipefail
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
bundle_root=$(realpath "$script_dir/../../..")
exec /opt/feedbot-nav2/docker/docker -H unix:///run/feedbot-nav2-docker.sock run --rm \
  --name feedbot-navigation --stop-signal SIGINT --network host --ipc host \
  -e ROS_DOMAIN_ID=89 -e ROS_LOCALHOST_ONLY=0 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e RMW_FASTRTPS_PUBLICATION_MODE=SYNCHRONOUS \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/work/agv_workspace/integration/mapping/config/dds_loopback.xml \
  -v "$bundle_root:/work:ro" \
  -v /opt/feedbot-maps/20261124_edited_e33245ba:/maps:ro \
  --entrypoint /bin/bash feedbot-navigation:jazzy-20261122 -c \
  'source /opt/ros/jazzy/setup.bash; exec ros2 launch /work/agv_workspace/integration/navigation/navigation.launch.py params_file:=/work/agv_workspace/integration/navigation/config/nav2.yaml map:=/maps/feedbot.yaml use_sim_time:=false'
