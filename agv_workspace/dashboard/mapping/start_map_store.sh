#!/bin/bash
set -eu
root=/opt/feedbot-integration/releases/20261122-mapping-b-v1
service=/opt/feedbot-map-store
mkdir -p /opt/feedbot-maps
exec /opt/feedbot-nav2/docker/docker -H unix:///run/feedbot-nav2-docker.sock run -d \
  --name feedbot-map-store --network host --ipc host \
  -e ROS_DOMAIN_ID=89 -e ROS_LOCALHOST_ONLY=0 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e RMW_FASTRTPS_PUBLICATION_MODE=SYNCHRONOUS \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/work/agv_workspace/integration/mapping/config/dds_loopback.xml \
  -e FEEDBOT_TEST_TOKEN_FILE=/key/token \
  -v "$root:/work:ro" -v "$service:/service:ro" \
  -v /opt/feedbot-maps:/maps:rw -v /run/feedbot-chassis-web:/key:ro \
  --entrypoint /bin/bash feedbot-navigation:jazzy-20261122 \
  -c 'source /opt/ros/jazzy/setup.bash; exec python3 -B /service/map_store.py'
