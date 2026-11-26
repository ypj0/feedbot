#!/bin/bash
set -euo pipefail

usage() {
    echo "usage: $0 mapping" >&2
    echo "       $0 localization MAP_YAML" >&2
    exit 2
}

[[ $# -ge 1 ]] || usage
mode=$1
shift
map_mount=()
map_argument=()
if [[ "$mode" == "mapping" ]]; then
    [[ $# -eq 0 ]] || usage
    launch_file=mapping.launch.py
elif [[ "$mode" == "localization" ]]; then
    [[ $# -eq 1 ]] || usage
    [[ -f "$1" ]] || { echo "map YAML does not exist: $1" >&2; exit 2; }
    map_yaml=$(realpath "$1")
    map_mount=(-v "$(dirname "$map_yaml"):/maps:ro")
    map_argument=("map_yaml:=/maps/$(basename "$map_yaml")")
    launch_file=localization.launch.py
else
    usage
fi

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
bundle_root=$(realpath "$script_dir/../../..")
docker=/opt/feedbot-nav2/docker/docker

[[ $("$docker" -H unix:///run/feedbot-nav2-docker.sock inspect \
    --format '{{.State.Running}}' feedbot-base) == true ]] || {
    echo "feedbot-base is not running; start the independent base session first" >&2
    exit 1
}

exec "$docker" -H unix:///run/feedbot-nav2-docker.sock run --rm \
    --name feedbot-mapping-mode --stop-signal SIGINT --network host --ipc host \
    -e ROS_DOMAIN_ID=89 -e ROS_LOCALHOST_ONLY=0 \
    -e ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e RMW_FASTRTPS_PUBLICATION_MODE=SYNCHRONOUS \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/work/agv_workspace/integration/mapping/config/dds_loopback.xml \
    -v "$bundle_root:/work:ro" "${map_mount[@]}" \
    --entrypoint /bin/bash feedbot-navigation:jazzy-20261122 -c \
    'source /opt/ros/jazzy/setup.bash; exec ros2 launch "$@"' bash \
    "/work/agv_workspace/integration/mapping/$launch_file" \
    "${map_argument[@]}"
