#!/bin/bash
set -eo pipefail
root=/opt/feedbot-nav2/lidar
run=$(mktemp -d "$root/live.XXXXXX")
docker=(/opt/feedbot-nav2/docker/docker -H unix:///run/feedbot-nav2-docker.sock)
name=feedbot-lidar-$(basename "$run" | tr . -)
trap '"${docker[@]}" rm -f "$name" >/dev/null 2>&1 || true' EXIT
systemctl show spirit.service -p MainPID -p ActiveState > "$run/factory_before.txt"
"${docker[@]}" run --rm --name "$name" --network=host --cpus=1 --memory=800m --pids-limit=128 --cap-drop=ALL --cap-add=NET_RAW --security-opt=no-new-privileges --read-only --tmpfs /tmp:rw --tmpfs /root/.ros:rw -e ROS_DOMAIN_ID=89 -e ROS_LOCALHOST_ONLY=1 -v "$root:/src:ro" -v "$run:/results" feedbot-nav2-onboard:jazzy bash -c '
set -e
source /opt/ros/jazzy/setup.bash
python3 /src/observe_native.py > /results/observer.log 2>&1 &
pid=$!
/src/build/ltr2_node --ros-args -p passive_interface:=enp2s0 -p duration:=15.0 > /results/node.log 2>&1
wait "$pid"
'
systemctl show spirit.service -p MainPID -p ActiveState > "$run/factory_after.txt"
printf '%s\n' "$run"
