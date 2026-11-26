#!/bin/bash
set -eo pipefail
D=(/opt/feedbot-nav2/docker/docker -H unix:///run/feedbot-nav2-docker.sock)
case "$1" in
start)
 [[ "$2" =~ ^[0-9a-f]{48}$ ]]
 "${D[@]}" run -d --name feedbot-native-lidar --network=host --cpus=1 --memory=800m --pids-limit=128 --cap-drop=ALL --security-opt=no-new-privileges --read-only --tmpfs /tmp:rw --tmpfs /root/.ros:rw -e ROS_DOMAIN_ID=89 -e ROS_LOCALHOST_ONLY=1 -v /opt/feedbot-nav2/lidar:/src:ro -v /run/feedbot-lidar:/run/feedbot-lidar feedbot-nav2-onboard:jazzy bash -c 'source /opt/ros/jazzy/setup.bash; exec /src/build/ltr2_driver --ros-args -p owner_token:="$1"' _ "$2"
 ;;
stop)
 if "${D[@]}" inspect feedbot-native-lidar >/dev/null 2>&1; then
  "${D[@]}" logs --timestamps feedbot-native-lidar > /run/feedbot-lidar/native-last.log 2>&1 || true
  "${D[@]}" stop --time 3 feedbot-native-lidar >/dev/null
  "${D[@]}" rm -f feedbot-native-lidar >/dev/null
 fi
 ;;
transition)
 "${D[@]}" exec feedbot-native-lidar bash -c 'source /opt/ros/jazzy/setup.bash; python3 /src/lifecycle_command.py "$1"' _ "$2"
 ;;
*) exit 2;;
esac
