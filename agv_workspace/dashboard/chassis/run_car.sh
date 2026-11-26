#!/bin/bash
set -em
source /opt/ros/jazzy/setup.bash
source /work/install/share/feedbot_chassis_ros2/local_setup.bash
ros2 launch feedbot_chassis_ros2 chassis.launch.py covariance_file:="${1:?covariance YAML path required}" &
controller_pid=$!
python3 -B /web/car_bridge.py &
bridge_pid=$!
shutdown() {
    trap - TERM INT EXIT
    kill -TERM "$bridge_pid" 2>/dev/null || true
    wait "$bridge_pid" || true
    kill -INT "$controller_pid" 2>/dev/null || true
    wait "$controller_pid" || true
}
trap 'exit 0' TERM INT
trap shutdown EXIT
set +e
wait -n "$controller_pid" "$bridge_pid"
result=$?
exit "$result"
