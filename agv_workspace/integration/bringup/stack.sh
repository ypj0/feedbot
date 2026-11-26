#!/bin/bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /work/install/share/feedbot_chassis_ros2/local_setup.bash
source /work/install/share/feedbot_imu_ros2/local_setup.bash
set -u

owner_token=$1
pids=()
names=()

start_process() {
    names+=("$1")
    shift
    env --default-signal=INT "$@" &
    pids+=("$!")
}

shutdown_stack() {
    trap - TERM INT EXIT
    status=$1
    for ((index=${#pids[@]}-1; index>=0; index--)); do
        pid=${pids[$index]}
        if kill -0 "$pid" 2>/dev/null && ! kill -INT "$pid"; then
            echo "failed to signal ${names[$index]} pid $pid" >&2
            status=1
        fi
    done
    for ((index=${#pids[@]}-1; index>=0; index--)); do
        if wait "${pids[$index]}"; then
            continue
        else
            code=$?
        fi
        if [[ $code -ne 143 && $code -ne 130 ]]; then
            echo "${names[$index]} exited with $code during shutdown" >&2
            status=1
        fi
    done
    exit "$status"
}

trap 'shutdown_stack 0' TERM INT EXIT
start_process chassis ros2 launch feedbot_chassis_ros2 chassis.launch.py \
    covariance_file:=/work/agv_workspace/integration/ekf/config/wheel_covariance.yaml
start_process imu /work/install/lib/feedbot_imu_ros2/imu_node
start_process ekf ros2 launch /work/agv_workspace/integration/ekf/ekf.launch.py
start_process lidar /ltr2/build/ltr2_ros2 --ros-args \
    -p owner_token:="$owner_token" -p owner_directory:=/run/feedbot-lidar

trap - EXIT
if wait -n "${pids[@]}"; then
    echo "stack process exited unexpectedly with status 0" >&2
    shutdown_stack 1
else
    code=$?
    echo "stack process exited unexpectedly with status $code" >&2
    shutdown_stack "$code"
fi
