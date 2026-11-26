#!/bin/bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
set -u

owner_token=$1
pids=()
names=()

start_process() {
    names+=("$1")
    shift
    "$@" &
    pids+=("$!")
}

shutdown_stack() {
    trap - TERM INT EXIT
    status=$1
    for ((index=${#pids[@]}-1; index>=0; index--)); do
        pid=${pids[$index]}
        if kill -0 "$pid" 2>/dev/null && ! kill -TERM "$pid"; then
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
start_process base /bin/bash /work/agv_workspace/integration/bringup/stack.sh "$owner_token"
start_process mapping ros2 launch /work/agv_workspace/integration/mapping/mapping.launch.py

trap - EXIT
if wait -n "${pids[@]}"; then
    echo "stack process exited unexpectedly with status 0" >&2
    shutdown_stack 1
else
    code=$?
    echo "stack process exited unexpectedly with status $code" >&2
    shutdown_stack "$code"
fi
