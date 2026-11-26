#!/bin/bash
set -e
docker=/opt/feedbot-nav2/docker/docker
socket=unix:///run/feedbot-nav2-docker.sock
result=/var/tmp/run20261122_imu_baseline_01

"$docker" -H "$socket" run -d --rm --name feedbot-imu-baseline-01-observer \
    --network host --pid host \
    -e ROS_DOMAIN_ID=89 -e ROS_LOCALHOST_ONLY=1 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -v /opt/feedbot-integration/releases/20261122-imu-baseline-01:/work:ro \
    -v "$result:/results:rw" --entrypoint /bin/bash feedbot-navigation:jazzy-20261122 \
    -lc "source /opt/ros/jazzy/setup.bash; exec python3 -B /work/agv_workspace/integration/ekf/collect_imu_baseline.py /results/raw.jsonl 45" \
    > "$result/observer.container_id"

ready=0
checks=0
while [ "$checks" -lt 50 ]; do
    if grep -q '"kind": "load"' "$result/raw.jsonl" 2>/dev/null; then
        ready=1
        break
    fi
    if ! "$docker" -H "$socket" inspect feedbot-imu-baseline-01-observer >/dev/null 2>&1; then
        break
    fi
    checks=$((checks + 1))
    sleep 0.1
done
echo "observer_ready=$ready observer_checks=$checks" | tee "$result/observer.readiness"
[ "$ready" -eq 1 ]

awk '{print "{\"event\":\"imu_start_command\",\"monotonic\":" $1 ",\"epoch\":" systime() "}"}' \
    /proc/uptime | tee -a "$result/events.jsonl"
systemctl start feedbot-imu.service
systemctl show feedbot-imu.service -p ActiveState -p SubState -p MainPID \
    -p ExecMainStartTimestampMonotonic -p ActiveEnterTimestampMonotonic \
    | tee "$result/feedbot-imu.start.show"
