#!/bin/bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
set -u
cd /probe
test "$ROS_DISTRO" = jazzy
test "$RMW_IMPLEMENTATION" = rmw_fastrtps_cpp
test -c /dev/rikibase
test -c /dev/rikiimu
stat -c '%n %t:%T' /dev/rikibase /dev/rikiimu > /results/devices.txt
dpkg-query -W ros-jazzy-controller-manager ros-jazzy-hardware-interface \
    ros-jazzy-diff-drive-controller ros-jazzy-robot-localization \
    ros-jazzy-nav2-smac-planner ros-jazzy-nav2-mppi-controller \
    ros-jazzy-rmw-fastrtps-cpp > /results/packages.txt
python3 peer.py talker /results/talker.json > /results/talker.log 2>&1 &
talker_pid=$!
python3 peer.py listener /results/listener_first.json > /results/listener_first.log 2>&1 &
listener_pid=$!
sleep 3
ros2 node list --no-daemon --spin-time 2 > /results/nodes_first.txt
timeout 5 ros2 topic echo /environment_probe/reply std_msgs/msg/String --once > /results/echo_first.txt
wait "$listener_pid"
python3 peer.py listener /results/listener_restarted.json > /results/listener_restarted.log 2>&1 &
listener_pid=$!
sleep 3
ros2 node list --no-daemon --spin-time 2 > /results/nodes_restarted.txt
timeout 5 ros2 topic echo /environment_probe/reply std_msgs/msg/String --once > /results/echo_restarted.txt
wait "$listener_pid"
wait "$talker_pid"
grep -Fx /environment_talker /results/nodes_first.txt
grep -Fx /environment_listener /results/nodes_first.txt
grep -Fx /environment_talker /results/nodes_restarted.txt
grep -Fx /environment_listener /results/nodes_restarted.txt
grep '^data:' /results/echo_first.txt
grep '^data:' /results/echo_restarted.txt
python3 - <<'PY'
import json
from pathlib import Path

results = Path('/results')
talker = json.loads((results / 'talker.json').read_text())
listeners = [json.loads((results / filename).read_text()) for filename in
             ('listener_first.json', 'listener_restarted.json')]
assert len({listener['pid'] for listener in listeners}) == 2
for peer in [talker] + listeners:
    assert peer['rmw'] == 'rmw_fastrtps_cpp', peer
    assert peer['received'] >= 20 and peer['sent'] >= 20, peer
for listener in listeners:
    assert talker['responder_counts'][str(listener['pid'])] >= 20, talker
summary = {'dds_bidirectional': True, 'listener_restart': True,
           'cli_discovery_and_echo': True, 'device_nodes_present': True,
           'rmw': talker['rmw'], 'hardware_io_performed': False,
           'talker': talker, 'listeners': listeners}
(results / 'communication.json').write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps(summary, indent=2))
PY
