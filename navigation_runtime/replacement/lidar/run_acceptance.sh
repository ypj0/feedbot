#!/bin/bash
set -eo pipefail
root=/opt/feedbot-nav2/lidar
/opt/feedbot-nav2/docker/docker -H unix:///run/feedbot-nav2-docker.sock run --rm --network=none --cpus=1 --memory=1200m --cap-drop=ALL --security-opt=no-new-privileges -e ROS_DOMAIN_ID=91 -v "$root:/src" feedbot-nav2-onboard:jazzy bash -c '
set -e
source /opt/ros/jazzy/setup.bash
cmake -S /src -B /src/build -DLDCP_SOURCE=/src/vendor/ldcp_sdk -DCMAKE_BUILD_TYPE=Release
cmake --build /src/build -j1
ctest --test-dir /src/build --output-on-failure
python3 /src/test_supervisor.py
python3 /src/sdk_fixture.py /src/build/ltr2_sdk_fixture
python3 /src/managed_fixture.py /src/passive.bin /src/build/ltr2_driver
'
