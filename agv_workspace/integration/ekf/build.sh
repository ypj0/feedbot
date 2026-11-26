#!/bin/bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
set -u
for package in chassis imu; do
    cmake -S "/work/agv_workspace/drivers/${package}_ros2" -B "/work/build/${package}" \
        -DCHASSIS_CORE=/work/navigation_runtime/replacement/chassis \
        -DCMAKE_INSTALL_PREFIX=/work/install -DCMAKE_BUILD_TYPE=Debug
    cmake --build "/work/build/${package}" -j2
    ctest --test-dir "/work/build/${package}" --output-on-failure
    cmake --install "/work/build/${package}"
done
cmake -S /work/agv_workspace/integration/ekf -B /work/build/tf_probe \
    -DCMAKE_INSTALL_PREFIX=/work/install -DCMAKE_BUILD_TYPE=Debug
cmake --build /work/build/tf_probe -j2
cmake --install /work/build/tf_probe
