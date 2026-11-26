FROM feedbot-nav2-onboard:jazzy

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-jazzy-controller-manager ros-jazzy-ros2controlcli \
    && rm -rf /var/lib/apt/lists/*
