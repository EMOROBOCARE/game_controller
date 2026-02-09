# Dockerfile for game_controller ROS2 package
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir pydantic pyyaml

# Create workspace
WORKDIR /ros2_ws/src

# Copy game_controller package
COPY . /ros2_ws/src/game_controller

# Copy generic_ui_interfaces for the UpdateManifest service
# This is expected to be in the build context (copied via docker-compose)
ARG GENERIC_UI_INTERFACES_PATH=generic_ui_interfaces
COPY ${GENERIC_UI_INTERFACES_PATH} /ros2_ws/src/generic_ui_interfaces

# Copy stub hri_actions_msgs for local builds/tests
ARG HRI_ACTIONS_MSGS_PATH=game_controller/stub_hri_actions_msgs
COPY ${HRI_ACTIONS_MSGS_PATH} /ros2_ws/src/hri_actions_msgs

# Avoid duplicate stub packages during colcon build
RUN rm -rf /ros2_ws/src/game_controller/test/stubs/generic_ui_interfaces

# Build workspace
WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --packages-select generic_ui_interfaces hri_actions_msgs game_controller

# Wrapper script for ros2 command
RUN printf '%s\n' \
    '#!/bin/bash' \
    'set -e' \
    'source /opt/ros/humble/setup.bash' \
    'source /ros2_ws/install/setup.bash' \
    'exec /opt/ros/humble/bin/ros2 "$@"' \
    > /usr/local/bin/ros2 && chmod +x /usr/local/bin/ros2

# Create entrypoint
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
source /ros2_ws/install/setup.bash\n\
exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "game_controller", "game_controller.launch.py"]
