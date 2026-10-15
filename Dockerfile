FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Update and install simulation dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3 \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-smac-planner \
    ros-jazzy-navigation2 \
    ros-jazzy-slam-toolbox \
    ros-jazzy-xacro \
    ros-jazzy-ros-gz \
    ros-jazzy-robot-state-publisher \
    ruby \
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# For Python dev inside container
RUN python3 -m pip install --no-cache-dir \
    scipy \
    fastapi \
    uvicorn \
    websockets \
    scikit-image \
    matplotlib \
    psutil \
    shapely \
    --break-system-packages

# Setup Workspace
WORKDIR /opt/robot_ws

# Copy source code
COPY ros2/src /opt/robot_ws/src
COPY frontend /opt/frontend

# Build overlay workspace
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install && \
    echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source ${ROS_WS}/install/setup.bash" >> /root/.bashrc

# Entrypoint that sources ROS
COPY utils/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
