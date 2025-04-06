FROM ros:jazzy

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Create workspace for micro-ROS
WORKDIR /uros_ws

# Clone micro-ROS Agent
RUN git clone https://github.com/micro-ROS/micro-ROS-Agent.git src/micro-ros-agent && \
    cd src/micro-ros-agent && \
    git checkout $ROS_DISTRO || git checkout humble

# Install dependencies using rosdep
RUN rosdep init || echo "rosdep already initialized" && \
    rosdep update && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    sudo apt-get update && \
    rosdep install --from-paths src --ignore-src -y

# Build micro-ROS Agent
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --packages-select micro_ros_agent

# Set up environment
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
    echo "source /uros_ws/install/setup.bash" >> ~/.bashrc

# Set default serial device (can be overridden at runtime)
ENV SERIAL_DEVICE=/dev/ttyACM0

# Run micro-ROS Agent with serial communication
# Create entrypoint script
RUN echo '#!/bin/bash\n\
source /opt/ros/'$ROS_DISTRO'/setup.bash\n\
source /uros_ws/install/setup.bash\n\
exec ros2 run micro_ros_agent micro_ros_agent "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["--dev", "/dev/ttyACM0", "-v6"]
