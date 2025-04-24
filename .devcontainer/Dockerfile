FROM ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:0

# Install GUI + Gazebo + ROS build + simulation tools
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-teleop-twist-keyboard \
    python3-colcon-common-extensions \
    git curl wget \
    xvfb fluxbox novnc websockify x11vnc \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p /root/ws/src
WORKDIR /root/ws/src

# Copy Hector packages from the host repo into the image
COPY hector_quadrotor /root/ws/src/hector_quadrotor
COPY hector_gazebo /root/ws/src/hector_gazebo

# Build the workspace
WORKDIR /root/ws
RUN . /opt/ros/humble/setup.sh && colcon build

# Install noVNC
RUN git clone https://github.com/novnc/noVNC.git /opt/noVNC

# VNC GUI launch script
RUN echo '#!/bin/bash\n\
export DISPLAY=:0\n\
Xvfb :0 -screen 0 1280x800x24 &\n\
sleep 2\n\
fluxbox &\n\
x11vnc -display :0 -forever -nopw -shared -rfbport 5900 &\n\
/opt/noVNC/utils/novnc_proxy --vnc localhost:5900 --listen 6080 &\n\
sleep 3\n\
bash' > /usr/local/bin/start-vnc.sh && chmod +x /usr/local/bin/start-vnc.sh

# Drone launch wrapper
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
source /root/ws/install/setup.bash\n\
ros2 launch hector_quadrotor_gazebo quadrotor_empty_world.launch.py' > /usr/local/bin/start-hector-drone.sh && chmod +x /usr/local/bin/start-hector-drone.sh

# Source ROS & workspace on container startup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/ws/install/setup.bash" >> ~/.bashrc && \
    echo "export DISPLAY=:0" >> ~/.bashrc
