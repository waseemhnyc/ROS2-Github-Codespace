FROM ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:0
ENV TURTLEBOT3_MODEL=burger

# Install system dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-teleop-twist-keyboard \
    ros-humble-xacro \
    git wget curl \
    python3-colcon-common-extensions \
    python3-pip \
    xvfb fluxbox novnc websockify x11vnc \
    && rm -rf /var/lib/apt/lists/*

# Clone and build Hector Quadrotor packages
RUN mkdir -p /root/ws/src
WORKDIR /root/ws/src
RUN git clone -b humble-devel https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor.git \
    && git clone -b humble-devel https://github.com/tu-darmstadt-ros-pkg/hector_gazebo.git
WORKDIR /root/ws
RUN . /opt/ros/humble/setup.sh && colcon build

# Setup noVNC
RUN git clone https://github.com/novnc/noVNC.git /opt/noVNC

# Add startup script for GUI
RUN echo '#!/bin/bash\n\
export DISPLAY=:0\n\
Xvfb :0 -screen 0 1280x800x24 &\n\
sleep 2\n\
fluxbox &\n\
x11vnc -display :0 -forever -nopw -shared -rfbport 5900 &\n\
/opt/noVNC/utils/novnc_proxy --vnc localhost:5900 --listen 6080 &\n\
sleep 3\n\
bash' > /usr/local/bin/start-vnc.sh && chmod +x /usr/local/bin/start-vnc.sh

# Source ROS setup on container start
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/ws/install/setup.bash" >> ~/.bashrc
