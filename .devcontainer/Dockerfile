FROM ros:humble-perception

# Install GUI and ROS simulation tools
RUN apt-get update && apt-get install -y \
    ros-humble-turtlesim \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rviz2 \
    xvfb fluxbox novnc websockify x11vnc \
    && rm -rf /var/lib/apt/lists/*

# Install noVNC (browser GUI)
RUN git clone https://github.com/novnc/noVNC.git /opt/noVNC

# Ensure DISPLAY=:0 always set automatically
RUN echo "export DISPLAY=:0" >> ~/.bashrc
