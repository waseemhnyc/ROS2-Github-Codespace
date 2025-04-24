FROM ros:humble-perception

# Set environment variables for ROS 2 and display
ENV DISPLAY=:0
ENV TURTLEBOT3_MODEL=burger

# Install GUI and simulation tools
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rviz2 \
    ros-humble-xacro \
    ros-humble-hector-gazebo \
    ros-humble-hector-quadrotor-demo \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-teleop-twist-keyboard \
    xvfb fluxbox novnc websockify x11vnc \
    && rm -rf /var/lib/apt/lists/*

# Optional: Install turtlesim and turtlebot3 for additional testing
RUN apt-get update && apt-get install -y \
    ros-humble-turtlesim \
    ros-humble-turtlebot3-gazebo \
    && rm -rf /var/lib/apt/lists/*

# Install noVNC
RUN git clone https://github.com/novnc/noVNC.git /opt/noVNC

# Create startup script to launch VNC environment
RUN echo '#!/bin/bash\n\
export DISPLAY=:0\n\
Xvfb :0 -screen 0 1280x800x24 &\n\
sleep 2\n\
fluxbox &\n\
x11vnc -display :0 -forever -nopw -shared -rfbport 5900 &\n\
/opt/noVNC/utils/novnc_proxy --vnc localhost:5900 --listen 6080 &\n\
sleep 3\n\
bash' > /usr/local/bin/start-vnc.sh \
&& chmod +x /usr/local/bin/start-vnc.sh

# Set ROS 2 setup in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
