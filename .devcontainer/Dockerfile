FROM ros:humble-perception

# Install GUI and simulation tools
RUN apt-get update && apt-get install -y \
    ros-humble-turtlesim \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rviz2 \
    ros-humble-turtlebot3-gazebo \
    xvfb fluxbox novnc websockify x11vnc \
    && rm -rf /var/lib/apt/lists/*

# Install noVNC
RUN git clone https://github.com/novnc/noVNC.git /opt/noVNC

# Startup script to launch graphical environment
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

# Ensure DISPLAY is always set correctly
RUN echo "export DISPLAY=:0" >> ~/.bashrc
RUN echo "export TURTLEBOT3_MODEL=burger " >> ~/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
