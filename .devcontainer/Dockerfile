FROM ros:humble-perception

# Install essential GUI packages, Gazebo, and Turtlesim
RUN apt-get update && apt-get install -y \
    ros-humble-turtlesim \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rviz2 \
    xvfb fluxbox novnc websockify x11vnc \
    && rm -rf /var/lib/apt/lists/*

# Install noVNC for browser-based GUI
RUN git clone https://github.com/novnc/noVNC.git /opt/noVNC

# Create startup script directly
RUN echo '#!/bin/bash\n\
export DISPLAY=:0\n\
Xvfb :0 -screen 0 1280x800x24 &\n\
sleep 2\n\
fluxbox &\n\
x11vnc -display :0 -forever -nopw -shared -rfbport 5900 &\n\
/opt/noVNC/utils/launch.sh --vnc localhost:5900 --listen 6080 &' > /usr/local/bin/start-vnc.sh \
&& chmod +x /usr/local/bin/start-vnc.sh

ENTRYPOINT ["/usr/local/bin/start-vnc.sh"]
