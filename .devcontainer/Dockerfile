FROM ros:humble-perception

# Install GUI and simulation tools
RUN apt-get update && apt-get install -y \
    ros-humble-turtlesim \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rviz2 \
    xvfb fluxbox novnc websockify x11vnc \
    && rm -rf /var/lib/apt/lists/*

# Install noVNC
RUN git clone https://github.com/novnc/noVNC.git /opt/noVNC

# Create a directory for startup scripts
RUN mkdir -p /etc/container/startup.d

# Startup script to launch graphical environment
COPY ensure-vnc.sh /usr/local/bin/ensure-vnc.sh
RUN chmod +x /usr/local/bin/ensure-vnc.sh

# Create a script to be run at container startup
RUN echo '#!/bin/bash\n\
# Start VNC service\n\
/usr/local/bin/ensure-vnc.sh &\n\
\n\
# Export display for GUI applications\n\
export DISPLAY=:0\n\
\n\
# Execute the container entrypoint\n\
exec "$@"' > /usr/local/bin/docker-entrypoint.sh \
&& chmod +x /usr/local/bin/docker-entrypoint.sh

# Ensure DISPLAY is always set correctly
RUN echo "export DISPLAY=:0" >> /etc/bash.bashrc

ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
CMD ["bash"]
