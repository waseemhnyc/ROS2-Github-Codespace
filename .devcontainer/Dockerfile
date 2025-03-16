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

# Startup script to launch graphical environment
COPY ensure-vnc.sh /usr/local/bin/ensure-vnc.sh
RUN chmod +x /usr/local/bin/ensure-vnc.sh

# Create a script for GitHub Codespaces to run at startup
RUN echo '#!/bin/bash\n\
\n\
# Start VNC service in the background\n\
nohup /usr/local/bin/ensure-vnc.sh > /tmp/vnc-nohup.log 2>&1 &\n\
\n\
# Export display for GUI applications\n\
export DISPLAY=:0\n\
\n\
# Execute the passed command\n\
exec "$@"' > /usr/local/bin/entrypoint.sh \
&& chmod +x /usr/local/bin/entrypoint.sh

# Ensure DISPLAY is always set correctly
RUN echo "export DISPLAY=:0" >> /etc/bash.bashrc

# Create a startup script for post-creation environment
RUN echo '#!/bin/bash\n\
# Check if VNC is running, restart if needed\n\
if ! pgrep -f Xvfb > /dev/null; then\n\
  echo "VNC not running, starting it..."\n\
  /usr/local/bin/ensure-vnc.sh\n\
fi\n\
echo "VNC should be available at http://localhost:6080/"' > /usr/local/bin/check-vnc.sh \
&& chmod +x /usr/local/bin/check-vnc.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
