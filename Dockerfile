  # Use the official ROS 2 Jazzy Jalisco base image
  FROM ros:jazzy-desktop

  # Install necessary packages
  RUN apt-get update && apt-get install -y \
      ros-jazzy-turtlesim \
      ros-jazzy-gazebo-ros-pkgs \
      python3-colcon-common-extensions \
      xvfb \
      x11vnc \
      fluxbox \
      novnc \
      websockify \
      && rm -rf /var/lib/apt/lists/*

  # Set up noVNC
  RUN git clone https://github.com/novnc/noVNC.git /opt/noVNC \
      && git clone https://github.com/novnc/websockify /opt/noVNC/utils/websockify \
      && ln -s /opt/noVNC/vnc.html /opt/noVNC/index.html

  # Copy the startup script
  COPY start-vnc.sh /usr/local/bin/start-vnc.sh
  RUN chmod +x /usr/local/bin/start-vnc.sh

  # Set the entrypoint
  ENTRYPOINT ["/usr/local/bin/start-vnc.sh"]
