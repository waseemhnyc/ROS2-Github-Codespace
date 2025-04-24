FROM ros:humble-perception

# Install all required packages
RUN apt-get update && apt-get install -y \
    ros-humble-turtlesim \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rviz2 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    xvfb fluxbox novnc websockify x11vnc \
    python3-colcon-common-extensions \
    git curl && \
    rm -rf /var/lib/apt/lists/*

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

# Create workspace and clone sjtu_drone
RUN mkdir -p /root/drone_ws/src
WORKDIR /root/drone_ws/src
RUN git clone https://github.com/NovoG93/sjtu_drone.git

# Build the workspace
WORKDIR /root/drone_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Create drone simulator launch script
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
source /root/drone_ws/install/setup.bash\n\
\n\
# Start the drone simulator\n\
ros2 launch sjtu_drone_bringup sjtu_drone_gazebo.launch.py' > /usr/local/bin/run_sim.sh \
&& chmod +x /usr/local/bin/run_sim.sh

# Persistent environment setup
RUN echo "export DISPLAY=:0" >> ~/.bashrc
RUN echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /root/drone_ws/install/setup.bash" >> ~/.bashrc

# Create script to spawn obstacles in turtlesim
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
\n\
# Spawn multiple obstacles at different positions\n\
ros2 service call /spawn turtlesim/srv/Spawn "{x: 3.0, y: 3.0, theta: 0.0, name: \"obstacle1\"}"\n\
ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.0, y: 5.0, theta: 0.0, name: \"obstacle2\"}"\n\
ros2 service call /spawn turtlesim/srv/Spawn "{x: 7.0, y: 3.0, theta: 0.0, name: \"obstacle3\"}"\n\
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 7.0, theta: 0.0, name: \"obstacle4\"}"' > /usr/local/bin/spawn_obstacles.sh \
&& chmod +x /usr/local/bin/spawn_obstacles.sh
