FROM ros:humble-perception

# Install GUI and simulation tools
RUN apt-get update && apt-get install -y \
    ros-humble-turtlesim \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rviz2 \
    ros-humble-turtlebot3-gazebo \
    xvfb fluxbox novnc websockify x11vnc \
    git \
    python3-rosdep \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install noVNC
RUN git clone https://github.com/novnc/noVNC.git /opt/noVNC

# Install Hector Quadrotor dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-gazebo-plugins \
    ros-humble-teleop-twist-keyboard \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    libgoogle-glog-dev \
    && rm -rf /var/lib/apt/lists/*

# Create workspace for Hector Quadrotor
RUN mkdir -p /root/hector_ws/src
WORKDIR /root/hector_ws/src

# Clone Hector Quadrotor repos
RUN git clone https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor.git && \
    git clone https://github.com/tu-darmstadt-ros-pkg/hector_localization.git && \
    git clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo.git && \
    git clone https://github.com/tu-darmstadt-ros-pkg/hector_models.git && \
    git clone https://github.com/tu-darmstadt-ros-pkg/hector_msgs.git

# Build Hector workspace
WORKDIR /root/hector_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install"

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
RUN echo "source /root/hector_ws/install/setup.bash" >> ~/.bashrc
