# ROS2 Github Codespace

A ready to use development environment for ROS2 Humble development. This GitHub Codespace provides a pre-configured container with all necessary ROS2 tools and dependencies installed, allowing you to start developing ROS2 applications immediately without complex local setup.

[![Watch the walkthrough](https://img.youtube.com/vi/Y-eRpRxrpx4/maxresdefault.jpg)](https://youtu.be/Y-eRpRxrpx4)


## Getting Started

To start using this environment:

1. Navigate to the GitHub repository
2. Click on the "Code" button
3. Select the "Codespaces" tab
4. Click "Create codespace on main"

This will launch a cloud-based development environment with VS Code in your browser, complete with ROS2 Humble already installed and configured.

---

## How to View the Simulator (via VNC)

To view graphical simulations like `turtlesim` or Gazebo in your Codespace, follow these steps:

### 1. Start the VNC Server

Open a terminal in your Codespace and run:

```bash
/usr/local/bin/start-vnc.sh
```

### 2. Access the VNC Viewer
In the Codespace UI, go to the Ports tab.

Find the port mapped to 6080.

Click the 🌍 globe icon next to it to open the URL in your browser.

###  3. Connect to the Desktop
When the browser tab opens, select vnc.html

Click Connect (no password is required).

You should now see the graphical desktop where you can run tools like RViz or Gazebo.


## Launching Simulators

### Run turtlesim:

```bash
ros2 run turtlesim turtlesim_node
```

### Run TurtleBot3 in Gazebo:
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
Or try the house world:
```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

## Useful ROS2 Commands

### Publish a Twist Command:
```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 1.8}}"
```

### Keyboard Control for Turtlesim:
```bash
ros2 run turtlesim turtle_teleop_key
```

### List Active Nodes:
```bash
ros2 node list
```

### List Active Topics:
```bash
ros2 topic list
```
