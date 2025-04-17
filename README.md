# ROS2 Github Codespace

A ready to use development environment for ROS2 Humble development. This GitHub Codespace provides a pre-configured container with all necessary ROS2 tools and dependencies installed, allowing you to start developing ROS2 applications immediately without complex local setup.

## Getting Started

To start using this environment:

1. Navigate to the GitHub repository
2. Click on the "Code" button
3. Select the "Codespaces" tab
4. Click "Create codespace on main"

This will launch a cloud-based development environment with VS Code in your browser, complete with ROS2 Humble already installed and configured.

## Features

- Full ROS2 Humble Hawksbill installation
- Common development tools pre-installed
- VS Code extensions for ROS2 development
- Terminal with ROS2 environment variables pre-configured
- Support for visualization tools and simulation

## Using ROS2 in the Codespace

Once your codespace is running, you can:

- Create and build ROS2 packages using `colcon`
- Run ROS2 nodes and launch files
- Visualize data using RViz (via port forwarding)
- Use simulation tools like Gazebo

### Basic Commands

```bash
# Source the ROS2 environment
source /opt/ros/humble/setup.bash

# Create a new workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build packages
colcon build

# Source workspace
source install/setup.bash

# Run a ROS2 node
ros2 run <package_name> <node_name>

# Launch a ROS2 application
ros2 launch <package_name> <launch_file>
```

## Customization

This codespace can be customized to fit your specific needs. The configuration is defined in the `.devcontainer` directory.

## Troubleshooting

If you encounter any issues with the codespace:

1. Try rebuilding the codespace from the GitHub interface
2. Check that your ROS2 environment is properly sourced
3. Verify port forwarding settings for visualization tools

## Contributing

Contributions to improve this development environment are welcome! Please submit pull requests or open issues to suggest enhancements.

## START HERE
In one terminal:
you want to run the following command:
```
/usr/local/bin/start-vnc.sh
```

Then you want to go to ports - find the port that is pointed to port 6080. Look for the little earth icon and select it to open the URL.

When you are on that URL in your browser - select `vnc.html`

Select `Connect`.

Then you can go back to your code space.

In another terminal run:

`ros2 run turtlesim turtlesim_node`


`ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 1.8}}"`


`ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`


`ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py`

