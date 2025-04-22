#!/bin/bash
set -e

# Source the ROS 2 setup (this is essential for ROS2 commands to work)
source /opt/ros/humble/setup.bash

# Source the workspace setup if it exists
if [ -f "/ros2_ws/install/setup.bash" ]; then
  source /ros2_ws/install/setup.bash
fi

# Execute the command passed to docker run
exec "$@"