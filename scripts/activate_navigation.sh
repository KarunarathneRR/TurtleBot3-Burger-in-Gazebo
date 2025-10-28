#!/bin/bash
# Activate Nav2 navigation stack after localization is ready
# Usage: run after Gazebo + localization (map_server + amcl) are up

set -e

source /opt/ros/jazzy/setup.bash
source ~/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash

# Ensure lifecycle manager exists
if ! ros2 node list | grep -q "/lifecycle_manager_navigation"; then
  echo "❌ lifecycle_manager_navigation not found. Start nav2_maze.launch.py first."
  exit 1
fi

# Wait for AMCL to publish pose (or timeout)
echo "⏳ Waiting for AMCL to publish /amcl_pose..."
if ! timeout 10 ros2 topic echo /amcl_pose --once >/dev/null 2>&1; then
  echo "⚠ AMCL pose not seen yet. Proceeding anyway."
else
  echo "✓ AMCL pose detected."
fi

# Start navigation lifecycle nodes
echo "▶️  Activating navigation nodes..."
ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 1}" || {
  echo "❌ Failed to call ManageLifecycleNodes start command"; exit 1;
}

# Quick check
sleep 2
if ros2 node list | grep -q "/controller_server" && ros2 topic list | grep -q "/local_costmap/costmap"; then
  echo "✅ Navigation nodes are active. You can send Nav2 goals from RViz."
else
  echo "⚠ Navigation may still be initializing. Check logs if goals fail."
fi
