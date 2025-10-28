#!/bin/bash

# Complete startup and localization fix - all in one
# Run this after Gazebo and Navigation are running

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  🤖 TurtleBot3 Navigation + Localization - Complete Setup"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash
source /home/shrinath/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash

# Check if navigation is running
echo "Step 1: Checking system status..."
echo ""

if ! ros2 node list | grep -q "/amcl"; then
    echo "❌ Navigation is NOT running!"
    echo ""
    echo "Please start navigation first in another terminal:"
    echo "  cd ~/RIS-TurtleBot3-Autonomous-Navigation"
    echo "  source install/setup.bash"
    echo "  ros2 launch my_simulations nav2_maze.launch.py"
    echo ""
    echo "Then run this script again."
    exit 1
fi

echo "✓ AMCL node is running"

if ros2 topic list | grep -q "/scan"; then
    echo "✓ Laser scan topic exists"
fi

if ros2 topic list | grep -q "/map"; then
    echo "✓ Map topic exists"
fi

echo ""
echo "Step 2: Setting initial pose for localization..."
echo ""

# Set initial pose
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    },
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909]
  }
}" > /dev/null 2>&1

echo "✓ Initial pose published to /initialpose"
echo ""
echo "Step 3: Waiting for AMCL to initialize (5 seconds)..."
sleep 5
echo ""

# Check results
echo "Step 4: Verification..."
echo ""

if timeout 2 ros2 topic hz /particle_cloud > /dev/null 2>&1; then
    echo "✓ Particle cloud is publishing"
else
    echo "⚠ Particle cloud not detected yet"
fi

if ros2 topic echo /amcl_pose --once > /dev/null 2>&1; then
    echo "✓ AMCL pose is publishing"
else
    echo "⚠ AMCL pose not detected yet"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  ✨ READY TO START RVIZ!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Now open RViz in another terminal:"
echo ""
echo "  cd ~/RIS-TurtleBot3-Autonomous-Navigation"
echo "  ./start_rviz.sh"
echo ""
echo "Or use standard command:"
echo "  ros2 run rviz2 rviz2 -d config/nav2_default_view.rviz"
echo ""
echo "In RViz you should see:"
echo "  ✓ Green particle cloud (AMCL particles)"
echo "  ✓ Yellow arrow (AMCL pose estimate)"
echo "  ✓ Red laser scan dots"
echo "  ✓ Black/white map"
echo "  ✓ Blue/white robot model"
echo ""
echo "Navigation panel should show:"
echo "  • Navigation: Ready"
echo "  • Localization: Converged"
echo "  • Feedback: Idle"
echo ""
echo "If localization still shows 'unknown':"
echo "  1. Wait 10-15 more seconds for convergence"
echo "  2. Or click '2D Pose Estimate' button in RViz"
echo "  3. Click on map at robot position (center)"
echo ""
echo "Then set navigation goals with 'Nav2 Goal' button!"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

default_nav_to_pose_bt_xml: "/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml"
