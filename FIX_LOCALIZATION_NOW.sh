#!/bin/bash

# Quick fix script for localization display in RViz
# Run this after starting Gazebo and Navigation

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  🎯 LOCALIZATION FIX - Setting Initial Pose"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash
source /home/shrinath/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash

echo "Step 1: Checking if AMCL is running..."
if ros2 node list | grep -q "/amcl"; then
    echo "  ✓ AMCL node is running"
else
    echo "  ✗ AMCL is not running! Start navigation first."
    echo ""
    echo "Run in another terminal:"
    echo "  ros2 launch my_simulations nav2_maze.launch.py"
    exit 1
fi

echo ""
echo "Step 2: Checking laser scan data..."
if ros2 topic list | grep -q "/scan"; then
    echo "  ✓ /scan topic exists"
    SCAN_HZ=$(timeout 2 ros2 topic hz /scan 2>&1 | grep "average rate" | awk '{print $3}')
    if [ ! -z "$SCAN_HZ" ]; then
        echo "  ✓ Laser scan publishing at ~${SCAN_HZ} Hz"
    fi
else
    echo "  ✗ /scan topic not found!"
fi

echo ""
echo "Step 3: Checking map data..."
if ros2 topic list | grep -q "/map"; then
    echo "  ✓ /map topic exists"
else
    echo "  ✗ /map topic not found!"
fi

echo ""
echo "Step 4: Setting initial pose at (0, 0, 0)..."
echo "  Publishing to /initialpose..."

ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'map'
  },
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    },
    covariance: [
      0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909
    ]
  }
}" > /dev/null 2>&1

echo "  ✓ Initial pose published!"

echo ""
echo "Step 5: Waiting for AMCL to respond (5 seconds)..."
sleep 5

echo ""
echo "Step 6: Checking AMCL pose..."
if ros2 topic echo /amcl_pose --once > /dev/null 2>&1; then
    echo "  ✓ AMCL is publishing pose estimates"
else
    echo "  ⚠ AMCL pose not detected yet - may need more time"
fi

echo ""
echo "Step 7: Checking particle cloud..."
if timeout 2 ros2 topic hz /particle_cloud > /dev/null 2>&1; then
    echo "  ✓ Particle cloud is publishing"
else
    echo "  ⚠ Particle cloud not detected - localization may still be initializing"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  ✨ LOCALIZATION INITIALIZED!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "What to check in RViz:"
echo "  1. Look for GREEN particle cloud (arrows) around robot"
echo "  2. Look for YELLOW arrow showing AMCL pose estimate"
echo "  3. Navigation panel should show 'Localization: Converged'"
echo ""
echo "If you don't see the visualizations:"
echo "  • Make sure RViz is using the updated config:"
echo "    ros2 run rviz2 rviz2 -d config/nav2_default_view.rviz"
echo ""
echo "  • Check these displays are ENABLED (checked) in RViz:"
echo "    ✓ Amcl Particle Swarm"
echo "    ✓ Amcl Pose"
echo "    ✓ LaserScan"
echo "    ✓ Map"
echo ""
echo "  • If localization still shows 'unknown', wait 10-15 seconds"
echo "    for particles to converge, or try clicking '2D Pose Estimate'"
echo "    button in RViz and clicking on the map at robot location"
echo ""
echo "Now you can set navigation goals with the 'Nav2 Goal' button!"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
