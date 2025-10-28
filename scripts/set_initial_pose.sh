#!/bin/bash

# Script to set initial pose for AMCL localization
# Usage: ./set_initial_pose.sh [x] [y] [yaw]
# Example: ./set_initial_pose.sh 0.0 0.0 0.0

# Default values (robot spawn position)
X_POS=${1:-0.0}
Y_POS=${2:-0.0}
YAW=${3:-0.0}

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Setting Initial Pose for AMCL Localization"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Position: x=$X_POS, y=$Y_POS"
echo "Orientation: yaw=$YAW radians"
echo ""

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash
source /home/shrinath/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash

# Convert yaw to quaternion (simplified - only for z-axis rotation)
# For a rotation around z-axis: qz = sin(yaw/2), qw = cos(yaw/2)
QZ=$(echo "scale=6; s($YAW/2)" | bc -l)
QW=$(echo "scale=6; c($YAW/2)" | bc -l)

echo "Publishing initial pose to /initialpose topic..."
echo ""

# Publish initial pose
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'map'
  },
  pose: {
    pose: {
      position: {x: $X_POS, y: $Y_POS, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}
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
}"

echo ""
echo "✓ Initial pose sent!"
echo ""
echo "Next steps:"
echo "  1. Check RViz - you should see green particle cloud appear"
echo "  2. Wait 5-10 seconds for particles to converge"
echo "  3. Localization should change from 'unknown' to 'Converged'"
echo "  4. Now you can set navigation goals!"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
