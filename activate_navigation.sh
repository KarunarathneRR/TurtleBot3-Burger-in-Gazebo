#!/bin/bash

# NAVIGATION ACTIVATION FIX
# Run this if navigation shows as inactive after startup

echo "=================================="
echo "Navigation Activation Fix"
echo "=================================="
echo ""

# Check current states
echo "Current States:"
echo -n "  bt_navigator: "
BT_STATE=$(ros2 lifecycle get /bt_navigator 2>/dev/null | grep -o "active\|inactive")
echo "$BT_STATE"

echo -n "  behavior_server: "
BEHAVIOR_STATE=$(ros2 lifecycle get /behavior_server 2>/dev/null | grep -o "active\|inactive")
echo "$BEHAVIOR_STATE"

echo ""

# Activate if needed
if [ "$BT_STATE" == "inactive" ] || [ "$BEHAVIOR_STATE" == "inactive" ]; then
    echo "Activating inactive nodes..."
    
    if [ "$BEHAVIOR_STATE" == "inactive" ]; then
        echo "  Activating behavior_server..."
        ros2 lifecycle set /behavior_server activate
    fi
    
    sleep 2
    
    if [ "$BT_STATE" == "inactive" ]; then
        echo "  Activating bt_navigator..."
        ros2 lifecycle set /bt_navigator activate
    fi
    
    sleep 2
    
    echo ""
    echo "Republishing initial pose..."
    ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
      header: {frame_id: "map"},
      pose: {
        pose: {
          position: {x: 0.0, y: 0.0, z: 0.0},
          orientation: {w: 1.0}
        }
      }
    }' >/dev/null 2>&1
    
    echo ""
    echo "✅ Navigation activated!"
else
    echo "✅ All navigation nodes already active!"
fi

echo ""
echo "Final States:"
echo -n "  bt_navigator: "
ros2 lifecycle get /bt_navigator 2>/dev/null
echo -n "  behavior_server: "
ros2 lifecycle get /behavior_server 2>/dev/null

echo ""
echo "=================================="
echo "Navigation should now be active in RViz"
echo "=================================="
