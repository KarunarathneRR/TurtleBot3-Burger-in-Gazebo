#!/bin/bash

# System Verification Script
# Checks if all components are working correctly

echo "=========================================="
echo "TurtleBot3 Navigation System Verification"
echo "=========================================="
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Check if Gazebo is running
echo -n "1. Gazebo Running: "
if ros2 topic list | grep -q "/scan"; then
    echo -e "${GREEN}✓ YES${NC}"
else
    echo -e "${RED}✗ NO - Start Gazebo first!${NC}"
    exit 1
fi

# Check if Nav2 nodes are running
echo -n "2. Nav2 Nodes: "
NODE_COUNT=$(ros2 node list 2>/dev/null | wc -l)
if [ "$NODE_COUNT" -gt 15 ]; then
    echo -e "${GREEN}✓ Running ($NODE_COUNT nodes)${NC}"
else
    echo -e "${RED}✗ Only $NODE_COUNT nodes - Nav2 not fully started${NC}"
fi

# Check map_server
echo -n "3. Map Server: "
if ros2 lifecycle get /map_server 2>/dev/null | grep -q "active"; then
    echo -e "${GREEN}✓ Active${NC}"
else
    echo -e "${RED}✗ Not Active${NC}"
fi

# Check AMCL
echo -n "4. AMCL Localization: "
if ros2 lifecycle get /amcl 2>/dev/null | grep -q "active"; then
    echo -e "${GREEN}✓ Active${NC}"
else
    echo -e "${RED}✗ Not Active${NC}"
fi

# Check controller
echo -n "5. Controller Server: "
if ros2 lifecycle get /controller_server 2>/dev/null | grep -q "active"; then
    echo -e "${GREEN}✓ Active${NC}"
else
    echo -e "${RED}✗ Not Active${NC}"
fi

# Check map publishing
echo -n "6. Map Publishing: "
if timeout 2 ros2 topic echo /map --once >/dev/null 2>&1; then
    MAP_INFO=$(ros2 topic echo /map --once 2>/dev/null | grep -E "width|height" | head -2)
    WIDTH=$(echo "$MAP_INFO" | grep width | awk '{print $2}')
    HEIGHT=$(echo "$MAP_INFO" | grep height | awk '{print $2}')
    echo -e "${GREEN}✓ YES (${WIDTH}x${HEIGHT})${NC}"
else
    echo -e "${RED}✗ NO${NC}"
fi

# Check LIDAR scan
echo -n "7. LIDAR Scan: "
if timeout 2 ros2 topic echo /scan --once >/dev/null 2>&1; then
    echo -e "${GREEN}✓ Publishing${NC}"
else
    echo -e "${RED}✗ Not Publishing${NC}"
fi

# Check RViz
echo -n "8. RViz Running: "
if ros2 node list 2>/dev/null | grep -q "rviz"; then
    echo -e "${GREEN}✓ YES${NC}"
else
    echo -e "${YELLOW}⚠ NO - Start RViz manually${NC}"
fi

# Check QoS compatibility
echo -n "9. Map QoS Match: "
MAP_INFO=$(ros2 topic info /map -v 2>/dev/null)
PUB_QOS=$(echo "$MAP_INFO" | grep -A 10 "Node name: map_server" | grep "Durability:" | awk '{print $2}')
SUB_QOS=$(echo "$MAP_INFO" | grep -A 10 "Node name: rviz" | grep "Durability:" | awk '{print $2}')

if [ "$PUB_QOS" == "$SUB_QOS" ] && [ "$PUB_QOS" == "TRANSIENT_LOCAL" ]; then
    echo -e "${GREEN}✓ Compatible (TRANSIENT_LOCAL)${NC}"
else
    echo -e "${RED}✗ Mismatch (Pub: $PUB_QOS, Sub: $SUB_QOS)${NC}"
fi

# Check TF frames
echo -n "10. TF Frames: "
if timeout 2 ros2 run tf2_ros tf2_echo map base_footprint >/dev/null 2>&1; then
    echo -e "${GREEN}✓ Complete (map→odom→base_footprint)${NC}"
else
    echo -e "${YELLOW}⚠ Incomplete - May need initial pose${NC}"
fi

echo ""
echo "=========================================="
echo "Status Summary:"
echo "=========================================="

# Overall status
if ros2 lifecycle get /map_server 2>/dev/null | grep -q "active" && \
   ros2 lifecycle get /amcl 2>/dev/null | grep -q "active" && \
   ros2 lifecycle get /controller_server 2>/dev/null | grep -q "active" && \
   timeout 2 ros2 topic echo /map --once >/dev/null 2>&1; then
    echo -e "${GREEN}✅ System is FULLY OPERATIONAL${NC}"
    echo ""
    echo "You can now:"
    echo "  1. Open RViz (if not already open)"
    echo "  2. You should see the maze map"
    echo "  3. Click 'Nav2 Goal' to set navigation goals"
    echo "  4. Robot will navigate in both Gazebo and RViz"
else
    echo -e "${YELLOW}⚠ System needs attention - check failed items above${NC}"
fi

echo ""
