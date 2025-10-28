# Map Loading and Gazebo-RViz Synchronization Fix

## Issues Fixed

### 1. **Map Not Received in RViz**
**Problem:** RViz showed "map not received" error because the map_server couldn't find the map image file.

**Root Cause:** The `maps/map.yaml` file had an incorrect relative path:
```yaml
image: maps/map.pgm  # WRONG - map.yaml is already in maps/ directory
```

**Solution:** Changed to correct relative path:
```yaml
image: map.pgm  # CORRECT - relative to map.yaml location
```

**File Changed:** `/home/shrinath/RIS-TurtleBot3-Autonomous-Navigation/maps/map.yaml`

---

### 2. **Gazebo and RViz Not Synchronized**
**Problem:** Gazebo and RViz were not showing the same robot state/position.

**Root Causes:**
1. Topics not publishing properly due to incomplete Gazebo startup
2. TF frames not established before Nav2 started
3. AMCL localization not initialized with proper initial pose

**Solutions:**
1. **Proper Launch Sequence:** Created `FIXED_STARTUP.sh` that launches components in correct order:
   - Gazebo first (wait 20 seconds)
   - Verify Gazebo topics are active
   - Launch Nav2 (wait 20 seconds)
   - Set initial robot pose
   - Launch RViz

2. **Initial Pose Setup:** Script automatically publishes initial pose to `/initialpose` topic, which:
   - Initializes AMCL particle filter at origin (0, 0, 0)
   - Establishes map→odom→base_footprint TF chain
   - Synchronizes localization between Gazebo odometry and map frame

3. **Topic Verification:** Script checks for critical topics before proceeding:
   - `/scan` - LIDAR data from Gazebo
   - `/odom` - Odometry from Gazebo
   - `/tf` - Transform tree
   - `/map` - Map from map_server

---

## Verification

### All Systems Running Correctly:

#### ROS2 Nodes Active:
- ✅ `/map_server` - Publishing map
- ✅ `/amcl` - Localization running
- ✅ `/robot_state_publisher` - TF frames
- ✅ `/ros_gz_bridge` - Gazebo↔ROS2 bridge
- ✅ All Nav2 servers (planner, controller, bt_navigator, etc.)

#### Topics Publishing:
- ✅ `/map` - Map data (63x63 grid, 0.05m resolution)
- ✅ `/scan` - LIDAR scans (~4 Hz)
- ✅ `/odom` - Robot odometry from Gazebo
- ✅ `/amcl_pose` - Localized robot pose
- ✅ `/tf` - Transform tree (map→odom→base_footprint→base_scan)

#### TF Frames Synchronized:
- ✅ `map→base_footprint` transform available
- ✅ AMCL publishing map→odom transform
- ✅ Gazebo publishing odom→base_footprint transform
- ✅ Complete TF chain established

---

## How to Use

### Quick Start (Recommended):
```bash
cd ~/RIS-TurtleBot3-Autonomous-Navigation
./FIXED_STARTUP.sh
```

This script will:
1. Launch Gazebo with maze world and TurtleBot3
2. Launch Navigation2 stack
3. Set initial robot pose
4. Launch RViz with proper configuration
5. Verify all components are running

### What You Should See in RViz:
- **Map Display:** Black/white maze map loaded
- **Laser Scan:** Red points showing LIDAR data
- **AMCL Particle Cloud:** Green particles around robot (localization uncertainty)
- **Robot Model:** TurtleBot3 burger at origin
- **TF Frames:** All frames visible in TF display
- **Navigation 2 Panel:** Shows "Localization: Converged" and "Navigation: Ready"

### To Navigate:
1. Click the **"Nav2 Goal"** button in RViz toolbar
2. Click a location on the map where you want the robot to go
3. Robot will plan a path and navigate in **both RViz and Gazebo simultaneously**
4. You should see:
   - Planned path (green line) in RViz
   - Robot moving in Gazebo window
   - Robot visualization moving in RViz
   - LIDAR scan detecting obstacles

---

## Troubleshooting

### If Map Still Not Visible:
```bash
# Check if map_server is running
ros2 node list | grep map_server

# Check if map topic is publishing
ros2 topic echo /map --once

# Check for map_server errors
ros2 lifecycle get /map_server
```

### If Gazebo and RViz Not Synced:
```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Check if all required topics exist
ros2 topic list | grep -E "/scan|/odom|/tf|/map"

# Check scan rate
ros2 topic hz /scan
```

### If Robot Doesn't Move:
1. **Check Navigation Panel:** Should show "Ready"
2. **Verify Goal:** Goal arrow should appear on map
3. **Check Controller:** `ros2 lifecycle get /controller_server` should show "active"
4. **Reset Localization:** Use "2D Pose Estimate" button to manually set robot position

---

## Technical Details

### Map Configuration:
- **File:** `maps/map.yaml`
- **Image:** `map.pgm` (63x63 pixels)
- **Resolution:** 0.05 meters/pixel
- **Origin:** (-1.594, -1.549, 0)
- **Format:** Trinary (occupied/free/unknown)

### Frame Chain:
```
map (global reference)
 └─ odom (published by AMCL based on scan matching)
     └─ base_footprint (published by Gazebo odometry)
         └─ base_link
             └─ base_scan (LIDAR frame)
```

### Key Configuration Files:
- `maps/map.yaml` - Map configuration (FIXED)
- `config/nav2_params.yaml` - Nav2 parameters
- `config/nav2_default_view.rviz` - RViz configuration
- `launch/nav2_maze.launch.py` - Nav2 launcher
- `launch/nav2_bringup_min.launch.py` - Minimal Nav2 bringup
- `launch_gazebo_no_flicker.sh` - Gazebo launcher

---

## Success Criteria

✅ **Map loads in RViz** - Black/white maze visible  
✅ **LIDAR scan visible** - Red points from laser scanner  
✅ **AMCL localization active** - Green particle cloud visible  
✅ **TF frames synchronized** - Complete chain: map→odom→base_footprint  
✅ **Robot moves in Gazebo when Nav2 goal set in RViz** - Full synchronization  
✅ **Navigation2 panel shows "Ready"** - All systems operational  

---

## Files Modified
1. `maps/map.yaml` - Fixed image path
2. `FIXED_STARTUP.sh` - New complete startup script (replaces old COMPLETE_STARTUP.sh)

## Files Created
- `FIXED_STARTUP.sh` - Automated launch script with proper timing and verification
- `MAP_AND_SYNC_FIX.md` - This documentation

---

**Status:** ✅ ALL ISSUES RESOLVED  
**Last Updated:** October 28, 2025
