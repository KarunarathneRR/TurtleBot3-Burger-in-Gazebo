# RIS-TurtleBot3-Autonomous-Navigation

This repository contains a Gazebo (ros_gz_sim) maze world, Nav2 configuration, and a small DQN trainer/runner for TurtleBot3 that uses ROS 2 and PyTorch.

## 🎯 **→→→ READ [START_HERE.md](START_HERE.md) FOR COMPLETE INSTRUCTIONS! ←←←**

## 🚨 **HAVING ISSUES? Navigation shows "unknown" in RViz?**
## → **Read [FIX_NAVIGATION.md](FIX_NAVIGATION.md) for the complete solution!**

This README explains how to prepare the environment, build the workspace, run the simulation and Nav2, and train/run the DQN agent.

## Quick Launch (Easiest Method)
```bash
cd /home/shrinath/RIS-TurtleBot3-Autonomous-Navigation
./scripts/launch_nav2_system.sh
```

Or check current system status:
```bash
./scripts/check_nav2_status.sh
```

---

## Contents
- `launch/maze_world.launch.py` — launches the Gazebo maze world, spawns TurtleBot3, and starts robot_state_publisher
- `launch/nav2_maze.launch.py` — Nav2 bringup using `maps/map.yaml` and `config/nav2_params.yaml`
- `config/nav2_params.yaml` — Nav2 configuration (costmaps, planners, controllers)
- `tb3_dqn_nav/` — Python package with `dqn_train` and `dqn_run` nodes
- `worlds/maze_world.world`, `models/` — Gazebo world and model resources

---

## Tested environment / assumptions
- OS: Ubuntu 24.04 (codename: `noble`) — this repo was prepared and tested on that system
- ROS 2 distribution: Jazzy (package names `ros-jazzy-*`) — the workspace was built with Jazzy
- Python 3 and pip available
- A GPU is optional; PyTorch CPU mode works but training will be slow

If you use a different ROS 2 distro, swap `jazzy` with your distro name and install corresponding packages.

---

## Quick install (copy-paste)
Run these commands from a shell (adjust distro name if you're not using `jazzy`):

1) Install ROS Jazzy and common packages (requires sudo):

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-turtlebot3-gazebo \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
  python3-colcon-common-extensions python3-rosdep
```

2) Initialize rosdep (if not already):

```bash
sudo rosdep init  # run once system-wide
rosdep update
```

3) (Optional) Install PyTorch. Choose the proper wheel for CPU or CUDA:

# CPU example
```bash
python3 -m pip install --user torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

# CUDA example (pick the right CUDA + wheel from pytorch.org)
```bash
python3 -m pip install --user torch torchvision --index-url https://download.pytorch.org/whl/cu118
```

4) Install Python tooling (optional):

```bash
python3 -m pip install --user -U pip setuptools wheel
```

---

## Build the workspace
From the repository root (`/home/shrinath/RIS-TurtleBot3-Autonomous-Navigation`):

```bash
# source ROS 2 (Jazzy)
source /opt/ros/jazzy/setup.bash
# install system deps for packages in this workspace
rosdep install --from-paths . --ignore-src -r -y
# build
colcon build --symlink-install
# source the workspace
source install/setup.bash
```

Notes:
- `--symlink-install` is convenient for Python packages during development.
- If rosdep reports missing system deps, follow its suggested apt/pip commands.

---

## Launching
Open separate terminals (or use tmux). Always source both ROS and the workspace before running commands in each terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /home/shrinath/RIS-TurtleBot3-Autonomous-Navigation/install/setup.bash
```

1) Launch the maze world (Gazebo + spawn TurtleBot3):

```bash
ros2 launch my_simulations maze_world.launch.py
```

This starts `ros_gz_sim` with `worlds/maze_world.world`, spawns TurtleBot3, and starts `robot_state_publisher`.

2) Launch Nav2 for this map:

```bash
ros2 launch my_simulations nav2_maze.launch.py
```

This includes the `turtlebot3_navigation2` bringup (map param and `config/nav2_params.yaml`).

3) Run the DQN runtime node (needs a TorchScript model saved on disk):

```bash
ros2 run tb3_dqn_nav dqn_run
```

By default `dqn_run` attempts to load `/home/<user>/.tb3_dqn.pt`. If you don't have a model yet, run training (next section).

---

## Training the DQN policy
The trainer collects LaserScan observations and commands `/cmd_vel` in the running simulator, and exports a TorchScript module.

Example (CPU training):

```bash
ros2 run tb3_dqn_nav dqn_train --episodes 200 --device cpu
```

To save the model elsewhere:

```bash
ros2 run tb3_dqn_nav dqn_train --model-path ~/models/tb3_dqn.pt --episodes 200
```

After training completes (or you interrupt it), the trainer exports a TorchScript model to the given path. Place/rename it to `~/.tb3_dqn.pt` if you want the default `dqn_run` to pick it up.

---

## Verification / debug
- List ROS nodes:

```bash
ros2 node list
```

- Check LaserScan topic:

```bash
ros2 topic echo /scan --once
```

- Watch cmd_vel messages (from DQN or teleop):

```bash
ros2 topic echo /cmd_vel
```

- Check reset services (trainer uses several reset services):

```bash
ros2 service list | grep reset
```

- If the simulator doesn't start or models can't be found, check the `GZ_SIM_RESOURCE_PATH` environment variable (the launch file sets it to include `my_simulations/models`).

---

## Notes & small improvements to consider
- `dqn_run.py` currently constructs the model path using `os.getlogin()` and a hard-coded filename. You may want to modify it to accept a command-line argument or ROS param for `model_path` (I can add this small change if you'd like).
- If you want an automated startup, I can add a `scripts/run_all.sh` that opens tmux panes and launches world, nav2, and the runner.
- If running headless or on a remote server, configure `ros_gz_sim` for headless operation.

---

## Where things live (quick reference)
- Launch files: `launch/maze_world.launch.py`, `launch/nav2_maze.launch.py`
- Nav2 params: `config/nav2_params.yaml`
- World: `worlds/maze_world.world`
- DQN code: `tb3_dqn_nav/tb3_dqn_nav/dqn_train.py`, `tb3_dqn_nav/tb3_dqn_nav/dqn_run.py`

---

If you want, I can now:
- Add `--model-path` CLI param to `dqn_run.py` and `dqn_train` (small code edit + quick local test),
- Create a `scripts/run_all.sh` tmux launcher,
- Or add additional verification scripts.

Tell me which of those you'd like next.
