# Autonomous Navigation with Nav2 (ROS2 Jazzy + Gazebo + Nav2)

Navigate a simulated differential drive robot autonomously through a walled Gazebo maze using the Nav2 stack — AMCL localization on a pre-built map, global path planning with NavfnPlanner, local trajectory tracking with DWB, and recovery behaviors.

**GPU/CPU auto-detection**: the launch file detects whether an NVIDIA GPU is available and selects the appropriate URDF (360-sample 10Hz lidar on GPU, 180-sample 5Hz on CPU) and Gazebo render settings automatically.

This is a proper ROS2 ament_python package — build it with `colcon`, source the workspace, and launch.

## How it works
1. A diff drive robot with a 360° lidar is spawned in a walled Gazebo world (same world as the SLAM tutorial)
2. `map_server` loads a pre-built occupancy grid map (included, or generate your own with SLAM)
3. `AMCL` localizes the robot on the map using the lidar `/scan` and `/tf`
4. The **Nav2 stack** provides:
   - `planner_server` — global path planning (NavfnPlanner / Dijkstra)
   - `controller_server` — local trajectory tracking (DWB)
   - `behavior_server` — recovery behaviors (spin, backup, wait)
   - `bt_navigator` — behavior tree orchestration
   - `velocity_smoother` — smooth cmd_vel output
5. You can send goals via **RViz 2D Nav Goal** tool, the `navigate.sh` CLI script, or the `waypoint_follower.py` Python node
6. The launch file auto-detects GPU/CPU and adjusts lidar resolution and Gazebo rendering accordingly

## Requirements
- Ubuntu 24.04 + ROS 2 Jazzy
- Gazebo (gz-sim, ships with `ros-jazzy-ros-gz`)
- The following ROS packages:
  ```bash
  sudo apt update
  sudo apt install -y \
      ros-jazzy-ros-gz \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-rviz2 \
      ros-jazzy-slam-toolbox \
      ros-jazzy-nav2-bringup \
      ros-jazzy-nav2-simple-commander \
      ros-jazzy-teleop-twist-keyboard
  ```

## Build (colcon)

```bash
# 1. Create a workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. Clone this repo
git clone https://github.com/Ahmed-m-abbas/ros2-autonomous-nav.git

# 3. Build the package
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ros2_autonomous_nav --symlink-install

# 4. Source the workspace overlay
source ~/ros2_ws/install/setup.bash
```

## Option A: Use the included map (quickstart)

A pre-generated map of the maze is included in `maps/`. Jump straight to navigation.

## Option B: Generate your own map with SLAM

```bash
# Terminal 1 — SLAM mode (robot + Gazebo + slam_toolbox)
bash slam.sh

# Terminal 2 — RViz
bash rviz.sh

# Terminal 3 — drive the robot around the maze
bash drive.sh

# Terminal 4 — when the map looks good, save it
bash save_map.sh nav_world
```

## Run Navigation

You need **three terminals**. Source the workspace in every terminal first:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

**Terminal 1** — robot + Gazebo + Nav2 stack (auto-detects GPU/CPU):
```bash
bash run.sh
# or: ros2 launch ros2_autonomous_nav robot_nav.launch.py
```

**Terminal 2** — RViz with Nav2 displays (map, costmaps, paths, AMCL particles):
```bash
bash rviz.sh
```

**Terminal 3** — send navigation goals (pick one method):

```bash
# Method 1: Click "2D Nav Goal" in RViz and click on the map

# Method 2: CLI single goal
bash navigate.sh 3.5 3.5 1.57

# Method 3: Waypoint tour of the maze
bash waypoints.sh
```

## GPU/CPU Auto-Detection

The launch file runs `nvidia-smi` at startup:

| Feature        | GPU mode           | CPU mode            |
|----------------|--------------------|---------------------|
| Lidar samples  | 360                | 180                 |
| Lidar rate     | 10 Hz              | 5 Hz                |
| Lidar viz      | enabled            | disabled            |
| Render engine  | ogre2 (HW accel)   | ogre2 (SW fallback) |
| URDF           | my_robot_gpu.urdf  | my_robot_cpu.urdf   |

## Project layout
```
ros2_autonomous_nav/
├── package.xml                              # ament_python manifest
├── setup.py / setup.cfg                     # colcon build
├── resource/ros2_autonomous_nav             # ament resource marker
├── urdf/
│   ├── my_robot_gpu.urdf                    # 360 samples, 10 Hz (GPU)
│   └── my_robot_cpu.urdf                    # 180 samples, 5 Hz (CPU)
├── worlds/nav_world.sdf                     # walled maze (same as SLAM)
├── maps/
│   ├── nav_world.pgm                        # pre-built occupancy grid
│   └── nav_world.yaml                       # map metadata
├── launch/
│   ├── robot_nav.launch.py                  # robot + Gazebo + Nav2 (GPU/CPU)
│   └── slam_map.launch.py                   # SLAM mode for map generation
├── config/
│   ├── nav2_params.yaml                     # full Nav2 stack configuration
│   ├── nav2_rviz.rviz                       # RViz Nav2 display config
│   └── mapper_params_online_async.yaml      # slam_toolbox params
├── scripts/
│   ├── generate_map.py                      # generate map from SDF geometry
│   └── waypoint_follower.py                 # nav2_simple_commander waypoints
├── run.sh / slam.sh / rviz.sh / drive.sh
├── save_map.sh / navigate.sh / waypoints.sh
```

## Tech
- ROS 2 Jazzy
- URDF (GPU/CPU variants)
- Gazebo (gz-sim) + `gpu_lidar` sensor + `DiffDrive` plugin
- `ros_gz_bridge` (Clock + Twist + Odometry + LaserScan + TF + JointState)
- Nav2 (AMCL, NavfnPlanner, DWB controller, behavior tree navigator)
- `nav2_simple_commander` (Python waypoint API)
- `slam_toolbox` (optional map generation)
- RViz2

## License
MIT
