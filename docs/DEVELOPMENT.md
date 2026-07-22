# Meister Development Guide

Team Ume Onigiri - ROS 2 Jazzy + Gazebo Harmonic + Mecanum Robot

---

## Table of Contents

1. [Environment Setup](#environment-setup)
2. [Directory Structure](#directory-structure)
3. [Quick Start](#quick-start)
4. [Common Commands](#common-commands)
5. [Custom URDF Editing](#custom-urdf-editing)
6. [Parameter Tuning Guide](#parameter-tuning-guide)
7. [Team Development](#team-development)
8. [Sim-to-Real Migration](#sim-to-real-migration)

---

## Environment Setup

### Prerequisites

- Ubuntu 24.04 (Noble)
- ROS 2 Jazzy
- Gazebo Harmonic (8.11.0+)

### Install ROS 2 Jazzy

Follow the official installation instructions at https://docs.ros.org/en/jazzy/Installation.html.

### Install apt packages

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
  ros-jazzy-xacro ros-jazzy-joint-state-publisher \
  ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-nav2-smac-planner
```

### Initialize rosdep

```bash
sudo rosdep init        # (skip if already done)
rosdep update
```

### Clone and build

```bash
cd /home/so/Meister

# Initialize submodule (rosnav dependency)
git submodule update --init --recursive

# Create symlink so colcon discovers the rosnav package
ln -sf ../rosnav_submodule/src/diff_drive_robot-main src/rosnav

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Install package-level dependencies
rosdep install -i --from-path src --rosdistro jazzy -y

# Build
colcon build --symlink-install

# Source workspace overlay
source install/setup.bash
```

### Verify setup

```bash
# Check Gazebo
gz sim --version

# Check URDF parsing
xacro src/meister_robot/urdf/robot.urdf.xacro

# Check colcon build
colcon build --symlink-install --packages-select meister_robot
```

---

## Directory Structure

```
Meister/                              # ROS 2 workspace root
├── src/
│   ├── meister_robot/                # Custom robot package
│   │   ├── urdf/
│   │   │   ├── robot.urdf.xacro          # Main robot definition
│   │   │   ├── mecanum_wheel.xacro       # Mecanum wheel macro
│   │   │   ├── lidar.xacro               # 2D LiDAR macro
│   │   │   └── inertial_macros.xacro     # Inertia calculation helpers
│   │   ├── config/
│   │   │   ├── nav2_params.yaml          # Nav2 stack parameters
│   │   │   ├── slam_params.yaml          # SLAM Toolbox parameters
│   │   │   ├── mppi_params.yaml          # MPPI controller (mecanum tuned)
│   │   │   ├── costmap_params.yaml       # Costmap configuration
│   │   │   ├── collision_monitor_params.yaml  # Safety collision zones
│   │   │   └── amcl_params.yaml          # AMCL localization parameters
│   │   ├── launch/
│   │   │   ├── meister_slam_nav.launch.py     # SLAM + Nav2 launch
│   │   │   ├── meister_localization.launch.py # AMCL on pre-built map
│   │   │   └── meister_rviz.launch.py         # RViz only
│   │   ├── rviz/
│   │   │   └── meister_nav.rviz          # RViz visualization config
│   │   ├── worlds/
│   │   │   └── maze_with_robot.world     # Custom world file
│   │   ├── models/
│   │   │   └── meister_robot/            # Gazebo model assets
│   │   ├── scripts/
│   │   │   ├── mecanum_controller.py     # Twist-to-4-wheel conversion
│   │   │   └── spawn_robot.py            # Robot spawn utility
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   │
│   └── rosnav/ -> ../rosnav_submodule/src/diff_drive_robot-main/
│                                           # Symlink to rosnav submodule
│
├── rosnav_submodule/                   # git submodule (rosnav repo)
│   └── src/diff_drive_robot-main/      # Upstream rosnav packages
│
├── build/                              # colcon build output (gitignored)
├── install/                            # colcon install output (gitignored)
├── log/                                # colcon log output (gitignored)
│
├── scripts/                            # Workspace-level scripts
│   ├── setup.sh                        # Dependency installation
│   ├── build.sh                        # colcon build wrapper
│   ├── sim.sh                          # One-command launch
│   └── record.sh                       # ros2 bag recording
│
└── docs/
    ├── DEVELOPMENT.md                  # This file
    └── TROUBLESHOOTING.md              # Common issues reference
```

### TF Tree

```
map
 └── odom (Gazebo odometry)
      └── base_footprint (ground projection)
           └── base_link (robot base)
                ├── laser_link (2D LiDAR)
                ├── lf_wheel (left front)
                ├── rf_wheel (right front)
                ├── lr_wheel (left rear)
                └── rr_wheel (right rear)
```

---

## Quick Start

### One-command launch

```bash
cd /home/so/Meister

source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch meister_robot meister_slam_nav.launch.py \
  world_name:=maze explore:=true
```

This launches:
1. Gazebo Harmonic with the maze world
2. SLAM Toolbox for online mapping
3. Nav2 stack (planner, controller, costmaps)
4. Collision monitor safety layer
5. RViz2 with full visualization
6. Frontier explorer for autonomous exploration

### Launch arguments

| Argument   | Default   | Description                              |
|------------|-----------|------------------------------------------|
| world_name | maze      | Gazebo world name in rosnav worlds/      |
| explore    | true      | Auto-start frontier explorer             |
| slam       | true      | true=SLAM Toolbox, false=AMCL            |
| rviz       | true      | Launch RViz2                             |
| headless   | false     | Skip Gazebo GUI and RViz                 |
| safety     | true      | Launch collision monitor                 |
| drive_type | mecanum   | Drive base type                          |

### Keyboarding teleop

```bash
# In a separate terminal
source /opt/ros/jazzy/setup.bash
source /home/so/Meister/install/setup.bash
export ROS_DOMAIN_ID=42

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Autonomous exploration

When launched with `explore:=true`, the robot autonomously explores the environment. Use RViz to monitor progress:

- Magenta path = planned route
- Green overlay = explored area
- Red cells = obstacles
- Blue arrow = goal pose (click "2D Nav Goal" to set manual goal)

---

## Common Commands

### Build

```bash
# Full workspace
colcon build --symlink-install

# Single package (faster)
colcon build --symlink-install --packages-select meister_robot

# With cmake debug
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### Source

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

### Launch variations

```bash
# SLAM + exploration (default)
ros2 launch meister_robot meister_slam_nav.launch.py

# Localization only (AMCL on pre-built map)
ros2 launch meister_robot meister_slam_nav.launch.py slam:=false

# Headless (server mode, no GUI)
ros2 launch meister_robot meister_slam_nav.launch.py headless:=true

# Different world
ros2 launch meister_robot meister_slam_nav.launch.py world_name:=warehouse
```

### Diagnostics

```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Check topic frequency
ros2 topic hz /scan /odom /cmd_vel

# List active nodes
ros2 node list

# Get controller parameters
ros2 param get /controller_server/FollowPath holonomic

# Set parameter at runtime
ros2 param set /controller_server/FollowPath vx_max 0.6

# Record a bag
ros2 bag record -a -o recording_name
```

### URDF / xacro

```bash
# Validate URDF
xacro src/meister_robot/urdf/robot.urdf.xacro | check_urdf

# Export to single URDF file
xacro src/meister_robot/urdf/robot.urdf.xacro > robot.urdf

# View robot model in RViz
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(xacro src/meister_robot/urdf/robot.urdf.xacro)"
```

---

## Custom URDF Editing

### File overview

All robot geometry is defined in `src/meister_robot/urdf/`.

| File | Purpose |
|------|---------|
| `robot.urdf.xacro` | Main file: includes all macros, defines chassis and overall layout |
| `mecanum_wheel.xacro` | Wheel macro with continuous joint, transmission, inertial |
| `lidar.xacro` | 2D LiDAR sensor macro with Gazebo gpu_lidar plugin |
| `inertial_macros.xacro` | Reusable inertial calculation macros (box, cylinder, sphere) |

### Editing workflow

1. Edit the xacro file
2. Validate with `xacro robot.urdf.xacro | check_urdf`
3. Restart the launch (no rebuild needed with `--symlink-install`)
4. Check TF tree with `ros2 run tf2_tools view_frames`

### Common edits

**Change chassis dimensions** (in `robot.urdf.xacro`):

```xml
<xacro:property name="body_length" value="0.4"/>
<xacro:property name="body_width"  value="0.3"/>
<xacro:property name="body_height" value="0.12"/>
```

**Adjust wheel positions** (also `robot.urdf.xacro`):

```xml
<!-- lx = half wheelbase, ly = half track -->
<xacro:property name="lx" value="0.15"/>
<xacro:property name="ly" value="0.10"/>
<!-- L = lx + ly = used by mecanum controller inverse kinematics -->
<xacro:property name="L" value="${lx + ly}"/>
```

**Add a sensor frame**:

```xml
<joint name="camera_link_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.2 0 0.15" rpy="0 0 0"/>
</joint>
<link name="camera_link"/>
```

### Xacro tips

- Use `${expression}` for inline math. Supports `+`, `-`, `*`, `/`, `>`, `<`, ternary.
- Use `<xacro:property>` for constants; reference with `$`.
- Use `<xacro:include>` to split URDF across files.
- Use `<xacro:macro>` with `params` for reusable components.
- Insert child blocks with `<xacro:insert_block name="origin"/>` and `<xacro:inertial_box...><origin.../></xacro:inertial_box>`.
- After editing a `.xacro` file, there is no need to rebuild. The launch file runs xacro at startup.

### Mecanum drive in Gazebo

The URDF uses Gazebo's built-in `MecanumDrive` plugin (in `robot.urdf.xacro`):

```xml
<plugin filename="libgz-sim-mecanum-drive-system"
        name="gz::sim::systems::MecanumDrive">
  <front_left_joint>lf_wheel_joint</front_left_joint>
  <front_right_joint>rf_wheel_joint</front_right_joint>
  <back_left_joint>lr_wheel_joint</back_left_joint>
  <back_right_joint>rr_wheel_joint</back_right_joint>
  <wheel_separation>${2 * ly}</wheel_separation>
  <wheelbase>${2 * lx}</wheelbase>
  <wheel_radius>${wheel_radius}</wheel_radius>
  <topic>cmd_vel</topic>
</plugin>
```

For an alternative approach using `mecanum_controller.py` (Twist-to-4-wheel conversion), swap this plugin with individual joint-level control and use `ros_gz_bridge` for joint command bridging.

---

## Parameter Tuning Guide

### MPPI Controller (`config/mppi_params.yaml`)

The MPPI (Model Predictive Path Integral) controller is the key to good mecanum motion.

**Holonomic mode** (required for mecanum):

```yaml
FollowPath:
  holonomic: true          # Must be true for mecanum
  motion_model: "Omni"     # Omnidirectional motion model
```

**Velocity limits** (adjust for your robot):

```yaml
  vx_max: 0.5              # Forward speed (m/s)
  vx_min: -0.35            # Backward speed (m/s)
  vy_max: 0.3              # Lateral speed (m/s) - mecanum advantage
  wz_max: 1.0              # Rotation speed (rad/s)
```

**Acceleration limits** (prevents slip):

```yaml
  ax_max: 0.5              # Forward acceleration
  ax_min: -1.0             # Deceleration (braking)
  ay_max: 0.3              # Lateral acceleration
  ay_min: -0.6             # Lateral deceleration
```

**Optimization parameters**:

```yaml
  time_steps: 56           # Prediction horizon steps (higher = smoother but slower)
  model_dt: 0.08           # Time step (s)
  batch_size: 2000         # Trajectory samples per iteration
  temperature: 0.3         # Exploration temperature (higher = more random)
  gamma: 0.015             # Discount factor
```

**Critic weights** (balance between path following, obstacle avoidance, and goal seeking):

```yaml
  critics: [ConstraintCritic, CostCritic, GoalCritic, GoalAngleCritic,
            PathAlignCritic, PathFollowCritic, PathAngleCritic, PreferForwardCritic]

  ConstraintCritic:  weight=4.0   # Kinematic constraint violation
  CostCritic:        weight=3.81  # Costmap obstacle cost
  GoalCritic:        weight=5.0   # Distance to goal
  PathAlignCritic:   weight=14.0  # Path alignment (most important for smooth path)
  PathFollowCritic:  weight=5.0   # Path following
  PreferForwardCritic: weight=5.0 # Prefer forward motion
```

**Tuning workflow**:

1. Lower `vx_max` to 0.3 for initial testing
2. Increase `PathAlignCritic` weight if the robot wobbles
3. Lower `temperature` if motion is too erratic
4. Increase `time_steps` if the robot gets stuck in local minima
5. Increase `batch_size` if path quality is poor (at cost of CPU)

### SLAM Toolbox (`config/slam_params.yaml`)

**Laser range** (match your LiDAR specs):

```yaml
  laser_min_range: 0.12    # Ignore points closer than this (cm)
  laser_max_range: 12.0    # Maximum valid range (match LiDAR)
  max_laser_range: 12.0    # Must match laser_max_range
```

**Map resolution**:

```yaml
  resolution: 0.05         # 5 cm per pixel (default; 0.025 for finer maps)
  map_update_interval: 5.0 # Seconds between map publications
```

**Loop closure** (enable for large environments):

```yaml
  loop_closure: true
  loop_search_max_distance: 4.0
```

**Tuning workflow**:

1. Verify `laser_min_range` is above your LiDARs physical minimum
2. Set `laser_max_range` to the LiDARs rated range
3. Increase `resolution` to 0.025 for detail-heavy environments
4. Disable `loop_closure` if map quality degrades in symmetric corridors

### Nav2 Planner (`config/nav2_params.yaml`)

**SmacPlannerHybrid** for global planning:

```yaml
  GridBased:
    motion_model_for_search: "REEDS_SHEPP"
    minimum_turning_radius: 0.40
    reverse_penalty: 2.0
```

For the mecanum robot, consider switching to `"DUBIN"` or `"STATE_LATTICE"` if REEDS_SHEPP produces unnatural paths.

### Costmaps (`config/costmap_params.yaml`)

```yaml
  robot_radius: 0.25       # Must match your actual robot footprint
  inflation_radius: 0.55   # Clearance from obstacles
```

If the robot gets stuck near walls, reduce `inflation_radius`. If it collides with obstacles, increase it.

### Collision Monitor (`config/collision_monitor_params.yaml`)

Safety zones defined as polygons relative to `base_link`:

```yaml
  StopZone:
    polygon: [[-0.30, -0.30], [-0.30, 0.30], [0.30, 0.30], [0.30, -0.30]]
    # Tight bounding box: immediate stop if obstacle enters
  SlowdownZone:
    polygon: [[-0.50, -0.50], [-0.50, 0.50], [0.50, 0.50], [0.50, -0.50]]
    slowdown_ratio: 0.3    # Reduce speed to 30% within this zone
```

---

## Team Development

### ROS_DOMAIN_ID

When multiple developers run simulations on the same network, ROS 2 topics
collide. Each developer must use a unique `ROS_DOMAIN_ID`.

```bash
# In every terminal before running ROS commands:
export ROS_DOMAIN_ID=42

# Or add to ~/.bashrc:
echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
```

**Team convention:**

| Developer | ROS_DOMAIN_ID |
|-----------|--------------|
| Developer A | 42 |
| Developer B | 43 |
| Developer C | 44 |
| (etc.) | increment |

Only one developer at a time can use the same ID on the same network segment.

### Git workflow

```bash
# Before pushing, check for local changes that should not be committed:
git status                # Check tracked/untracked files
git diff                  # Review unstaged changes

# Common branch naming:
#   feature/<description>
#   fix/<description>
#   tuning/<parameter-name>
```

### Workspace isolation

Each developer should maintain their own `install/`, `build/`, and `log/`
directories (they are gitignored). Source the workspace overlay after building:

```bash
source install/setup.bash
```

If two developers share a machine, build in separate workspaces or use
separate `ROS_DOMAIN_ID` values.

---

## Sim-to-Real Migration

### Current state

All development runs in Gazebo Harmonic simulation.

### Migration steps

When moving to hardware, the following changes are required:

| Component | Simulation | Real robot |
|-----------|-----------|------------|
| use_sim_time | true | false |
| /odom | Gazebo MecanumDrive plugin | Robot odometry driver |
| /scan | Gazebo gpu_lidar plugin | Physical LiDAR driver |
| /cmd_vel | Gazebo plugin input | Motor controller input |
| TF | Gazebo publishes TF | robot_state_publisher + driver |
| Robot model | Same URDF | Same URDF (with real inertias) |

### Design rule

All topic names and frame IDs are identical between sim and real.
Switching requires only changing `use_sim_time:=true` to `use_sim_time:=false`
in the launch file or launch argument.

### Pre-migration checklist

- [ ] Verify all sensor topics match between sim and real
- [ ] Test AMCL localization with recorded bag data
- [ ] Tune MPPI velocity limits for hardware safety (start at 50% of sim speeds)
- [ ] Increase collision monitor zones for real-world safety margins
- [ ] Add E-stop handling to the launch stack
- [ ] Validate TF tree on hardware
