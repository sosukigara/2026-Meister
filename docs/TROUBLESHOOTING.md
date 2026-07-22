# Meister Troubleshooting Guide

Common issues, root causes, and solutions for the Meister ROS 2 project.

---

## Quick Reference

| Symptom | Likely cause | Quick fix |
|---------|-------------|-----------|
| Gazebo doesn't start | GPU acceleration not available | Add `headless:=true` to launch |
| colcon build fails | Missing dependencies | Run `rosdep install -i --from-path src` |
| TF_OLD_DATA warnings | Simulation time mismatch | Set `use_sim_time:=true` on all nodes |
| Robot doesn't move | No cmd_vel being published | Check `ros2 topic hz /cmd_vel` |
| Mecanum moves diagonally only | MPPI holonomic mode off | Set `holonomic: true` in mppi_params.yaml |
| Topic cross-talk between developers | Same ROS_DOMAIN_ID | Set unique ROS_DOMAIN_ID per person |
| LiDAR not showing in RViz | Wrong topic or frame | Check /scan topic and laser_link frame |
| SLAM map not building | Laser range params wrong | Adjust laser_min_range/max_range in slam_params.yaml |
| Robot gets stuck at walls | Inflation radius too large | Reduce inflation_radius in costmap_params.yaml |
| RViz shows no robot model | robot_description not published | Check URDF path and xacro output |
| Collision monitor stops robot spuriously | Safety zones too large | Shrink polygon zones in collision_monitor_params.yaml |
| Teleop twist not working | Wrong ROS_DOMAIN_ID or not sourced | Verify ROS_DOMAIN_ID and workspace source |

---

## Detailed Diagnoses

### Gazebo doesn't start

**Symptom:** Gazebo window does not appear, or the process exits immediately.

**Causes:**
- No GPU available (SSH session, headless server, VM without GPU passthrough)
- Missing Gazebo runtime libraries
- Display environment variable not set

**Solutions:**

1. **Immediate workaround** -- launch without GUI:
   ```bash
   ros2 launch meister_robot meister_slam_nav.launch.py headless:=true
   ```

2. **If running over SSH**, use `export DISPLAY=:0` or forward X11:
   ```bash
   ssh -X user@host
   export LIBGL_ALWAYS_SOFTWARE=1   # Force software rendering
   ```

3. **In a VM**, enable 3D acceleration in the VM settings or use `headless:=true`.

4. **Verify Gazebo is installed correctly:**
   ```bash
   gz sim --version
   ```

5. **Check GPU driver:**
   ```bash
   sudo apt install mesa-utils
   glxinfo | grep "OpenGL renderer"
   ```

---

### colcon build fails

**Symptom:** `colcon build` exits with CMake errors, missing package errors,
or "Could not find a package configuration file".

**Causes:**
- Missing system-level or rosdep-managed dependencies
- Symlink for rosnav package not created
- Wrong ROS distribution sourced

**Solutions:**

1. **Install all package dependencies:**
   ```bash
   source /opt/ros/jazzy/setup.bash
   rosdep update
   rosdep install -i --from-path src --rosdistro jazzy -y
   ```

2. **Verify rosnav symlink exists:**
   ```bash
   ls -la src/rosnav  # Should point to ../rosnav_submodule/src/diff_drive_robot-main
   ```
   If missing or broken:
   ```bash
   ln -sf ../rosnav_submodule/src/diff_drive_robot-main src/rosnav
   ```

3. **Install missing apt packages:**
   ```bash
   sudo apt install -y \
     ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
     ros-jazzy-xacro ros-jazzy-joint-state-publisher \
     ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
     ros-jazzy-navigation2 ros-jazzy-teleop-twist-keyboard \
     ros-jazzy-nav2-smac-planner
   ```

4. **Check ROS distribution:**
   ```bash
   echo $ROS_DISTRO  # Should print "jazzy"
   ```

5. **Clean rebuild:**
   ```bash
   rm -rf build/ install/ log/
   colcon build --symlink-install
   ```

---

### TF_OLD_DATA warnings

**Symptom:** Console repeatedly prints `TF_OLD_DATA ignoring transform from
old data` or similar TF warning messages.

**Causes:**
- Simulation time (`/clock`) is being published by Gazebo, but nodes use
  system wall clock instead of sim time
- Mixed time sources across nodes

**Solutions:**

1. **Ensure all nodes use sim time.** Pass the argument to the launch file:
   ```bash
   ros2 launch meister_robot meister_slam_nav.launch.py
   ```
   The launch file should propagate `use_sim_time:=true` automatically.
   Verify with:
   ```bash
   ros2 param get /robot_state_publisher use_sim_time
   ros2 param get /slam_toolbox use_sim_time
   ```

2. **Manually set parameter on any node** if needed:
   ```bash
   ros2 param set /node_name use_sim_time true
   ```

3. **For custom nodes**, in the source code:
   ```python
   node = rclpy.create_node('my_node')
   node.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
   ```

4. **Confirm /clock is being published:**
   ```bash
   ros2 topic hz /clock
   ```

---

### Robot doesn't move

**Symptom:** Teleop keystrokes produce no movement. Gazebo robot stays still.

**Causes:**
- No `/cmd_vel` messages being published
- Teleop node not running or on wrong ROS_DOMAIN_ID
- MPPI controller not receiving velocity commands
- Collision monitor blocking commands

**Solutions:**

1. **Check if cmd_vel is being published:**
   ```bash
   ros2 topic hz /cmd_vel
   ```
   If `0 Hz`, the teleop node is not running or not connected.

2. **Verify teleop node:**
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   Ensure the terminal running teleop has the same `ROS_DOMAIN_ID` and
   workspace source.

3. **Echo cmd_vel to see published values:**
   ```bash
   ros2 topic echo /cmd_vel
   ```

4. **Bypass collision monitor** (if it is stopping commands):
   Check `collision_monitor_params.yaml` for `StopZone` polygon size.
   Publish a test velocity directly to the topic the Gazebo plugin listens on:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
   ```

5. **Check if controller server is active:**
   ```bash
   ros2 node list | grep controller
   ros2 lifecycle get /controller_server  # Should be "active"
   ```

---

### Mecanum moves diagonally only

**Symptom:** Robot moves forward at an angle, cannot strafe sideways.
Strafe commands cause diagonal movement instead of pure lateral motion.

**Cause:** The MPPI controller has `holonomic: false` (default). For mecanum
drive, holonomic mode must be enabled so the controller generates velocity
commands with a `y` component.

**Solutions:**

1. **Check current value:**
   ```bash
   ros2 param get /controller_server/FollowPath holonomic
   ```

2. **Fix in `config/mppi_params.yaml`:**
   ```yaml
   FollowPath:
     holonomic: true
     motion_model: "Omni"
   ```

3. **Also check `config/nav2_params.yaml`** (overrides mppi_params.yaml if
   both are loaded). Ensure `holonomic` is `true` in both files.

4. **Restart the launch** after editing:
   ```bash
   ros2 launch meister_robot meister_slam_nav.launch.py
   ```

5. **Set runtime** (for testing without restart):
   ```bash
   ros2 param set /controller_server/FollowPath holonomic true
   ```

**Verification:** The robot should strafe left/right when pressing the
corresponding teleop keys (`u`/`o` left-shift, `j`/`l` right-shift with
standard teleop_twist_keyboard mappings).

---

### Topic cross-talk between developers

**Symptom:** Two developers running simulation on the same network see each
others robot data. RViz shows another developers map or laser scans.

**Cause:** ROS 2 uses DDS discovery. All nodes on the same network with the
same `ROS_DOMAIN_ID` (default 0) discover each other and share topics.

**Solutions:**

1. **Each developer sets a unique ROS_DOMAIN_ID:**
   ```bash
   export ROS_DOMAIN_ID=42   # Developer A
   export ROS_DOMAIN_ID=43   # Developer B
   export ROS_DOMAIN_ID=44   # Developer C
   ```

2. **Persist in ~/.bashrc:**
   ```bash
   echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
   ```

3. **Set per-terminal** if sharing a machine:
   ```bash
   export ROS_DOMAIN_ID=42 && ros2 launch meister_robot meister_slam_nav.launch.py
   ```

4. **Verify isolation:**
   ```bash
   ros2 topic list         # Should show only your own topics
   ros2 node list          # Should show only your own nodes
   ```

---

### LiDAR not showing in RViz

**Symptom:** RViz LaserScan display shows "No data" or "No transform"
between frames.

**Causes:**
- LiDAR topic (`/scan`) not being published
- Frame ID mismatch (Gazebo publishes as `laser_link`, RViz expects different)
- TF tree missing the `laser_link` frame

**Solutions:**

1. **Check if the scan topic exists:**
   ```bash
   ros2 topic list | grep scan
   ros2 topic hz /scan
   ```

2. **Inspect the message:**
   ```bash
   ros2 topic echo /scan --once | head -20
   ```
   Verify `frame_id` in the header. It should be `laser_link`.

3. **Check TF for laser_link:**
   ```bash
   ros2 run tf2_tools view_frames
   ```
   Open the generated `frames.pdf`. Verify `laser_link` exists under
   `base_link`.

4. **In RViz:**
   - Add a new LaserScan display
   - Set topic to `/scan`
   - Set frame to `laser_link`
   - Set size (m): 0.05
   - Set style: Points or Flat Squares
   - Check that "Transform" status is OK (green)

5. **If the URDF was modified**, verify the lidar frame exists:
   ```bash
   xacro src/meister_robot/urdf/robot.urdf.xacro | grep -i laser
   ```

---

### SLAM map not building

**Symptom:** RViz /map display shows empty or rarely updates. Frontier
explorer sends the robot to areas but no map appears.

**Causes:**
- Laser scan range parameters in SLAM Toolbox do not match the actual
  LiDAR sensor range
- SLAM node not receiving scan data
- SLAM parameters configured for a different sensor

**Solutions:**

1. **Check `laser_min_range` and `laser_max_range` in `config/slam_params.yaml`:**
   ```yaml
   slam_toolbox:
     ros__parameters:
       laser_min_range: 0.12
       laser_max_range: 12.0
       max_laser_range: 12.0
   ```
   `laser_min_range` must be below the LiDARs actual minimum range.
   `laser_max_range` and `max_laser_range` should match the LiDAR spec.

2. **Verify scan data reaches SLAM:**
   ```bash
   ros2 topic info /scan
   ros2 topic hz /scan
   ```

3. **Check SLAM node status:**
   ```bash
   ros2 node list | grep slam
   ros2 lifecycle get /slam_toolbox
   ```

4. **If using the gpu_lidar in Gazebo**, ensure the sensor range in
   `lidar.xacro` matches slam_params.yaml:
   ```xml
   <range>
     <min>0.3</min>
     <max>12.0</max>
   </range>
   ```
   Note: the physical LiDAR range and SLAM `laser_min_range` can differ.
   SLAM params should be _inside_ the sensor range:
   `sensor_min < slam_laser_min_range < slam_laser_max_range < sensor_max`

5. **Reduce minimum_travel_distance** if map updates are too infrequent:
   ```yaml
   minimum_travel_distance: 0.15
   minimum_travel_heading: 0.3
   ```

6. **Debug by saving a snapshot:**
   ```bash
   ros2 run nav2_map_server map_saver_cli -f test_map
   ```

---

## General Debugging Workflow

1. **Check ROS_DOMAIN_ID** -- most cross-talk and node discovery issues.
   ```bash
   echo $ROS_DOMAIN_ID
   ```

2. **Verify topic flow** -- start from sensor output and follow upstream.
   ```bash
   # LiDAR
   ros2 topic hz /scan
   # Odometry
   ros2 topic hz /odom
   # Command velocity
   ros2 topic hz /cmd_vel
   # TF
   ros2 run tf2_tools view_frames
   ```

3. **Check node lifecycle** -- Nav2 nodes use lifecycle management.
   ```bash
   ros2 lifecycle get /slam_toolbox
   ros2 lifecycle get /controller_server
   ros2 lifecycle get /planner_server
   ```
   Expected state: `active` (3). If not, a lifecycle transition may be needed.

4. **Review logs** -- colcon build and launch logs:
   ```bash
   less log/latest_build/           # Build output
   less ~/.ros/log/latest/          # ROS 2 runtime logs
   ```

5. **Isolate the problem** -- run components individually:
   ```bash
   # Robot model only
   xacro src/meister_robot/urdf/robot.urdf.xacro
   # SLAM only (no navigation)
   ros2 launch meister_robot meister_slam_nav.launch.py explore:=false
   # Navigation only with pre-built map
   ros2 launch meister_robot meister_slam_nav.launch.py slam:=false
   ```

6. **Record and replay** to debug without running Gazebo:
   ```bash
   # Record
   ros2 bag record -a -o problem_recording
   # Replay
   ros2 bag play problem_recording
   ```

---

## Still stuck?

If the table above does not cover your issue, check:

1. The rosnav upstream documentation in `rosnav_submodule/README.md`
2. `ros2 doctor` for system-level diagnostics
3. Open an issue with the full output of:
   ```bash
   ros2 doctor
   ros2 node list
   ros2 topic list
   ros2 param dump /controller_server > params_dump.txt
   ```
