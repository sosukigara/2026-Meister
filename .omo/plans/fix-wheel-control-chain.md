# Fix: Robot receives nav goal but does not move

## Problem
Sending a 2D Nav Goal in RViz generates a planned path but the robot does **not** move. The odometry updates (position changes in RViz), indicating the control chain is broken between the wheel controller node and actual wheel actuation in Gazebo.

## Root Cause Analysis

### Issue 1: cmd_vel bridge conflicts with ros2_control
The `simulation.launch.py` starts a `parameter_bridge` that subscribes to `/cmd_vel` (ROS) and publishes to `/model/rover/cmd_vel` (Gazebo). This is a **relic from the old Gazebo-native architecture** (before ros2_control was merged). With ros2_control, `/cmd_vel` should be handled **only** by `wheel_controller_node` — the bridge is unnecessary and creates a competing subscriber that may cause QoS mismatch or DDS interference.

**Fix:** Remove the cmd_vel bridge entirely.

### Issue 2: No debug output to verify the control chain
`wheel_controller_node` has zero logging after startup. We cannot tell whether:
- `/cmd_vel` messages from Nav2 are being received
- The control loop is computing wheel speeds
- Commands are being published to `/velocity_controller/commands`

**Fix:** Add `RCLCPP_INFO` logging for received cmd_vel values and published command values.

## Plan Steps

### Step 1: Remove cmd_vel bridge
**File:** `src/umeonigiri/launch/simulation.launch.py`
**Action:** Delete lines 94-104 (the `# Bridge: cmd_vel` block):
```python
        # Bridge: cmd_vel (ROS → Gazebo, for external control / teleop)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/rover/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            ],
            remappings=[('/model/rover/cmd_vel', '/cmd_vel')],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
```

### Step 2: Add debug logging to wheel_controller_node
**File:** `src/umeonigiri/src/wheel_controller_node.cpp`
**Actions:**
1. In `cmdVelCallback`: add `RCLCPP_INFO` to print received velocity command
2. In `controlLoop`: add `RCLCPP_INFO` to print the first published wheel speed and steer angle
3. Use throttled logging (e.g., `RCLCPP_INFO_THROTTLE`) to avoid flooding the console (once per second)

**Key code changes:**
```cpp
void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    linear_x_ = msg->linear.x;
    angular_z_ = msg->angular.z;
    RCLCPP_INFO(this->get_logger(), "cmd_vel received: linear=%.3f angular=%.3f", msg->linear.x, msg->angular.z);
}

void controlLoop()
{
    // ... existing computation ...
    
    auto vel_msg = std_msgs::msg::Float64MultiArray();
    vel_msg.data = wheel_speeds;
    vel_pub_->publish(vel_msg);
    auto steer_msg = std_msgs::msg::Float64MultiArray();
    steer_msg.data = steer_angles;
    steer_pub_->publish(steer_msg);
    
    // Log first wheel command as a heartbeat (once per second)
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "control: v=%.3f omega=%.3f → wheel[0].speed=%.3f steer[0]=%.3f",
        v, omega, wheel_speeds[0], steer_angles[0]);
}
```

### Step 3: Build and push
```bash
cd /home/so/Meistar
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select umeonigiri
source install/setup.bash
```

Then commit and push to `main`:
```bash
git add -A
git commit -m "fix: remove stale cmd_vel bridge, add debug logging to wheel_controller_node"
git push origin main
```

### Step 4: User verification
1. Run `./stop.sh && ./start.sh`
2. In RViz, send a 2D Nav Goal
3. Observe console output from `wheel_controller_node`:
   - If `cmd_vel received: linear=X angular=Y` appears → Nav2 is working
   - If `control: v=X omega=Y → wheel[0].speed=Z steer[0]=W` appears → control computation works
   - If robot still doesn't move → issue is in ros2_control → Gazebo joint actuation layer

## Acceptance Criteria
- [ ] Sending 2D Nav Goal shows path **and** robot moves physically in Gazebo
- [ ] Debug logs confirm cmd_vel reception and wheel command publication
- [ ] No stale cmd_vel bridge running
