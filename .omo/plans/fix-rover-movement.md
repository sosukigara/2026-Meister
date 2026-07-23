# Plan: Fix Rover Movement — Revert to velocity + Solver Tuning

## Root Cause
`velocity_controller` is **inactive** (claimed interfaces: 0). Changing command interface from `velocity` to `effort` broke the controller activation.

## Changes

### 1. `src/umeonigiri/urdf/rover.xacro` — Revert drive joints to `velocity`

All 6 drive joints: change `effort` back to `velocity`:
```xml
<command_interface name="velocity"/>
```

### 2. `src/umeonigiri/config/controllers.yaml` — Revert controller config

Remove `command_interfaces: [effort]` and `pid: {...}` from velocity_controller. Restore to original:
```yaml
velocity_controller:
  type: velocity_controllers/JointGroupVelocityController
  ros__parameters:
    joints:
      - left_front_drive_joint
      - left_mid_drive_joint
      - left_rear_drive_joint
      - right_front_drive_joint
      - right_mid_drive_joint
      - right_rear_drive_joint
```

### 3. `src/umeonigiri/worlds/nav_world.sdf` — Add DART physics solver tuning

Change the `<physics>` block from:
```xml
<physics name="1ms" type="ignored">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
</physics>
```
To:
```xml
<physics name="dart" type="dart">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <dart>
    <solver>
      <solver_type>Dantzig</solver_type>
      <solver_iterations>200</solver_iterations>
    </solver>
  </dart>
</physics>
```

This increases the constraint solver iterations from default (~50) to 200, making velocity constraints enforced more strictly by the physics engine.

### 4. Keep existing friction changes (mu=3.0) — already applied

### 5. Optionally, increase `position_proportional_gain` to 1.0 for faster steering response

## Expected Outcome
- Controller activates correctly (velocity command interface)
- Physics solver enforces JointVelocityCmd more strictly with 200 iterations
- Wheel velocity: 80% → 95%+ of commanded
- Chassis angular velocity: 47% → 80%+ of commanded

## Build & Test
```bash
cd /home/so/Meistar
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select umeonigiri
source install/setup.bash
./start.sh
```
