# Fix: Robot not spawning in Gazebo (bot_description topic not received)

## TL;DR (For humans)

**Problem:** After launch, the `ros_gz_sim create` node subscribes to `robot_description` but never receives the message. Robot never appears in Gazebo → no topics flow → rviz blank.

**Root cause:** `robot_state_publisher` publishes `robot_description` with `TRANSIENT_LOCAL` durability. The `create` node spins up at the same time. When DDS discovery is slow (common on random domains with zombie participants), `create`'s subscriber defaults to `VOLATILE` durability before discovering the publisher — so it misses the already-published message and waits forever.

**Fix:** Wrap `spawn_robot` in a `TimerAction(period=3.0)` so `create` starts **after** robot_state_publisher has published + DDS discovery has completed. The subscriber will then adopt `TRANSIENT_LOCAL` compatibility and receive the message.

---

## Diagnosis

### Symptoms
- `create-*` log: `Waiting messages on topic [robot_description].` (repeats every 1s forever)
- `robot_state_publisher-1` log: `Robot initialized`
- No entity created in Gazebo
- No `/odom`, `/scan`, `/tf` topics published
- rviz shows empty "No data" on every panel
- lifecycle_manager waits forever for `controller_server/get_state`

### Root cause chain
1. `slam_nav.launch.py` starts `rsp` (robot_state_publisher) and `spawn_robot` (ros_gz_sim create) **concurrently** in the same action list
2. `rsp` parses URDF via xacro and publishes `robot_description` topic with QoS `TRANSIENT_LOCAL` durability
3. `spawn_robot` subscribes to `robot_description` topic
4. If DDS discovery completes **before** `create` subscribes → subscriber adopts publisher's `TRANSIENT_LOCAL` → receives published message → spawns robot ✅
5. If DDS discovery is **still in progress** when `create` subscribes → subscriber falls back to `VOLATILE` durability → misses the already-published message → waits forever ❌

### Why DDS discovery is slow
- Random domain IDs (`RANDOM % 200`) mean each launch is on a fresh domain
- Previously killed processes leave zombie participants that linger in the discovery database
- On a domain with zombie participants, the DomainParticipant discovery handshake takes longer
- During this extended discovery window, the subscriber starts with incompatible `VOLATILE` QoS

### Verification
The test with `ros2 launch` directly on domain 60 worked because DDS discovery happened to be fast on that domain at that time. Tests through `launch.sh` (which sets the ROS_DOMAIN_ID) sometimes hit slower domains.

---

## Plan

### Files to modify

**`src/diff_drive_robot/launch/slam_nav.launch.py`**

#### Change 1: Wrap `spawn_robot` in `TimerAction`
```
- spwan_robot = Node(
+ spawn_robot = TimerAction(
+     period=3.0,
+     actions=[Node(
          package='ros_gz_sim',
          executable='create',
          ...
-     )
+     )]
+ )
```

#### Change 2 (optional): Add `OnProcessStart` event handler
Alternative to TimerAction — start `create` only after `robot_state_publisher` process has started:
```python
RegisterEventHandler(
    OnProcessStart(
        target_action=rsp,
        on_start=[spawn_robot],
    )
)
```
But `TimerAction` is simpler and more reliable.

### Verification
1. `colcon build --symlink-install --packages-select diff_drive_robot`
2. `source install/setup.bash`
3. `export GZ_IP=127.0.0.1`
4. `export ROS_DOMAIN_ID=$((RANDOM % 200 + 10))`
5. `ros2 launch diff_drive_robot slam_nav.launch.py world_name:=maze explore:=false headless:=true`
6. Wait 15s — robot should appear in Gazebo, topics should flow, rviz should show data

### Must-NOT-Have
- Do NOT change the robot_state_publisher QoS (it's correct as TRANSIENT_LOCAL)
- Do NOT change the `ros_gz_sim create` package or its API
- Do NOT add sleep() calls in Python code — use `TimerAction` in the launch file

---

## Todos

- [x] ### `src/diff_drive_robot/launch/slam_nav.launch.py`: Wrap spawn_robot in 3s TimerAction + use -string instead of -topic
- **Where:** `_build_runtime_actions()`, lines 115-135
- **How:** (1) Import subprocess, run xacro to generate URDF string. (2) Replace `-topic robot_description` with `-string <xml>`. (3) Wrap in `TimerAction(period=3.0)`.
- **Why:** `-topic` depends on DDS for message delivery; QoS mismatch on slow-discovery domains causes create to never receive robot_description. `-string` bypasses DDS entirely.
- **Acceptance:** After build, create node starts 3s after launch, receives robot_description as string argument, robot spawns in Gazebo. Log shows `Entity creation successful`.
- **QA:** `ros2 launch` shows `Entity creation successful.` then lifecycle_manager creates costmaps, no `Waiting messages on topic` errors.
- **Commit:** `fix: use -string instead of -topic for ros_gz_sim create to avoid DDS QoS issue`
