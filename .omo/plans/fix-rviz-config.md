# Fix: rviz config for single-robot mode

## TL;DR (For humans)

**Problem:** rviz shows nothing despite Gazebo running with the robot visible. Two causes:
1. rviz is on a different ROS_DOMAIN_ID than the launch system
2. rviz config (`bot.rviz`) uses multi-robot topic names (`/robot1/scan`, `/robot1/robot_description`, `/robot2/initialpose`)

**Fix:** (a) Update rviz config for single-robot names, (b) `launch.sh` prints the chosen domain so rviz can match it.

---

## Diagnosis

| Check | Current | Should be |
|---|---|---|
| LaserScan topic | `/robot1/scan` | `/scan` (after laser_filter) |
| RobotModel topic | `/robot1/robot_description` | `/robot_description` |
| Robot2 Model | present (robot2 doesn't exist) | removed |
| Fixed Frame | `map` | `odom` (SLAM starts later) |
| SetInitialPose | `/robot2/initialpose` | `/initialpose` |
| Domain display | not printed | print in launch.sh banner |

---

## Todos

- [x] ### `src/diff_drive_robot/rviz/bot.rviz`: Fix topic names for single robot
- `Value: /robot1/scan` → `Value: /scan` ✅
- `Value: /robot1/robot_description` → `Value: /robot_description` ✅
- Remove RobotModel block for Robot2 ✅
- `Fixed Frame: map` → `Fixed Frame: odom` ✅
- `Value: /robot2/initialpose` → `Value: /initialpose` ✅
- **QA:** Run `ros2 launch` then `rviz2 -d <config>` with matching domain → robot model + laser scan visible

- [x] ### `launch.sh`: Print ROS_DOMAIN_ID in banner
- Add `echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"` to the banner section ✅
- **QA:** `bash launch.sh maze` banner shows `ROS_DOMAIN_ID: <number>`
