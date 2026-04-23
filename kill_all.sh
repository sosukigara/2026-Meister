#!/bin/bash

# ==========================================
# ROS 2 & Gazebo Force Cleanup Script
# ==========================================

echo "Forcefully killing all ROS 2 and Gazebo processes..."

# 1. Gazebo 関連の停止
pkill -9 -f gz-sim
pkill -9 -f gz-gui

# 2. ROS 2 ノード関連の停止
pkill -9 -f parameter_bridge
pkill -9 -f robot_state_publisher
pkill -9 -f joint_state_publisher
pkill -9 -f controller_server
pkill -9 -f planner_server
pkill -9 -f behavior_server
pkill -9 -f bt_navigator
pkill -9 -f waypoint_follower
pkill -9 -f velocity_smoother
pkill -9 -f collision_monitor
pkill -9 -f lifecycle_manager
pkill -9 -f slam_toolbox
pkill -9 -f rviz2
pkill -9 -f teleop_twist_keyboard


echo "Cleanup complete. All nodes have been killed."
