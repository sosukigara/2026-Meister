#!/bin/bash

# ==========================================
# ROS 2 & Gazebo Force Cleanup Script
# ==========================================

echo "Forcefully killing all ROS 2 and Gazebo processes..."

# 1. Gazebo 関連の停止
pkill -9 -f gz

# 2. ROS 2 ノード関連の停止
pkill -9 -f ros
pkill -9 -f rcl
pkill -9 -f rviz
pkill -9 -f teleop
pkill -9 -f slam
pkill -9 -f nav2
pkill -9 -f lifecycle_manager

# 3. ビルドプロセスの停止（念のため）
pkill -9 -f colcon

echo "Cleanup complete. All nodes have been killed."
