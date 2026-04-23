#!/bin/bash

echo "Killing all ROS 2 and Gazebo processes..."

# ROS 2 nodes and launch files
pkill -f "ros2"
pkill -f "launch.py"
pkill -f "all_system.launch.py"
pkill -f "spawn.launch.py"

# Gazebo Sim (GZ Sim)
pkill -f "ruby /usr/bin/gz"
pkill -f "gz sim"
pkill -f "gz-sim-system"

# Micro-ROS or other related processes if any
pkill -f "micro_ros"

# Cleanup any remaining ROS 2 daemon
ros2 daemon stop

echo "Cleanup complete."
