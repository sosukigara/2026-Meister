#!/usr/bin/env python3
"""
Send the robot through a sequence of waypoints using Nav2.
Uses nav2_simple_commander's goToPose for each waypoint sequentially.

Usage:
    python3 waypoint_follower.py
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math
import time


def create_pose(navigator, x, y, yaw):
    """Create a PoseStamped goal."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Wait for Nav2 to be active
    # Initial pose is set via nav2_params.yaml (set_initial_pose: true)
    navigator.waitUntilNav2Active(localizer='amcl')
    print('Nav2 is active! Starting waypoint navigation...')

    # Define waypoints — a tour of the maze
    # Routes through gaps in the internal walls to avoid getting stuck
    waypoints = [
        (-3.0, -1.5, 0.0,    'bottom-left area'),
        (-0.5, -0.5, 1.57,   'center'),
        (-3.5,  3.5, 3.14,   'top-left room'),
        (-0.5,  0.0, 0.0,    'back to center'),
        ( 3.0, -0.5, 0.0,    'right side gap'),
        ( 3.5,  1.0, 1.57,   'right corridor'),
        ( 3.5, -3.5, -1.57,  'bottom-right area'),
        (-0.5, -3.5, 3.14,   'bottom corridor'),
        (-4.0, -4.0, 0.0,    'start position'),
    ]

    for i, (x, y, yaw, label) in enumerate(waypoints):
        print(f'\n[{i+1}/{len(waypoints)}] Navigating to {label} ({x}, {y})...')
        goal = create_pose(navigator, x, y, yaw)
        navigator.goToPose(goal)

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                dist = feedback.distance_remaining
                print(f'  Distance remaining: {dist:.2f} m', end='\r')
            time.sleep(0.5)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f'  Reached {label}!')
        elif result == TaskResult.CANCELED:
            print(f'  Navigation to {label} was canceled.')
            break
        elif result == TaskResult.FAILED:
            print(f'  Failed to reach {label}, skipping...')
        print()

    print('\nWaypoint tour complete!')
    navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
