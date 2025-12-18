#!/usr/bin/env python3
"""
Spawn Robot Launch File - Module 2, Chapter 5
Purpose: Launch Gazebo and spawn a humanoid robot
Usage: ros2 launch spawn_robot.launch.py
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Launch arguments
    world_file = LaunchConfiguration('world')
    robot_file = LaunchConfiguration('robot')
    robot_name = LaunchConfiguration('name')

    # Default file paths
    pkg_dir = os.path.dirname(os.path.realpath(__file__))
    default_world = os.path.join(pkg_dir, '..', 'sdf', 'humanoid_world.sdf')
    default_robot = os.path.join(pkg_dir, '..', 'urdf', 'example_humanoid.urdf')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Path to world SDF file'
        ),
        DeclareLaunchArgument(
            'robot',
            default_value=default_robot,
            description='Path to robot URDF file'
        ),
        DeclareLaunchArgument(
            'name',
            default_value='example_humanoid',
            description='Robot name in simulation'
        ),

        # Start Gazebo with specified world
        ExecuteProcess(
            cmd=['gz', 'sim', world_file, '-r'],
            output='screen',
            shell=False
        ),

        # Spawn robot (after Gazebo starts)
        # Note: In production, add delay or wait for Gazebo ready
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', robot_name,
                '-file', robot_file,
                '-x', '0',
                '-y', '0',
                '-z', '1.0'  # Spawn 1m above ground
            ],
            output='screen'
        ),
    ])
