#!/usr/bin/env python3
"""
Launch file for RVO2 Fleet Control

This launches:
1. Gazebo with multiple Neatos
2. RVO Fleet Controller
3. Fleet Scenarios
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('neato_fleet')
    
    config_file = os.path.join(pkg, 'config', 'fleet_config.yaml')
    
    # launch arguments
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='3',
        description='Number of robots in the fleet'
    )
    
    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value='swap',
        description='Fleet scenario to run: swap, cross, home')
    
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='0.5',
        description='Maximum robot speed in m/s'
    )
    
    multi_neato_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'multi_neato_world.py')
        )
    )
    
    # RVO fleet controller node
    rvo_controller = Node(
        package='neato_fleet',
        executable='rvo_fleet_controller',
        name='rvo_fleet_controller',
        output='screen',
        parameters=[config_file]
    )
    
    # fleet goals node
    fleet_goals = Node(
        package='neato_fleet',
        executable='fleet_goals',
        name='fleet_goals',
        output='screen',
        parameters=[config_file]
    )
    
    recorder = Node(
        package='neato_fleet',
        executable='cmd_recorder',
        name='cmd_vel_recorder',
        output='screen',
        parameters=[{'num_robots': 3}]
        
    )
    
    return LaunchDescription([
        num_robots_arg,
        scenario_arg,
        max_speed_arg,
        multi_neato_launch,
        rvo_controller,
        fleet_goals,
    ])
