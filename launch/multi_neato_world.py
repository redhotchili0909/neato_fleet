#!/usr/bin/env python3
import os
import yaml
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # 1. Get the config file path from the launch configuration
    config_file_path = LaunchConfiguration('config_file').perform(context)
    
    # 2. Parse the YAML file to get start positions
    start_positions = []
    try:
        with open(config_file_path, 'r') as f:
            params = yaml.safe_load(f)
            # Navigate to /rvo_fleet_controller -> ros__parameters -> start_positions
            start_positions = params['/rvo_fleet_controller']['ros__parameters']['start_positions']
    except Exception as e:
        print(f"Error reading config file: {e}")
        # Fallback default if yaml fails
        start_positions = [0.0, 0.0, 0.9144, 0.0, 1.8288, 0.0]

    # 3. Setup Models and Paths
    pkg = get_package_share_directory('neato_fleet')
    
    sdf_path_red = os.path.join(pkg, 'models', 'neato', 'neato_red.sdf')
    sdf_path_green = os.path.join(pkg, 'models', 'neato', 'neato_green.sdf')
    sdf_path_blue = os.path.join(pkg, 'models', 'neato', 'neato_blue.sdf')
    
    # Available models to cycle through
    model_files = [sdf_path_blue, sdf_path_red, sdf_path_green]

    # 4. Generate Nodes
    spawn_nodes = []
    
    # The YAML list is [x1, y1, x2, y2, ...], so we step by 2
    num_robots = len(start_positions) // 2
    
    for i in range(num_robots):
        name = f"robot{i+1}"
        
        # Extract x, y from the flat list
        pos_x = start_positions[i * 2]
        pos_y = start_positions[i * 2 + 1]
        
        # Cycle through colors (modulus operator handles cases where num_robots > 3)
        model_file = model_files[i % len(model_files)]

        # Spawn Neato Node
        spawn_nodes.append(
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', name,
                    '-robot_namespace', f'/{name}',
                    '-file', model_file,
                    '-x', str(pos_x),
                    '-y', str(pos_y),
                    '-z', '0.05',
                    '-Y', str(math.pi / 2) # Facing North
                ],
                output='screen'
            )
        )

        # Simulator adapter
        spawn_nodes.append(
            Node(
                package='neato_node2',
                executable='simulator_adapter',
                namespace=name,
                parameters=[{'tf_prefix': name}],
                output='screen'
            )
        )

        # Optional: scan_to_pc2
        spawn_nodes.append(
            Node(
                package='fix_scan',
                executable='scan_to_pc2',
                namespace=name,
                output='screen'
            )
        )

    return spawn_nodes

def generate_launch_description():
    pkg = get_package_share_directory('neato_fleet')
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    
    default_config = os.path.join(pkg, 'config', 'fleet_config.yaml')
    world_path = os.path.join(pkg, 'models', 'worlds', 'simple_path_planning.world')

    return LaunchDescription([
        # Declare the config_file argument so we can pass it in
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to the fleet config file containing start positions'
        ),

        # Gazebo server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_path}.items()
        ),

        # Gazebo client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg, 'launch', 'gzclient.launch.py')
            )
        ),

        # Use OpaqueFunction to run python logic (reading yaml) at runtime
        OpaqueFunction(function=launch_setup)
    ])