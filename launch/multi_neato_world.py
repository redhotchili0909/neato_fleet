#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import math

def generate_launch_description():

    pkg = get_package_share_directory('neato_fleet')
    gazebo_pkg = get_package_share_directory('gazebo_ros')

    world_path = os.path.join(pkg, 'models', 'worlds', 'simple_path_planning.world')

    sdf_path_red = os.path.join(
        pkg,
        'models',
        'neato',
        'neato_red.sdf'
    )
    sdf_path_green = os.path.join(
        pkg,
        'models',
        'neato',
        'neato_green.sdf'
    )
    sdf_path_blue = os.path.join(
        pkg,
        'models',
        'neato',
        'neato_blue.sdf'
    )
    
    models = [sdf_path_blue, sdf_path_red, sdf_path_green]

    num_robots = 3   # change to however many you want

    spawn_nodes = []
    for i in range(num_robots):
        name = f"robot{i+1}"

        # Spawn Neato
        spawn_nodes.append(
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', name,
                    '-robot_namespace', f'/{name}',
                    '-file', models[i],
                    '-x', str(i * 0.9144),
                    '-y', '0',
                    '-z', '0.05',
                    '-Y', str(math.pi / 2)
                ],
                output='screen'
            )
        )

        # Simulator adapter (adds TF, stable_scan, accel, bump)
        spawn_nodes.append(
            Node(
                package='neato_node2',
                executable='simulator_adapter',
                namespace=name,
                parameters=[{'tf_prefix': name}],
                output='screen'
            )
        )

        # Optional: scan_to_pc2 fix node
        spawn_nodes.append(
            Node(
                package='fix_scan',
                executable='scan_to_pc2',
                namespace=name,
                output='screen'
            )
        )

    return LaunchDescription([

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

        # Spawn robots + nodes
        *spawn_nodes,
    ])
