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
    models = [sdf_path_blue, sdf_path_blue, sdf_path_red, sdf_path_red]

    num_robots = 4   # change to however many you want

    spawn_nodes = []

    # Desired starting positions
    start_positions = [
        (-2, 0),   # robot1
        ( 2, 0),   # robot2
        ( 0,-2),   # robot3
        ( 0, 2)    # robot4
    ]
    angles = [str(0), str(math.pi), str(math.pi/2), str(math.pi * 3/2)]

    for i in range(num_robots):
        name = f"robot{i+1}"

        # Select the position (safe even if num_robots < 4)
        x, y = start_positions[i]

        # Spawn Neato
        spawn_nodes.append(
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', name,
                    '-robot_namespace', f'/{name}',
                    '-file', models[i],
                    '-x', str(x),
                    '-y', str(y),
                    '-z', '0.05',
                    '-Y', angles[i]
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

        # scan_to_pc2 fix node
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
