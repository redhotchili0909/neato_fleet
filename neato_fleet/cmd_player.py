#!/usr/bin/env python3
"""
Command Player for RVO Fleet - Plays back recorded cmd_vel on Real Robots

Plays back pre-recorded cmd_vel commands from simulation runs on real hardware.
This bypasses the need for odometry feedback, which can be noisy on real Neatos.

Usage:
  # Play back a recorded trajectory on real robots
  ros2 run neato_fleet cmd_player --ros-args -p trajectory_file:=cmd_trajectory_20251207_120000.csv
  
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import csv
import os
from typing import List, Tuple
from dataclasses import dataclass


@dataclass
class CommandFrame:
    timestamp: float
    commands: List[Tuple[float, float]]  # [(linear_x, angular_z), ...]


class CmdPlayer(Node):
    """Plays back recorded cmd_vel trajectories on real robots"""

    def __init__(self):
        super().__init__('cmd_player')
        
        # Parameters
        self.declare_parameter('num_robots', 3)
        self.declare_parameter('trajectory_file', '')
        
        self.num_robots = self.get_parameter('num_robots').value
        trajectory_file = self.get_parameter('trajectory_file').value
        
        if not trajectory_file:
            self.get_logger().error('No trajectory_file specified!')
        
        base_dir = os.path.expanduser('~/ros2_ws/src/comp-robo/neato_fleet/trajectories')
        self.trajectory_path = os.path.join(base_dir, trajectory_file)
        
        if not os.path.exists(self.trajectory_path):
            self.get_logger().error(f'Trajectory file not found: {self.trajectory_path}')
        
        self.frames = []
        self.load_trajectory()
        
        # publishers for each robot
        self.vel_pubs = []
        for i in range(1, self.num_robots + 1):
            pub = self.create_publisher(Twist, f'/robot{i}/cmd_vel', 20)
            self.vel_pubs.append(pub)
        
        # playback state
        self.playing = False
        self.start_time = None
        self.current_frame = 0
        self.finished = False
        
        # playback timer
        self.playback_timer = self.create_timer(0.01, self.playback_tick)
        
        self.start_playback()

    def load_trajectory(self):
        """Load trajectory from CSV file"""
        with open(self.trajectory_path, 'r') as f:
            reader = csv.reader(f)
            header = next(reader)
            
            for row in reader:
                if not row:
                    continue
                    
                timestamp = float(row[0])
                commands = []
                
                for i in range(self.num_robots):
                    col_idx = 1 + i * 2
                    if col_idx + 1 < len(row):
                        linear_x = float(row[col_idx])
                        angular_z = float(row[col_idx + 1])
                        commands.append((linear_x, angular_z))
                    else:
                        commands.append((0.0, 0.0))
                
                self.frames.append(CommandFrame(timestamp=timestamp, commands=commands))
        
        self.get_logger().info(f'Loaded trajectory')


    def start_playback(self):
        """Start playing back the trajectory"""
        self.playing = True
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.current_frame = 0
        self.last_log_time = 0.0
        self.get_logger().info('▶PLAYBACK STARTED')

    def playback_tick(self):
        """Main playback loop"""
        if not self.playing or self.finished:
            return
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = (current_time - self.start_time)
        
        while (self.current_frame < len(self.frames) - 1 and 
               self.frames[self.current_frame + 1].timestamp <= elapsed):
            self.current_frame += 1
        
        # check if we're done
        if self.current_frame >= len(self.frames):
            if not self.finished:
                self.finish_playback()
            return
        
        frame = self.frames[self.current_frame]
        
        for i in range(self.num_robots):
            cmd = Twist()
            cmd.linear.x = frame.commands[i][0]
            cmd.angular.z = frame.commands[i][1]
            self.vel_pubs[i].publish(cmd)
        

    def finish_playback(self):
        """Handle end of playback"""
        self.finished = True
        self.playing = False
        
        # send stop commands
        stop_cmd = Twist()
        for pub in self.vel_pubs:
            pub.publish(stop_cmd)
        
        self.create_timer(1.0, lambda: rclpy.shutdown())

    def destroy_node(self):
        """Send stop commands on shutdown"""
        stop_cmd = Twist()
        for pub in self.vel_pubs:
            for _ in range(5):
                pub.publish(stop_cmd)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CmdPlayer()
        rclpy.spin(node)
    except (ValueError, FileNotFoundError) as e:
        print(f'Error: {e}')
    except KeyboardInterrupt:
        print('Interrupted - stopping robots...')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
