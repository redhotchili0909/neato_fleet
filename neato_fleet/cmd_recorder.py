#!/usr/bin/env python3
"""
Command Recorder for RVO Fleet - Records cmd_vel for Trajectory Playback
Records cmd_vel commands from simulation runs so they can be played back on Neatos.

Usage:
  # Run in simulation with RVO controller, this node records the commands
  ros2 run neato_fleet cmd_recorder --ros-args -p num_robots:=3 -p output_file:=my_trajectory.csv
  
  # The trajectory file can then be used with cmd_player.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import csv
import os
from datetime import datetime
from dataclasses import dataclass
import json



@dataclass
class RobotCommandRecord:
    timestamp: float
    linear_x: float
    angular_z: float


class CmdRecorder(Node):
    """Records cmd_vel commands for all robots during a simulation run"""

    def __init__(self):
        super().__init__('cmd_recorder')
        
        self.declare_parameter('num_robots', 3)
        
        self.num_robots = self.get_parameter('num_robots').value
        
        self.output_dir = os.getcwd()

        os.makedirs(self.output_dir, exist_ok=True)
        
        # generate output filename
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_file = f'cmd_trajectory_{timestamp}.csv'
        
        self.output_path = os.path.join(self.output_dir, 'trajectories', output_file)
        
        self.recording = False
        self.start_time = None
        self.trajectories = [[] for _ in range(self.num_robots)]
        self.last_commands = [Twist() for _ in range(self.num_robots)]
        
        # Subscribe to cmd_vel for each robot
        self.cmd_subs = []
        for i in range(1, self.num_robots + 1):
            sub = self.create_subscription(
                Twist,
                f'/robot{i}/cmd_vel',
                lambda msg, idx=i-1: self.cmd_callback(msg, idx),
                10
            )
            self.cmd_subs.append(sub)
        
        # subscribe to debug topic to know when goals are received
        self.debug_sub = self.create_subscription(
            String,
            '/rvo_debug',
            self.debug_callback,
            10
        )
        
        # recording timer
        self.record_timer = self.create_timer(0.1, self.record)
        
        self.goals_received = False
        self.frames_at_zero = 0  # count frames where all robots stopped
        
        self.get_logger().info(f' Recorder initialized')
        self.get_logger().info(f'Output saved to: {self.output_path}')
        self.get_logger().info(f'Waiting for goal to be set...')

    def cmd_callback(self,msg,robot_idx):
        """Store latest command for each robot"""
        self.last_commands[robot_idx] = msg

    def debug_callback(self,msg):
        """Check if goals have been received from controller"""
        try:
            data = json.loads(msg.data)
            if data.get('goals_received', False) and not self.goals_received:
                self.goals_received = True
                self.recording = True
                self.start_time = self.get_clock().now().nanoseconds / 1e9
                self.get_logger().info('RECORDING STARTED')
        except json.JSONDecodeError:
            pass

    def record(self):
        """Record commands at each timer tick"""
        if not self.recording:
            return
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        relative_time = current_time - self.start_time
        
        all_stopped = True
        for i, cmd in enumerate(self.last_commands):
            record = RobotCommandRecord(
                timestamp=relative_time,
                linear_x=cmd.linear.x,
                angular_z=cmd.angular.z
            )
            self.trajectories[i].append(record)
            
            # check if robot is moving
            if abs(cmd.linear.x) > 0.001 or abs(cmd.angular.z) > 0.001:
                all_stopped = False
        
        # detect when all robots have stopped (reached goal)
        if all_stopped and relative_time > 1.0:
            self.frames_at_zero += 1
            if self.frames_at_zero >= 30:  # stop recording if 3 seconds of no movement
                self.stop_recording()
        else:
            self.frames_at_zero = 0

    def stop_recording(self):
        """Stop recording and save trajectory"""
        if not self.recording:
            return
        
        self.recording = False
        self.get_logger().info('RECORDING STOPPED')
        self.save_trajectory()

    def save_trajectory(self):
        """Save recorded trajectory to CSV file"""

        max_len = max(len(traj) for traj in self.trajectories)
        
        with open(self.output_path, 'w', newline='') as f:
            writer = csv.writer(f)
            
            header = ['timestamp']
            for i in range(self.num_robots):
                header.extend([f'robot{i+1}_linear_x', f'robot{i+1}_angular_z'])
            writer.writerow(header)
            
            for frame_idx in range(max_len):
                row = []
                
                timestamp = 0.0
                for traj in self.trajectories:
                    if frame_idx < len(traj):
                        timestamp = traj[frame_idx].timestamp
                        break
                row.append(f'{timestamp:.4f}')
                
                # add each robot's command
                for i in range(self.num_robots):
                    if frame_idx < len(self.trajectories[i]):
                        record = self.trajectories[i][frame_idx]
                        row.extend([f'{record.linear_x:.6f}', f'{record.angular_z:.6f}'])
                    else:
                        row.extend(['0.0', '0.0'])
                
                writer.writerow(row)
        
        self.get_logger().info(f'Trajectory saved to: {self.output_path}')

    def destroy_node(self):
        """Save trajectory on shutdown"""
        if self.recording:
            self.stop_recording()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdRecorder()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted - saving trajectory...')
        node.stop_recording()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
