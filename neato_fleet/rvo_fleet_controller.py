#!/usr/bin/env python3
"""
RVO2 Fleet Controller for Neato Robots

This node integrates the Python-RVO2 library with ROS2 to provide
collision-free multi-robot navigation for Neato robots.

- Subscribes to /robot{i}/odom for each robot's position
- Uses RVO2 to compute collision-free velocities
- Publishes to /robot{i}/cmd_vel for each robot

The RVO2 library handles the collision avoidance math, we just need to:
1. Feed it current positions and preferred velocities (toward goals)
2. Get back safe velocities that avoid collisions
3. Convert those to Twist commands for each Neato
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, String
import rvo2
import math
import json
from typing import List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class RobotState:
    """Holds the state of a single robot"""
    position: Tuple[float, float] = (0.0, 0.0)
    velocity: Tuple[float, float] = (0.0, 0.0)
    yaw: float = 0.0
    goal: Tuple[float, float] = (0.0, 0.0)
    has_odom: bool = False


class RVOFleetController(Node):
    """
    Controller for multi-Neato navigation using RVO2.
    """

    def __init__(self):
        super().__init__('rvo_fleet_controller')

        # Declare parameters
        self.declare_parameter('num_robots', 3)
        self.declare_parameter('max_speed', 0.2)  # m/s
        self.declare_parameter('max_angular_speed', 1.5)  # rad/s
        self.declare_parameter('neighbor_dist', 10.0)  # how far in meters to look for other agents
        self.declare_parameter('max_neighbors', 10)   # max number of other agents to consider
        self.declare_parameter('time_horizon', 5.0)  # how far ahead in seconds to plan for other agents
        self.declare_parameter('time_horizon_obst', 2.0)  # how far ahead in seconds to plan for obstacles
        self.declare_parameter('robot_radius', 0.33)  # physical radius of robot in meters
        self.declare_parameter('goal_tolerance', 0.1)  # how close to the goal the agent is
        self.declare_parameter('control_rate', 10.0)  # hz
        # Start positions: flat list [x1,y1, x2,y2, x3,y3] - default (0,0), (0.9114,0), (1.8228,0)
        # self.declare_parameter('start_positions', [0.0, 0.0, 0.9114, 0.0, 1.8228, 0.0])
        self.declare_parameter('start_positions', [0.0])
        self.declare_parameter('obstacle_positions', [0.0])

        # Get parameters
        self.num_robots = self.get_parameter('num_robots').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.neighbor_dist = self.get_parameter('neighbor_dist').value
        self.max_neighbors = self.get_parameter('max_neighbors').value
        self.time_horizon = self.get_parameter('time_horizon').value
        self.time_horizon_obst = self.get_parameter('time_horizon_obst').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.control_rate = self.get_parameter('control_rate').value
        
        # Parse start positions - if empty, use default spacing
        start_pos_flat = self.get_parameter('start_positions').value
        self.start_positions = []
        if start_pos_flat and len(start_pos_flat) >= 2:
            # Parse flat list [x1,y1, x2,y2, ...] into tuples
            for i in range(0, len(start_pos_flat), 2):
                if i + 1 < len(start_pos_flat):
                    self.start_positions.append((start_pos_flat[i], start_pos_flat[i + 1]))

        # If not enough positions provided, use defaults
        while len(self.start_positions) < self.num_robots:
            idx = len(self.start_positions)
            self.start_positions.append((idx * 2.0, 0.0))  # Default: (0,0), (2,0), (4,0), ...
                    
        obstacle_pos_flat = self.get_parameter('obstacle_positions').value
        self.obstacle_positions = []
        
        obst_size = 0.34576
        half_size = obst_size / 2.0
        
        if obstacle_pos_flat and len(obstacle_pos_flat) >= 2:
            for i in range(0, len(obstacle_pos_flat), 2):
                if i + 1 < len(obstacle_pos_flat):
                    cx = obstacle_pos_flat[i]
                    cy = obstacle_pos_flat[i + 1]
                    
                    corners = [
                        (cx + half_size, cy + half_size),
                        (cx - half_size, cy + half_size),
                        (cx - half_size, cy - half_size),
                        (cx + half_size, cy - half_size)
                    ]
                    
                    self.obstacle_positions.append(corners)
        

        # initialize RVO2 simulator
        self.sim = rvo2.PyRVOSimulator(
            1.0 / self.control_rate,
            self.neighbor_dist,
            self.max_neighbors,
            self.time_horizon,
            self.time_horizon_obst,
            self.robot_radius,
            self.max_speed
        )
        
        for obstacle_pos in self.obstacle_positions:
            self.sim.addObstacle(obstacle_pos)
            
        self.sim.processObstacles()


        # robot states and agent lists
        self.robot_states: List[RobotState] = []
        self.agent_ids: List[int] = []
        self.goals_received = False  # Wait for goals before moving

        # initialize robots
        for i in range(self.num_robots):
            start_pos = self.start_positions[i]
            # add agent to RVO sim
            agent_id = self.sim.addAgent(start_pos)
            self.agent_ids.append(agent_id)
            
            # initialize robot state with no set goal destination  + will wait for fleet commands
            state = RobotState(
                position=start_pos,
                goal=start_pos  # start position (no moving yet)
            )
            self.robot_states.append(state)
            self.get_logger().info(f'Robot {i+1} start position: ({start_pos[0]:.2f}, {start_pos[1]:.2f})')

        self.odom_subs = []
        self.vel_pubs = []
        
        for i in range(1, self.num_robots + 1):
            # subscribe to odometry for each robot
            sub = self.create_subscription(
                Odometry,
                f'/robot{i}/odom',
                lambda msg, idx=i-1: self.odom_callback(msg, idx),
                10
            )
            self.odom_subs.append(sub)
            
            # publisher for velocity commands
            pub = self.create_publisher(Twist, f'/robot{i}/cmd_vel', 10)
            self.vel_pubs.append(pub)

        # goal subscription
        self.goal_sub = self.create_subscription(
            MarkerArray,
            '/fleet_goals',
            self.goals_callback,
            10
        )

        # control loop timer
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop
        )
        
        # debug publisher for cmd_recorder integration
        self.debug_pub = self.create_publisher(String, '/rvo_debug', 10)
        self.debug_timer = self.create_timer(0.5, self.publish_debug_state)

        self.get_logger().info(
            f'RVO Fleet Controller initialized with {self.num_robots} robots'
        )

    def publish_debug_state(self):
        """Publish internal state for cmd_recorder to know when goals are received"""
        debug_data = {
            'goals_received': self.goals_received,
            'robots': []
        }
        for i, state in enumerate(self.robot_states):
            robot_data = {
                'id': i + 1,
                'has_odom': state.has_odom,
                'position': list(state.position),
                'yaw_deg': math.degrees(state.yaw),
                'goal': list(state.goal),
            }
            debug_data['robots'].append(robot_data)
        
        msg = String()
        msg.data = json.dumps(debug_data)
        self.debug_pub.publish(msg)

    def odom_callback(self, msg, robot_idx):
        """Update robot state from odometry message"""
        state = self.robot_states[robot_idx]
        
        # extract position
        state.position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        
        # extract velocity
        state.velocity = (
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y
        )
        
        # extract yaw (we only need yaw for 2d space)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0*(q.w*q.z+q.x*q.y)
        cosy_cosp = 1.0-2.0*(q.y*q.y+q.z*q.z)
        state.yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # On first odom, set goal to current position (stay in place until goals received)
        if not state.has_odom:
            state.goal = state.position
            self.get_logger().info(
                f'Robot {robot_idx+1} odom received at ({state.position[0]:.2f}, {state.position[1]:.2f})'
            )
        
        state.has_odom = True


    def set_goals(self, goals):
        """Set goals for all robots"""
        for i, goal in enumerate(goals):
            if i < self.num_robots:
                self.robot_states[i].goal = goal
                self.get_logger().info(f'Robot {i} goal set to {goal}')

    def goals_callback(self, msg: MarkerArray):
        """Update goals from marker array (from fleet_goals node)"""
        for marker in msg.markers:
            if marker.id < self.num_robots:
                self.robot_states[marker.id].goal = (
                    marker.pose.position.x,
                    marker.pose.position.y
                )
                if not self.goals_received:
                    self.get_logger().info(
                        f'Robot {marker.id} goal updated to '
                        f'({marker.pose.position.x:.2f}, {marker.pose.position.y:.2f})'
                    )
        self.goals_received = True

    def control_loop(self):
        """Main control loop"""
        
        # check if we have odom for all robots
        if not all(state.has_odom for state in self.robot_states):
            status = ', '.join([
                f'robot{i+1}: {"yes" if state.has_odom else "no"}'
                for i, state in enumerate(self.robot_states)
            ])
            self.get_logger().warn(f'Waiting for odometry... [{status}]', throttle_duration_sec=2.0)
            return

        # update RVO simulator with current positions and preferred velocities
        for i, state in enumerate(self.robot_states):
            agent_id = self.agent_ids[i]
            
            # calculate preferred velocity toward goal
            dx = state.goal[0] - state.position[0]
            dy = state.goal[1] - state.position[1]
            dist = math.sqrt(dx * dx + dy * dy)
            
            if dist > self.goal_tolerance:
                # normalize and scale to max speed
                pref_vel = (
                    (dx / dist) * self.max_speed,
                    (dy / dist) * self.max_speed
                )
            else:
                # stop at goal
                pref_vel = (0.0, 0.0)
                self.get_logger().info(
                    f'Robot {i} reached goal!',
                    throttle_duration_sec=5.0
                )
            
            self.sim.setAgentPrefVelocity(agent_id, pref_vel)

        # step the RVO simulation
        self.sim.doStep()

        for i, state in enumerate(self.robot_states):
            agent_id = self.agent_ids[i]
            
            # get the collision-free velocity from RVO
            rvo_vel = self.sim.getAgentVelocity(agent_id)
            
            # convert global velocity to Neato Twist command
            cmd_vel = self.convert_to_twist(rvo_vel[0], rvo_vel[1], state.yaw)
            
            # publish velocity command
            self.vel_pubs[i].publish(cmd_vel)

    def convert_to_twist(self, vx_global, vy_global, yaw):
        """
        Convert global frame velocity to Neato Twist command.
        
        Basically, (vx, vy) in global frame to (linear.x, angular.z) in robot frame.
        """
        cmd = Twist()
        
        # calculate desired heading from velocity
        if abs(vx_global) > 0.01 or abs(vy_global) > 0.01:
            desired_yaw = math.atan2(vy_global, vx_global)
            yaw_error = self.normalize_angle(desired_yaw - yaw)
            
            # if yaw error is large, reduce forward speed
            # if abs(yaw_error) > math.pi / 4:  # > 45 degrees
            #     cmd.linear.x = 0.05
            #     cmd.angular.z = 1.0 * yaw_error  # turn toward goal
            # else:
            # move forward and correct heading
            speed = math.sqrt(vx_global**2 + vy_global**2)
            cmd.linear.x = speed
            cmd.angular.z = 2.0 * yaw_error  # proportional error correction (if too reactive, adjust)
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        
        # # set Twist limits
        # cmd.linear.x = max(min(cmd.linear.x, self.max_speed), -self.max_speed)
        # cmd.angular.z = max(min(cmd.angular.z, self.max_angular_speed), -self.max_angular_speed)
        
        return cmd

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle



def main(args=None):
    rclpy.init(args=args)
    node = RVOFleetController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # stop all robots
        for pub in node.vel_pubs:
            stop_cmd = Twist()
            pub.publish(stop_cmd)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
