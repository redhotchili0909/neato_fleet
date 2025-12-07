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
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass
import json


@dataclass
class RobotState:
    """Holds the state of a single robot"""
    position: Tuple[float, float] = (0.0, 0.0)  # Global position (after offset correction)
    velocity: Tuple[float, float] = (0.0, 0.0)
    yaw: float = 0.0
    goal: Tuple[float, float] = (0.0, 0.0)
    has_odom: bool = False
    # For offset correction: first odom reading becomes the reference
    odom_offset: Optional[Tuple[float, float]] = None  # (odom_x, odom_y) at startup
    yaw_offset: Optional[float] = None  # yaw at startup
    start_position: Tuple[float, float] = (0.0, 0.0)   # Where we placed the robot
    start_yaw: float = 0.0  # Initial heading (0 = facing +X)


class RVOFleetController(Node):
    """
    Controller for multi-Neato navigation using RVO2.
    """

    def __init__(self):
        super().__init__('rvo_fleet_controller')

        # Declare parameters
        self.declare_parameter('num_robots', 3)
        self.declare_parameter('max_speed', 0.1)  # m/s
        self.declare_parameter('max_angular_speed', 1.5)  # rad/s
        self.declare_parameter('neighbor_dist', 10.0)  # how far in meters to look for other agents
        self.declare_parameter('max_neighbors', 10)   # max number of other agents to consider
        self.declare_parameter('time_horizon', 5.0)  # how far ahead in seconds to plan for other agents
        self.declare_parameter('time_horizon_obst', 2.0)  # how far ahead in seconds to plan for obstacles
        self.declare_parameter('robot_radius', 0.4)  # physical radius of robot in meters
        self.declare_parameter('goal_tolerance', 0.1)  # how close to the goal the agent is
        self.declare_parameter('control_rate', 10.0)  # hz
        # Start positions: flat list [x1,y1, x2,y2, x3,y3] - default (0,0), (2,0), (4,0)
        self.declare_parameter('start_positions', [0.0, 0.0, 2.0, 0.0, 4.0, 0.0])
        # Start yaws: [yaw1, yaw2, yaw3] in radians (0 = facing +X, pi/2 = facing +Y)
        self.declare_parameter('start_yaws', [0.0, 0.0, 0.0])

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
            # parse flat list [x1,y1, x2,y2, ...] into tuples
            for i in range(0, len(start_pos_flat), 2):
                if i + 1 < len(start_pos_flat):
                    self.start_positions.append((start_pos_flat[i], start_pos_flat[i + 1]))

        # Parse start yaws - if empty, default to 0 (facing +X)
        start_yaws_flat = self.get_parameter('start_yaws').value
        self.start_yaws = []
        if start_yaws_flat and len(start_yaws_flat) >= 1:
            # parse flat list [yaw1, yaw2, ...] into list   
            for i in range(0, len(start_yaws_flat)):
                if i < len(start_yaws_flat):
                    self.start_yaws.append(start_yaws_flat[i])

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

        # robot states and agent lists
        self.robot_states: List[RobotState] = []
        self.agent_ids: List[int] = []
        self.goals_received = False  # Wait for goals before moving

        # initialize robots
        for i in range(self.num_robots):
            start_pos = self.start_positions[i]
            start_yaw = self.start_yaws[i]
            # add agent to RVO sim
            agent_id = self.sim.addAgent(start_pos)
            self.agent_ids.append(agent_id)
            
            # initialize robot state with no set goal destination  + will wait for fleet commands
            state = RobotState(
                position=start_pos,
                goal=start_pos,  # start position
                start_position=start_pos,
                start_yaw=start_yaw,
                odom_offset=None,
                yaw_offset=None
            )
            self.robot_states.append(state)
            self.get_logger().info(
                f'Robot {i+1} start: pos=({start_pos[0]:.2f}, {start_pos[1]:.2f}), yaw={math.degrees(start_yaw):.1f}°'
            )

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
        
        # debug publisher
        self.debug_pub = self.create_publisher(String, '/rvo_debug', 10)
        self.debug_timer = self.create_timer(0.5, self.publish_debug_state)
        
        # RViz visualization publishers
        self.robot_marker_pub = self.create_publisher(MarkerArray, '/rvo_robot_markers', 10)
        self.goal_marker_pub = self.create_publisher(MarkerArray, '/rvo_goal_markers', 10)
        self.velocity_marker_pub = self.create_publisher(MarkerArray, '/rvo_velocity_markers', 10)
        self.viz_timer = self.create_timer(0.1, self.publish_visualization)  # 10Hz viz update
        
        # Store last RVO velocities for visualization
        self.last_rvo_velocities = [(0.0, 0.0)] * self.num_robots

        self.get_logger().info(
            f'RVO Fleet Controller initialized with {self.num_robots} robots'
        )

    def publish_debug_state(self):
        """Publish internal corrected state for debugging"""
        debug_data = {
            'goals_received': self.goals_received,
            'robots': []
        }
        for i, state in enumerate(self.robot_states):
            robot_data = {
                'id': i + 1,
                'has_odom': state.has_odom,
                'corrected_pos': list(state.position),
                'corrected_yaw_deg': math.degrees(state.yaw),
                'goal': list(state.goal),
                'odom_offset': list(state.odom_offset) if state.odom_offset else None,
                'yaw_offset_deg': math.degrees(state.yaw_offset) if state.yaw_offset else None,
                'start_pos': list(state.start_position),
                'start_yaw_deg': math.degrees(state.start_yaw),
            }
            debug_data['robots'].append(robot_data)
        
        msg = String()
        msg.data = json.dumps(debug_data, indent=2)
        self.debug_pub.publish(msg)

    def publish_visualization(self):
        """Publish RViz markers for robot positions, goals, and velocities"""
        now = self.get_clock().now().to_msg()
        
        # Colors for each robot (matching typical RViz colors)
        colors = [
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9),   # Red - Robot 1
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9),   # Green - Robot 2
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.9),   # Blue - Robot 3
            ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9),   # Yellow - Robot 4
            ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.9),   # Magenta - Robot 5
        ]
        
        # === Robot Position Markers (where robot thinks it is) ===
        robot_markers = MarkerArray()
        for i, state in enumerate(self.robot_states):
            if not state.has_odom:
                continue
                
            # Robot body - cylinder
            body = Marker()
            body.header.frame_id = 'odom'
            body.header.stamp = now
            body.ns = 'robot_body'
            body.id = i
            body.type = Marker.CYLINDER
            body.action = Marker.ADD
            body.pose.position.x = state.position[0]
            body.pose.position.y = state.position[1]
            body.pose.position.z = 0.05
            body.scale.x = self.robot_radius * 2  # diameter
            body.scale.y = self.robot_radius * 2
            body.scale.z = 0.1
            body.color = colors[i % len(colors)]
            robot_markers.markers.append(body)
            
            # Robot heading arrow
            heading = Marker()
            heading.header.frame_id = 'odom'
            heading.header.stamp = now
            heading.ns = 'robot_heading'
            heading.id = i
            heading.type = Marker.ARROW
            heading.action = Marker.ADD
            heading.pose.position.x = state.position[0]
            heading.pose.position.y = state.position[1]
            heading.pose.position.z = 0.15
            # Convert yaw to quaternion for arrow orientation
            heading.pose.orientation.z = math.sin(state.yaw / 2)
            heading.pose.orientation.w = math.cos(state.yaw / 2)
            heading.scale.x = 0.3  # arrow length
            heading.scale.y = 0.05  # arrow width
            heading.scale.z = 0.05  # arrow height
            heading.color = colors[i % len(colors)]
            heading.color.a = 1.0
            robot_markers.markers.append(heading)
            
            # Robot ID text
            text = Marker()
            text.header.frame_id = 'odom'
            text.header.stamp = now
            text.ns = 'robot_id'
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = state.position[0]
            text.pose.position.y = state.position[1]
            text.pose.position.z = 0.4
            text.scale.z = 0.15  # text height
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = f'R{i+1}'
            robot_markers.markers.append(text)
        
        self.robot_marker_pub.publish(robot_markers)
        
        # === Goal Markers ===
        goal_markers = MarkerArray()
        for i, state in enumerate(self.robot_states):
            # Goal position - ring/torus shape
            goal = Marker()
            goal.header.frame_id = 'odom'
            goal.header.stamp = now
            goal.ns = 'goal_marker'
            goal.id = i
            goal.type = Marker.CYLINDER
            goal.action = Marker.ADD
            goal.pose.position.x = state.goal[0]
            goal.pose.position.y = state.goal[1]
            goal.pose.position.z = 0.01
            goal.scale.x = 0.25
            goal.scale.y = 0.25
            goal.scale.z = 0.02
            goal.color = colors[i % len(colors)]
            goal.color.a = 0.5  # semi-transparent
            goal_markers.markers.append(goal)
            
            # Goal text label
            goal_text = Marker()
            goal_text.header.frame_id = 'odom'
            goal_text.header.stamp = now
            goal_text.ns = 'goal_text'
            goal_text.id = i
            goal_text.type = Marker.TEXT_VIEW_FACING
            goal_text.action = Marker.ADD
            goal_text.pose.position.x = state.goal[0]
            goal_text.pose.position.y = state.goal[1]
            goal_text.pose.position.z = 0.2
            goal_text.scale.z = 0.1
            goal_text.color = colors[i % len(colors)]
            goal_text.text = f'G{i+1}'
            goal_markers.markers.append(goal_text)
        
        self.goal_marker_pub.publish(goal_markers)
        
        # === Velocity Arrows (RVO computed velocity) ===
        velocity_markers = MarkerArray()
        for i, state in enumerate(self.robot_states):
            if not state.has_odom:
                continue
            
            vx, vy = self.last_rvo_velocities[i]
            speed = math.sqrt(vx**2 + vy**2)
            
            if speed < 0.001:
                # No velocity - show small dot
                vel_marker = Marker()
                vel_marker.header.frame_id = 'odom'
                vel_marker.header.stamp = now
                vel_marker.ns = 'velocity'
                vel_marker.id = i
                vel_marker.type = Marker.SPHERE
                vel_marker.action = Marker.ADD
                vel_marker.pose.position.x = state.position[0]
                vel_marker.pose.position.y = state.position[1]
                vel_marker.pose.position.z = 0.25
                vel_marker.scale.x = 0.05
                vel_marker.scale.y = 0.05
                vel_marker.scale.z = 0.05
                vel_marker.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.5)
            else:
                # Velocity arrow
                vel_marker = Marker()
                vel_marker.header.frame_id = 'odom'
                vel_marker.header.stamp = now
                vel_marker.ns = 'velocity'
                vel_marker.id = i
                vel_marker.type = Marker.ARROW
                vel_marker.action = Marker.ADD
                
                # Arrow from robot position in velocity direction
                start = Point()
                start.x = state.position[0]
                start.y = state.position[1]
                start.z = 0.25
                
                # Scale arrow length by velocity (capped for visibility)
                arrow_length = min(speed * 5.0, 1.0)  # 5x velocity, max 1m
                end = Point()
                end.x = state.position[0] + (vx / speed) * arrow_length
                end.y = state.position[1] + (vy / speed) * arrow_length
                end.z = 0.25
                
                vel_marker.points = [start, end]
                vel_marker.scale.x = 0.04  # shaft diameter
                vel_marker.scale.y = 0.08  # head diameter
                vel_marker.scale.z = 0.0
                
                # Color: green for moving, yellow for slow
                if speed > 0.05:
                    vel_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.5, a=1.0)  # cyan-green
                else:
                    vel_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # yellow
            
            velocity_markers.markers.append(vel_marker)
        
        self.velocity_marker_pub.publish(velocity_markers)


    def odom_callback(self, msg, robot_idx):
        """Update robot state from odometry message"""
        state = self.robot_states[robot_idx]
        
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2.0*(q.w*q.z+q.x*q.y)
        cosy_cosp = 1.0-2.0*(q.y*q.y+q.z*q.z)
        odom_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        if state.odom_offset is None:
            state.odom_offset = (odom_x, odom_y)
            state.yaw_offset = odom_yaw
        
        # calculate global odom
        state.position = (
            state.start_position[0] + (odom_x - state.odom_offset[0]),
            state.start_position[1] + (odom_y - state.odom_offset[1])
        )
        
        state.yaw = self.normalize_angle(
            state.start_yaw + (odom_yaw - state.yaw_offset)
        )
        
        # Extract velocity
        state.velocity = (
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y
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
                        f'Robot {marker.id+1} goal updated to '
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
                    f'Robot {i+1} reached goal at ({state.goal[0]:.2f}, {state.goal[1]:.2f})!',
                    throttle_duration_sec=5.0
                )
            
            self.sim.setAgentPrefVelocity(agent_id, pref_vel)

        # step the RVO simulation
        self.sim.doStep()

        for i, state in enumerate(self.robot_states):
            agent_id = self.agent_ids[i]
            
            # get the collision-free velocity from RVO
            rvo_vel = self.sim.getAgentVelocity(agent_id)
            
            # Store for visualization
            self.last_rvo_velocities[i] = rvo_vel
            
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
            if abs(yaw_error) > math.pi / 4:  # > 45 degrees
                cmd.linear.x = 0.05
                cmd.angular.z = 1.0 * yaw_error  # turn toward goal
            else:
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
        node.get_logger().warn('Shutting down - stopping all robots...')
    finally:
        # Send stop commands multiple times to ensure robots stop
        import time
        stop_cmd = Twist()
        for _ in range(10):  # Send 10 stop commands over 1 second
            for pub in node.vel_pubs:
                pub.publish(stop_cmd)
            time.sleep(0.1)
        
        node.get_logger().info('All robots stopped.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
