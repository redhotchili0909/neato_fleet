#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from typing import List, Tuple


class FleetGoalPublisher(Node):
    """Publishes goal positions for the fleet based on different scenarios"""

    def __init__(self):
        super().__init__('fleet_goal_publisher')
        
        self.declare_parameter('num_robots', 3)
        self.declare_parameter('scenario', 'swap')  # swap,cross,home
        
        self.num_robots = self.get_parameter('num_robots').value
        self.scenario = self.get_parameter('scenario').value
        
        self.goal_pub = self.create_publisher(MarkerArray, '/fleet_goals', 10)
            
        self.create_timer(1.0, self.publish_goals)
        self.goals_published = False
        
        self.get_logger().info(f'Goal publisher initialized with scenario: {self.scenario}')

    def get_scenario_goals(self):
        """Get goals based on selected scenario"""
        if self.scenario == 'swap':
            return self.swap_scenario()
        elif self.scenario == 'cross':
            return self.cross_scenario()
        elif self.scenario == 'home':
            return self.home_scenario()
        else:
            self.get_logger().warn(f'Unknown scenario: {self.scenario}, using swap')
            return self.swap_scenario()

    def swap_scenario(self):
        # if robots start at (0,0), (2,0), (4,0)
        # they should go to (4,0), (2,0), (0,0)
        goals = []
        for i in range(self.num_robots):
            goals.append(((self.num_robots - 1 - i) * 2.0, 0.0))
        return goals
    
    def cross_scenario(self):
        """Robots in a cross pattern swapping through center"""
        if self.num_robots == 3:
            return [(4.0, 2.0), (0.0, 2.0), (2.0, 2.0)]
        else:
            return self.swap_scenario()


    def home_scenario(self) -> List[Tuple[float, float]]:
        """Return all robots to their original starting positions"""
        # robots start at (0,0), (2,0), (4,0)
        goals = []
        for i in range(self.num_robots):
            goals.append((i * 2.0, 0.0))
        return goals

    def publish_goals(self):
        """Publish goal markers for visualization and fleet controller"""
        goals = self.get_scenario_goals()
        
        marker_array = MarkerArray()
        colors = [
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8),
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8),
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8),
            ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8),
            ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.8),
        ]
        
        for i, goal in enumerate(goals):
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'fleet_goals'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = goal[0]
            marker.pose.position.y = goal[1]
            marker.pose.position.z = 0.01
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.02
            marker.color = colors[i % len(colors)]
            marker_array.markers.append(marker)
            
            # Only log once
            if not self.goals_published:
                self.get_logger().info(f'Robot {i} goal: ({goal[0]:.2f}, {goal[1]:.2f})')
        
        self.goal_pub.publish(marker_array)
        self.goals_published = True


def main(args=None):
    rclpy.init(args=args)
    node = FleetGoalPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
