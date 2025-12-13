#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from typing import List, Tuple

class FleetGoalPublisher(Node):
    """
    Publishes goal positions based on YAML configurations.
    Reads parameters in the format 'goals_<scenario_name>'.
    """

    def __init__(self):
        super().__init__('fleet_goal_publisher')
        
        self.declare_parameter('num_robots', 3)
        self.declare_parameter('scenario', 'swap')
        
        self.num_robots = self.get_parameter('num_robots').value
        self.scenario = self.get_parameter('scenario').value

        # Declare the goal list parameters (must match YAML keys)
        # We declare them with empty defaults so the node doesn't crash if YAML is missing one
        self.declare_parameter('goals_swap', [0.0])
        self.declare_parameter('goals_cross', [0.0])
        self.declare_parameter('goals_straight', [0.0])
        self.declare_parameter('goals_home', [0.0])
        self.declare_parameter('goals_pillar', [0.0])

        self.goal_pub = self.create_publisher(MarkerArray, '/fleet_goals', 10)
        self.create_timer(1.0, self.publish_goals)
        self.goals_published = False
        
        self.get_logger().info(f'Goal publisher initialized. Selected scenario: {self.scenario}')

    def get_scenario_goals(self) -> List[Tuple[float, float]]:
        """Dynamically fetch the goal list for the selected scenario"""
        
        param_name = f'goals_{self.scenario}'
        
        if not self.has_parameter(param_name):
            self.get_logger().error(f'Scenario parameter "{param_name}" not found!')
            return []

        flat_goals = self.get_parameter(param_name).value
        
        if not flat_goals:
            self.get_logger().warn(f'Goal list for "{self.scenario}" is empty.')
            return []

        goals = []
        for i in range(0, len(flat_goals), 2):
            if i + 1 < len(flat_goals):
                goals.append((flat_goals[i], flat_goals[i+1]))

        if len(goals) < self.num_robots:
            self.get_logger().warn(
                f'Scenario "{self.scenario}" has {len(goals)} goals, but num_robots is {self.num_robots}.'
            )
            
        return goals

    def publish_goals(self):
        """Publish goal markers"""
        goals = self.get_scenario_goals()
        
        if not goals:
            return

        marker_array = MarkerArray()
        
        colors = [
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8), # Red
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8), # Green
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8), # Blue
            ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8), # Yellow
            ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.8), # Magenta
        ]
        
        for i, goal in enumerate(goals):
            if i >= self.num_robots:
                break

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