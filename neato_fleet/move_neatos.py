import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8

class SimpleMove(Node):
    
    def __init__(self):
        super().__init__('move_neatos_node')

        self.NUM_ROBOTS = 2
        self.pubs = []
        
        for i in range(1, self.NUM_ROBOTS + 1):
            pub = self.create_publisher(Twist, f'robot{i}/cmd_vel', 10)
            self.pubs.append(pub)
            
            
        self.create_timer(0.1, self.periodic)
        
    def periodic(self):
        for pub in self.pubs:
            msg = Twist()
            msg.linear.x = -0.1
            pub.publish(msg)
            
def main(args=None):
    rclpy.init(args=args)
    node = SimpleMove()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        
        