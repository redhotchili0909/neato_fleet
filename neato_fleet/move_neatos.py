import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from neato2_interfaces.msg import TwistArray
from nav_msgs.msg import Odometry
from functools import partial

class MoveNeatos(Node):
    
    def __init__(self):
        super().__init__('move_neatos_node')

        self.NUM_ROBOTS = 3
        self.move_pubs = []
        self.odom_subs = []
        
        for i in range(1, self.NUM_ROBOTS + 1):
            move_pub = self.create_publisher(Twist, f'/robot{i}/cmd_vel', 10)
            odom_sub = self.create_subscription(Odometry, f'/robot{i}/odom', partial(self.odom_callback, robot_id=i), 10)
            
            self.move_pubs.append(move_pub)
            self.odom_subs.append(odom_sub)
            
        self.move_sub = self.create_subscription(TwistArray, '/move_neatos', self.move_neatos, 10)
            
        self.create_timer(0.1, self.periodic)
        
    def odom_callback(self, msg: Odometry, robot_id = 0):
        print(f'Robot: {robot_id} | Pose: {msg.pose}')
        
    def move_neatos(self, msg: TwistArray):
        for i, twist_msg in enumerate(msg.twist_array):
            self.move_pubs[i].publish(twist_msg)
            
    def generate_twist(self, lin, ang):
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        return msg
        
    def periodic(self):
        twist_arr_msg = []
        for i in range(self.NUM_ROBOTS):
            twist_arr_msg.append(self.generate_twist(0.1, 0 + (0.01 * i)))
            
        twist_arr = TwistArray()
        twist_arr.twist_array = twist_arr_msg
        
        self.move_neatos(twist_arr)
        
        # print("test")
        
            
def main(args=None):
    rclpy.init(args=args)
    node = MoveNeatos()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        
        