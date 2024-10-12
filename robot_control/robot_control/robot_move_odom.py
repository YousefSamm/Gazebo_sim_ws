import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile

class Move(Node):
    def __init__(self):
        super().__init__('robot_control')
        self.publisher=self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription=self.create_subscription(Odometry, '/odom', self.Odometry_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.timer=self.create_timer(0.2, self.timer_callback)

    def Odometry_callback(self, msg:Odometry):
        self.x=msg.pose.pose.position.x
        self.y=msg.pose.pose.position.y
    def timer_callback(self):
        self.cmd=Twist()
        self.cmd.linear.x=0.5
        self.cmd.angular.z=0.2
        self.publisher.publish(self.cmd)
        self.get_logger().info('X values is: %s, Y value is: %s' %(self.x, self.y))
def main(args=None):
    rclpy.init(args=args)
    node=Move()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()
if __name__=='__main__':
    main()