import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, QoSProfile

class IMUNode(Node):
    def __init__(self):
        super().__init__("imu_node")
        self._publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self._subscriber = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE
            )
        )
        self._timer = self.create_timer(0.2, self.timer_callback)

    def imu_callback(self, msg: Imu):
        x_orientation_quaternions = msg.orientation.x
        x_linear_acceleration = msg.linear_acceleration.x
        z_angular_velocity = msg.angular_velocity.z
        self.get_logger().info(
            'X Orientation = %f, X Linear Acceleration = %f, Z Angular Velocity = %f' % 
            (x_orientation_quaternions, x_linear_acceleration, z_angular_velocity)
        )

    def timer_callback(self):
        cmd = Twist()
        cmd.angular.z = 0.5
        cmd.angular.x = 1.0
        self._publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
