#!/bin/env python3
import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, QoSProfile

class Laser_node(Node):
    def __init__(self):
        super().__init__('laser_node')
        self._publisher=self.create_publisher(Twist, "/cmd_vel", 10)
        self._subscriber=self.create_subscription(LaserScan, '/laser/scan', self.laserscan_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE))
        self._timer=self.create_timer(0.2, self.timer_callback)
    def laserscan_callback(self, msg: LaserScan):
        min_distance=min(msg.ranges)
        self.get_logger().info('minimum distance = %s' %(min_distance))
    def timer_callback(self):
        cmd=Twist()
        cmd.linear.x=0.2
        cmd.angular.z=0.5
        self._publisher.publish(cmd)
def main(args=None):
    rclpy.init(args=args)
    node=Laser_node()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()