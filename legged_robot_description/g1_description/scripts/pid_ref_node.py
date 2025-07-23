#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from control_msgs.msg import MultiDOFCommand

import math

class PIDRefNode(Node):
    def __init__(self):
        super().__init__('pid_ref_node')
        self.publisher_ = self.create_publisher(MultiDOFCommand, '/pid_controller/reference', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.flag = True

    def timer_callback(self):
        msg = MultiDOFCommand()
        
        msg.values = [0.0] * 27
        msg.values_dot = [0.0] * 27

        msg.values[3] = math.radians(30.0) if self.flag else 0.0
        self.flag = not self.flag

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing PID reference')

if __name__ == '__main__':
    rclpy.init()
    node = PIDRefNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()