#!/usr/bin/env python3
"""
LMS200 LiDAR Data Sanitizer Node (ROS 2 Jazzy)

Listens to scan messages from the legacy LMS200 driver on /bad_scan
and republishes them to /good_scan with corrected header metrics.
Adapted directly from Professor Eduardo Feo Flushing's ROS 1 codebase.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserFixer(Node):

    def __init__(self):
        super().__init__('lms200fix')
        self.get_logger().info("Starting laser fixer node (ROS 2)")
        
        # Subscriptions and Publishers
        self.subscription = self.create_subscription(
            LaserScan, 
            '/bad_scan', 
            self._handle_scan_message, 
            10
        )
        self.publisher = self.create_publisher(LaserScan, '/good_scan', 10)

    def _handle_scan_message(self, msg):
        sanitized_msg = msg
        
        # Injecting Professor Eduardo's exact hardware constants
        sanitized_msg.angle_increment = 0.0174532923847
        sanitized_msg.time_increment = 3.70370362361e-05
        sanitized_msg.scan_time = 0.0133333336562
        sanitized_msg.range_min = 0.0
        sanitized_msg.range_max = 81.0
        
        self.publisher.publish(sanitized_msg)

def main(args=None):
    rclpy.init(args=args)
    laser_fixer = LaserFixer()
    try:
        rclpy.spin(laser_fixer)
    except KeyboardInterrupt:
        laser_fixer.get_logger().info('Shutting down cleanly.')
    finally:
        laser_fixer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()