#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        
        self._laser_scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self._odemetry_sub = self.create_subscription(Odometry, 'ego_racecar/odom', self.odom_callback, 10)
        self._cmd_drive_pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        
        self.speed = 0.
        self.scan = LaserScan()
        self.last_scan = LaserScan()
        self.ttc_threshold = 1.

    def odom_callback(self, odom_msg: Odometry):
        self.speed = odom_msg.twist.twist.linear

    def scan_callback(self, scan_msg: LaserScan):
        self.last_scan = self.scan
        self.scan = scan_msg
        
        curr_time = self.scan.header.stamp.sec + self.scan.header.stamp.nanosec/10e9
        last_time = self.last_scan.header.stamp.sec + self.last_scan.header.stamp.nanosec/10e9
        r_dot = curr_time - last_time
        
        for r in self.scan.ranges:
            if r >= self.scan.angle_min and r <= self.scan.angle_max:
                
                ittc = r / -r_dot
                
                if ittc < self.ttc_threshold:
                    ackermann_msg = AckermannDriveStamped()
                    ackermann_msg.drive.speed = 0.
                    self._cmd_drive_pub.publish(ackermann_msg)
                
def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()