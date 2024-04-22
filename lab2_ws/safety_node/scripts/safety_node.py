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
        
        
        self.speed = None
        self.scan = LaserScan()
        self.last_scan = LaserScan()
        self.ttc_threshold = 1.
        self.forward_range = np.deg2rad(30)

    def odom_callback(self, odom_msg: Odometry):
        self.speed = odom_msg.twist.twist.linear

    def scan_callback(self, scan_msg: LaserScan):
        
        if self.speed is None:
            self.get_logger().warn('No odometry data available yet.')
            return
        
        self.last_scan = self.scan
        self.scan = scan_msg
        
        velocity = np.sqrt(self.speed.x**2 + self.speed.y**2)
        
        num_points = len(self.scan.ranges)
        mid_point = num_points // 2
        angle_per_step = self.scan.angle_increment
        start_index = int(mid_point - self.forward_range / angle_per_step)
        end_index = int(mid_point + self.forward_range / angle_per_step)
        
        ittc_list = []
        
        for i in range(start_index, end_index):
            distance = self.scan.ranges[i]
            ittc = distance / velocity if velocity > 0 else float('inf')
            ittc_list.append(ittc)
        
        if any(ittc < self.ttc_threshold for ittc in ittc_list):
            self._emergency_stop()

    def _emergency_stop(self):
        self.get_logger().info(f"iTTC below threshold and emergency stop")
                    
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.drive.speed = 0.
        ackermann_msg.drive.steering_angle = 0.
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