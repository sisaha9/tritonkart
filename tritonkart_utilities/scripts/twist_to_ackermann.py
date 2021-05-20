#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

class DriveConverter(Node):

    def __init__(self):
        super().__init__("DriveConverter")

        self.wheelbase = float(self.declare_parameter("wheelbase", 1.0).value)
        self.twist_topic = str(self.declare_parameter('twist_topic', '/cmd_vel').value)
        self.ackermann_topic = str(self.declare_parameter('vehicle_topic', '/ackermann_cmd').value)

        self.twist_subscriber = self.create_subscription(
            Twist,
            self.twist_topic,
            self.twist_callback,
            1
        )

        self.vehicle_publisher = self.create_publisher(
            AckermannDriveStamped,
            self.ackermann_topic,
            1
        )

    def twist_callback(self, drive_cmd):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.drive.speed = float(drive_cmd.linear.x)
        steering = self.convert_angle(msg.drive.speed, drive_cmd.angular.z)
        msg.drive.steering_angle = steering

        self.vehicle_publisher.publish(msg)

    def convert_angle(self, v, av):
        if av == 0 or v == 0:
            return 0
        return math.atan(self.wheelbase/(v/av))


def main(args=None):
    rclpy.init(args=args)

    drive_converter = DriveConverter()

    rclpy.spin(drive_converter)

    drive_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()