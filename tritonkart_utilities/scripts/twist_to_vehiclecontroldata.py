#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from lgsvl_msgs.msg import VehicleControlData

class DriveConverter(Node):

    def __init__(self):
        super().__init__("DriveConverter")

        self.clock_subscriber = self.create_subscription(
            Clock,
            "/clock",
            self.clock_callback,
            1
        )

        self.latest_time = None

        self.vehicle_publisher = self.create_publisher(
            VehicleControlData,
            "/vehicle_output",
            1
        )

        self.twist_subscriber = self.create_subscription(
            Twist,
            "/nav2_twist",
            self.twist_callback,
            1
        )

        self.last_speed = 0

    def clock_callback(self, time):
        self.latest_time = time.clock

    def twist_callback(self, drive_cmd):
        speed = drive_cmd.linear.x
        vehicle_output = VehicleControlData()

        if speed > self.last_speed:
            vehicle_output.acceleration_pct = (speed - self.last_speed)/speed

        elif speed < self.last_speed:
            vehicle_output.braking_pct = (self.last_speed - self.speed)/self.last_speed

        vehicle_output.target_wheel_angular_rate = drive_cmd.angular.x
        if self.latest_time is not None:
            vehicle_output.header.stamp = self.latest_time
            self.publisher.publish(control)


def main(args=None):
    rclpy.init(args=args)

    drive_converter = DriveConverter()

    rclpy.spin(drive_converter)

    drive_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()