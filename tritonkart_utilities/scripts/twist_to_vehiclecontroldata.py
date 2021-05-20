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
        self.clock_topic = str(self.declare_parameter('clock_topic', '/clock').value)
        self.twist_topic = str(self.declare_parameter('twist_topic', '/cmd_vel').value)
        self.svl_topic = str(self.declare_parameter('svl_topic', '/vehicle_output').value)

        self.clock_subscriber = self.create_subscription(
            Clock,
            self.clock_topic,
            self.clock_callback,
            1
        )

        self.latest_time = None

        self.vehicle_publisher = self.create_publisher(
            VehicleControlData,
            self.svl_topic,
            1
        )

        self.twist_subscriber = self.create_subscription(
            Twist,
            self.twist_topic,
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
            vehicle_output.braking_pct = (self.last_speed - speed)/self.last_speed

        self.last_speed = speed
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