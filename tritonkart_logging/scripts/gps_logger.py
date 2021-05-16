#!/usr/bin/env python3
import pandas as pd
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
import message_filters
import os


class GPSLogger(Node):
    def __init__(self):
        super().__init__("gps_logger")
        self.declare_parameter('frequency')
        self.declare_parameter('save_path')
        freq = float(str(self.get_parameter('frequency').value))
        self.save_path = str(self.get_parameter('save_path').value)
        gps_sub = message_filters.Subscriber(self, NavSatFix, '/gps')
        imu_sub = message_filters.Subscriber(self, Imu, '/imu')
        self.logs = []
        self.recent_message = None
        ts = message_filters.ApproximateTimeSynchronizer([gps_sub, imu_sub], \
            1, 0.1)
        ts.registerCallback(self.callback)

        self.create_timer(1/freq, self.log_msg)

    def callback(self, gps_data, imu_data):
        self.recent_message = [gps_data.latitude, gps_data.longitude, \
        gps_data.altitude, imu_data.orientation.x, imu_data.orientation.y, \
        imu_data.orientation.z, imu_data.orientation.w]

    def log_msg(self):
        if self.recent_message is not None:
            self.logs.append(self.recent_message)
            self.get_logger().info("Logging 1 coordinate")


def main(args = None):
    rclpy.init(args=args)
    gps_node = GPSLogger()
    try:
        rclpy.spin(gps_node)
    except KeyboardInterrupt:
        gps_node.get_logger().info("Merging")
    finally:
        df_logs = pd.DataFrame(gps_node.logs, \
            columns = ['latitude', 'longitude', 'altitude', 'heading_x', \
            'heading_y', 'heading_z', 'heading_w'])
        df_logs.to_csv(gps_node.save_path, index=False)
        gps_node.get_logger().info("Done")
        gps_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()