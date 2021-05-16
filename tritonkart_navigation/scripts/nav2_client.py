#!/usr/bin/env python3

import os
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Quaternion
from nav2_msgs.action import NavigateToPose
from robot_localization.srv import FromLL
from geographic_msgs.msg import GeoPoint
import pandas as pd

class TritonKartRacing(Node):

    def __init__(self):
        super().__init__('route_manager')
        self.declare_parameter('map_path')
        self.route_path = str(self.get_parameter('map_path').value)
        self.route = pd.read_csv(self.route_path)
        if self.route.shape[0] == 0:
            raise StopIteration("File is empty. Please pass a file with routes")
        self.current_goal = NavigateToPose.Goal()
        self.current_index = 0
        self.converter = self.create_client(FromLL, '/fromLL')
        while not self.converter.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.client = ActionClient(self, NavigateToPose, '/NavigateToPose')
        self.client.wait_for_server()

    def get_goal(self, gps_coord):
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = "map"
        req = FromLL.Request()
        req_msg = GeoPoint()
        req_msg.latitude = gps_coord['latitude']
        req_msg.longitude = gps_coord['longitude']
        req_msg.altitude = gps_coord['altitude']
        req.ll_point = req_msg
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        
        goal.pose.pose.position = result.map_point
        quat = Quaternion()
        quat.x = gps_coord['heading_x']
        quat.y = gps_coord['heading_y']
        quat.z = gps_coord['heading_z']
        quat.w = gps_coord['heading_w']
        goal.pose.pose.orientation = quat
        return goal

    def race(self):
        try:
            next_goal = self.get_goal(self.route.iloc[self.current_index])
            self.current_index += 1
            self.current_index = self.current_index % self.route.shape[0]
            self.get_logger().info("Next goal: {}".format(next_goal))
            self._send_goal_future = self.client.send_goal_async(
                current_goal,
                feedback_callback= lambda x: None)
            self._send_goal_future.add_done_callback(self.response_callback)
        except StopIteration:
            self.get_logger().info("No goals, stopping route manager.")
            return

    def response_callback(self, response):
        accept_status = response.result()
        if not accept_status.accepted:
            self.get_logger().warning('Failed to accept goal')
        else:
            self.get_logger().info('Accepted goal')
            self.future_result = accept_status.get_result_async()
            self.future_result.add_done_callback(self.result_callback)

    def result_callback(self, result):
        self.race()


def main(args = None):
    rclpy.init(args=args)
    try:
        tk_racing = TritonKartRacing()
        route_manager.race()
        rclpy.spin(tk_racing)
    except KeyboardInterrupt:
        tk_racing.get_logger().info("Shutting down racing mode")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
