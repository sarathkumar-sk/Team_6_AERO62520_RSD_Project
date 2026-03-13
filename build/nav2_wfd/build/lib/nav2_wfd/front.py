#!/usr/bin/env python3

import rclpy
import numpy as np
import math

from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class FrontierExplorer(Node):

    def __init__(self):

        super().__init__('frontier_explorer')

        self.map = None
        self.pose = None

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose')

        self.timer = self.create_timer(5.0, self.explore)

        self.get_logger().info("Frontier explorer started")


    def map_callback(self, msg):
        self.map = msg


    def odom_callback(self, msg):
        self.pose = msg.pose.pose


    def explore(self):

        if self.map is None or self.pose is None:
            self.get_logger().info("Waiting for map and pose...")
            return

        frontier = self.find_frontier()

        if frontier is None:
            self.get_logger().info("No frontiers found")
            return

        self.send_goal(frontier)


    def find_frontier(self):

        data = np.array(self.map.data).reshape(
            self.map.info.height,
            self.map.info.width)

        resolution = self.map.info.resolution
        origin = self.map.info.origin.position

        frontiers = []

        for y in range(1, self.map.info.height-1):
            for x in range(1, self.map.info.width-1):

                if data[y][x] != -1:
                    continue

                neighbors = data[y-1:y+2, x-1:x+2]

                if np.any(neighbors == 0):
                    wx = origin.x + x * resolution
                    wy = origin.y + y * resolution
                    frontiers.append((wx, wy))

        if len(frontiers) == 0:
            return None

        robot_x = self.pose.position.x
        robot_y = self.pose.position.y

        closest = None
        min_dist = float("inf")

        for f in frontiers:

            d = math.sqrt(
                (f[0]-robot_x)**2 +
                (f[1]-robot_y)**2)

            if d < min_dist:
                min_dist = d
                closest = f

        return closest


    def send_goal(self, point):

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Nav2 not ready")
            return

        goal = NavigateToPose.Goal()

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.orientation.w = 1.0

        goal.pose = pose

        self.get_logger().info(
            f"Exploring frontier: {point}")

        self.nav_client.send_goal_async(goal)


def main():

    rclpy.init()

    node = FrontierExplorer()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
