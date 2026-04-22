#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from moveit2 import MoveIt2
from moveit2.robots import MyCobot
import tf2_ros
import tf2_geometry_msgs

import math
import time

class CubeGraspNode(Node):
    def __init__(self):
        super().__init__("cube_grasp_node")

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=MyCobot.joint_names(),
            base_link_name="g_base",
            end_effector_name="joint6_flange",
            group_name="manipulator",
            execute_via_moveit=True
        )

        # Slow and safe for testing
        self.moveit2.max_velocity = 0.2
        self.moveit2.max_acceleration = 0.2

        # Subscribe to cube pose
        self.create_subscription(
            PoseStamped,
            "/cube_pose_camera",
            self.cube_pose_callback,
            10
        )

        self.get_logger().info("CubeGraspNode ready — waiting for cube poses")

    # ---------------------------------------------------------
    # Callback: cube pose received
    # ---------------------------------------------------------
    def cube_pose_callback(self, msg: PoseStamped):
        try:
            # Transform cube pose into robot base frame
            transform = self.tf_buffer.lookup_transform(
                "g_base",
                msg.header.frame_id,
                rclpy.time.Time()
            )
            cube_pose_base = tf2_geometry_msgs.do_transform_pose(msg, transform)

        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return

        x = cube_pose_base.pose.position.x
        y = cube_pose_base.pose.position.y
        z = cube_pose_base.pose.position.z

        self.get_logger().info(f"Cube in base frame: x={x:.3f}, y={y:.3f}, z={z:.3f}")

        # ---------------------------------------------------------
        # Build pre‑grasp and grasp poses
        # ---------------------------------------------------------
        pre_grasp = PoseStamped()
        pre_grasp.header.frame_id = "g_base"
        pre_grasp.pose.position.x = x
        pre_grasp.pose.position.y = y
        pre_grasp.pose.position.z = z + 0.10  # 10 cm above cube

        # Tool pointing down
        pre_grasp.pose.orientation.x = 1.0
        pre_grasp.pose.orientation.y = 0.0
        pre_grasp.pose.orientation.z = 0.0
        pre_grasp.pose.orientation.w = 0.0

        grasp = PoseStamped()
        grasp.header.frame_id = "g_base"
        grasp.pose.position.x = x
        grasp.pose.position.y = y
        grasp.pose.position.z = z + 0.02  # 2 cm above cube

        grasp.pose.orientation = pre_grasp.pose.orientation

        # ---------------------------------------------------------
        # Execute motion
        # ---------------------------------------------------------
        self.get_logger().info("Moving to pre‑grasp")
        self.moveit2.move_to_pose(pre_grasp)
        self.moveit2.wait_until_executed()

        time.sleep(0.5)

        self.get_logger().info("Moving to grasp")
        self.moveit2.move_to_pose(grasp)
        self.moveit2.wait_until_executed()

        time.sleep(0.5)

        # ---------------------------------------------------------
        # Close gripper (replace with your gripper command)
        # ---------------------------------------------------------
        self.get_logger().info("Closing gripper")
        # TODO: insert your gripper command here
        time.sleep(0.5)

        # ---------------------------------------------------------
        # Lift cube
        # ---------------------------------------------------------
        lift = PoseStamped()
        lift.header.frame_id = "g_base"
        lift.pose.position.x = x
        lift.pose.position.y = y
        lift.pose.position.z = z + 0.15
        lift.pose.orientation = pre_grasp.pose.orientation

        self.get_logger().info("Lifting cube")
        self.moveit2.move_to_pose(lift)
        self.moveit2.wait_until_executed()

        self.get_logger().info("Grasp sequence complete")

def main(args=None):
    rclpy.init(args=args)
    node = CubeGraspNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
