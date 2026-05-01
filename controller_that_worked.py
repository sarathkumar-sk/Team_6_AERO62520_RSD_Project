#controller_that_worked.py
#!/usr/bin/env python3
import sys
import termios
import tty
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, 
    PositionConstraint, 
    OrientationConstraint, 
    JointConstraint, 
    BoundingVolume
)
from shape_msgs.msg import SolidPrimitive

# ===========================================================================
# CONFIGURATION
# ===========================================================================
GROUP_NAME_ARM     = "arm_group"       
GROUP_NAME_GRIPPER = "gripper"   

ARM_JOINT_NAMES = [
    "joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
    "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6",
]

GRIPPER_JOINT_NAMES = ["gripper_controller"] 

# Target for the 'm' key
MOVE_TO_TARGET = [0.2, 0.00, 0.02]

BASE_LINK = "g_base"
EEF_LINK  = "joint6_flange"

# ===========================================================================
# Helpers
# ===========================================================================
def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

# ===========================================================================
# Test Node
# ===========================================================================
class CubeGraspNode(Node):
    def __init__(self):
        super().__init__("cube_grasp_node")
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        self.get_logger().info("Waiting for MoveGroup action server...")
        self._action_client.wait_for_server()
        
        self.get_logger().info("CubeGraspNode ready. [h] Home, [m] Move, [o] Open, [c] Close, [q] Quit.")
        
        self.menu_thread = threading.Thread(target=self._run_menu, daemon=True)
        self.menu_thread.start()

    def _send_joint_goal(self, group_name, joint_names, positions):
        """Generic helper to move joints for any group."""
        goal = MoveGroup.Goal()
        goal.request.group_name = group_name
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0  
        goal.request.max_velocity_scaling_factor = 0.15  
        goal.request.max_acceleration_scaling_factor = 0.15
                
        con = Constraints()
        for name, pos in zip(joint_names, positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.1
            jc.tolerance_below = 0.1
            jc.weight = 1.0
            con.joint_constraints.append(jc)
        
        goal.request.goal_constraints.append(con)
        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        handle = future.result()
        if handle and handle.accepted:
            res = handle.get_result_async()
            rclpy.spin_until_future_complete(self, res)

    def _move_to_pose(self, position, label: str):
        """Moves the arm to an XYZ coordinate."""
        print(f"  → Planning to {label} at {position} …")
        goal = MoveGroup.Goal()
        goal.request.group_name = GROUP_NAME_ARM
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0  
        goal.request.max_velocity_scaling_factor = 0.15  
        goal.request.max_acceleration_scaling_factor = 0.15

        con = Constraints(name="goal")
        
        # Position Constraint
        pos = PositionConstraint()
        pos.header.frame_id = BASE_LINK
        pos.link_name = EEF_LINK
        bv = BoundingVolume()
        prim = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.002])
        target = PoseStamped()
        target.pose.position.x, target.pose.position.y, target.pose.position.z = position
        bv.primitives.append(prim)
        bv.primitive_poses.append(target.pose)
        pos.constraint_region = bv
        
        # Orientation Constraint (Tool Downwards)
        ori = OrientationConstraint()
        ori.header.frame_id = BASE_LINK
        ori.link_name = EEF_LINK
        ori.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        ori.absolute_x_axis_tolerance = 0.1
        ori.absolute_y_axis_tolerance = 0.1
        ori.absolute_z_axis_tolerance = 0.1

        con.position_constraints.append(pos)
        con.orientation_constraints.append(ori)
        goal.request.goal_constraints.append(con)
        
        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        handle = future.result()
        if handle and handle.accepted:
            res = handle.get_result_async()
            rclpy.spin_until_future_complete(self, res)

    def open_gripper(self):
        print("  → Opening Gripper")
        self._send_joint_goal(GROUP_NAME_GRIPPER, GRIPPER_JOINT_NAMES, [0.0])

    def close_gripper(self):
        print("  → Closing Gripper")
        self._send_joint_goal(GROUP_NAME_GRIPPER, GRIPPER_JOINT_NAMES, [-0.7]) 

    def move_home(self):
        print("  → Moving Home")
        self._send_joint_goal(GROUP_NAME_ARM, ARM_JOINT_NAMES, [0.0]*6)

    def move_to_target(self):
        self._move_to_pose(MOVE_TO_TARGET, "TARGET_POSITION")

    def _run_menu(self):
        """Internal menu loop."""
        print("\n[h] Home | [m] Move to Pose | [o] Open | [c] Close | [q] Quit\n")
        while rclpy.ok():
            k = get_key()
            if k == "o": self.open_gripper()
            elif k == "c": self.close_gripper()
            elif k == "h": self.move_home()
            elif k == "m": self.move_to_target()
            elif k == "q": 
                print("Quitting...")
                time.sleep(0.2)
                sys.exit(0)

# ===========================================================================
# Main Entry Point
# ===========================================================================
def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    node     = CubeGraspNode()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
