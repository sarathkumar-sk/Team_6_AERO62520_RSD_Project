#!/usr/bin/env python3
"""
cube_place_node.py
-------------------
Assumes the arm is already holding a cube (gripper closed).
Subscribes to /box_pose_camera from realsense_box_publisher, transforms
the box position into the robot base frame, and executes a single drop
sequence into the 20×20×20 cm box.

TCP offset
----------
There is 10 cm between joint6_flange (what MoveIt2 controls) and the
actual fingertip grip point.  Every Cartesian target is therefore raised
by TCP_OFFSET so that the FINGERS reach the intended position.

  flange_target_z = intended_position_z + TCP_OFFSET

All offsets below are defined relative to the BOX geometry (intuitive),
and TCP_OFFSET is added automatically in _move_to_pose().

State machine:
  IDLE  ──►  PLACING  ──►  DONE
               │ (on error)
               └──────────►  IDLE  (allows retry)

Subscriptions:
  /box_pose_camera  (geometry_msgs/PoseStamped)
  /box_color        (std_msgs/String)
"""

import enum
import math
import threading
import time

import rclpy
import rclpy.executors
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from std_msgs.msg import String
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PositionConstraint,
)
from shape_msgs.msg import SolidPrimitive
import tf2_ros
import tf2_geometry_msgs          # noqa: F401
from tf2_ros import StaticTransformBroadcaster


# ===========================================================================
# CONFIGURATION
# ===========================================================================

GROUP_NAME_ARM     = "arm_group"      # CHANGE: verify in your .srdf
GROUP_NAME_GRIPPER = "gripper"        # CHANGE: verify in your .srdf

ARM_JOINT_NAMES = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]

GRIPPER_JOINT_NAMES = ["gripper_controller"]  # CHANGE: verify against /joint_states

BASE_LINK = "g_base"
EEF_LINK  = "joint6_flange"

HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # CHANGE

GRIPPER_OPEN   =  0.0   # CHANGE
GRIPPER_CLOSED = -0.7   # CHANGE

# ------------------------------------------------------------------
# TCP (Tool Centre Point) offset
# Distance from joint6_flange to the actual fingertip grip point.
# CHANGE if your gripper is a different length.
# ------------------------------------------------------------------
TCP_OFFSET = 0.10   # 10 cm

# ------------------------------------------------------------------
# Box geometry
# The box is 20×20×20 cm with its open top facing up.
# realsense_box_publisher gives us the nearest face centre in the
# camera frame, which after TF transform into g_base gives us a
# position at roughly the base/floor level of the box.
# ------------------------------------------------------------------
BOX_HEIGHT_M = 0.20   # full internal height of the box (metres)

# ------------------------------------------------------------------
# Place offsets — defined relative to the BOX OPENING (top rim).
# TCP_OFFSET is added automatically inside _move_to_pose().
#
#   SAFE_HOVER_ABOVE_RIM — fingertips this far ABOVE the rim on approach.
#                          Clears the box walls if the detected pose
#                          has small errors.
#
#   RELEASE_BELOW_RIM    — fingertips this far BELOW the rim to release.
#                          Keeps the drop height short so the cube
#                          doesn't bounce out.
# ------------------------------------------------------------------
SAFE_HOVER_ABOVE_RIM = 0.10   # 10 cm above rim
RELEASE_BELOW_RIM    = 0.05   # 5 cm below rim (inside box)

# Only accept a box within this distance from the robot base
MAX_PLACE_DIST = 0.70   # metres — CHANGE to suit your workspace

# Motion tuning
VELOCITY_SCALE     = 0.15
ACCELERATION_SCALE = 0.15
PLANNING_TIME      = 5.0
PLANNING_ATTEMPTS  = 10

# EEF pointing straight down  [x, y, z, w]
# CHANGE if your EEF convention differs — verify in RViz
DOWN_QUAT = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)


# ===========================================================================
# State enum
# ===========================================================================
class State(enum.Enum):
    IDLE    = "IDLE"
    PLACING = "PLACING"
    DONE    = "DONE"


# ===========================================================================
# Node
# ===========================================================================
class CubePlaceNode(Node):

    def __init__(self):
        super().__init__("cube_place_node")

        self._publish_hand_eye_transform()

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._action_client = ActionClient(self, MoveGroup, "move_action")
        self.get_logger().info("Waiting for MoveGroup action server …")
        self._action_client.wait_for_server()
        self.get_logger().info("MoveGroup connected ✓")

        self._state_lock = threading.Lock()
        self._state      = State.IDLE

        self._latest_box_pose  : PoseStamped | None = None
        self._latest_box_color : str | None         = None

        self.create_subscription(
            PoseStamped, "/box_pose_camera", self._box_pose_cb, 10
        )
        self.create_subscription(
            String, "/box_color", self._box_color_cb, 10
        )

        self.create_timer(0.5, self._tick)

        self.get_logger().info(
            f"CubePlaceNode ready — TCP_OFFSET={TCP_OFFSET} m  "
            f"MAX_PLACE_DIST={MAX_PLACE_DIST} m"
        )

    # -----------------------------------------------------------------------
    # Hand-eye transform
    # -----------------------------------------------------------------------
    def _publish_hand_eye_transform(self):
        self._static_broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = BASE_LINK
        t.child_frame_id  = "camera_depth_optical_frame"

        # CHANGE: your hand-eye calibration values
        t.transform.translation.x =  0.10901317268405636
        t.transform.translation.y =  0.004291147472906663
        t.transform.translation.z = -0.0021601816053998316
        t.transform.rotation.x    = -0.2153073817880376
        t.transform.rotation.y    =  0.4644251937597519
        t.transform.rotation.z    = -0.1916116625982454
        t.transform.rotation.w    =  0.837398914201071

        self._static_broadcaster.sendTransform(t)
        self.get_logger().info("Published static hand-eye transform")

    # -----------------------------------------------------------------------
    # Detection callbacks
    # -----------------------------------------------------------------------
    def _box_pose_cb(self, msg: PoseStamped):
        self._latest_box_pose = msg

    def _box_color_cb(self, msg: String):
        self._latest_box_color = msg.data

    # -----------------------------------------------------------------------
    # Tick — triggers place when a close-enough box is detected
    # -----------------------------------------------------------------------
    def _tick(self):
        with self._state_lock:
            if self._state != State.IDLE:
                return
            if self._latest_box_pose is None:
                return
            pose_snap              = self._latest_box_pose
            color_snap             = self._latest_box_color
            self._latest_box_pose  = None
            self._latest_box_color = None

        try:
            box_in_base: PoseStamped = self.tf_buffer.transform(
                pose_snap, BASE_LINK, timeout=Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return

        bx = box_in_base.pose.position.x
        by = box_in_base.pose.position.y
        bz = box_in_base.pose.position.z
        dist = math.sqrt(bx**2 + by**2 + bz**2)

        self.get_logger().info(
            f"Box ({color_snap}) at ({bx:.3f}, {by:.3f}, {bz:.3f}) "
            f"dist={dist:.3f} m"
        )

        if dist > MAX_PLACE_DIST:
            self.get_logger().info(
                f"Box {dist:.3f} m away — beyond {MAX_PLACE_DIST} m, ignoring"
            )
            return

        self.get_logger().info(f"Box within range ({dist:.3f} m) — starting place")

        with self._state_lock:
            self._state = State.PLACING

        threading.Thread(
            target=self._place_sequence,
            args=(box_in_base, color_snap),
            daemon=True,
        ).start()

    # -----------------------------------------------------------------------
    # MoveGroup helpers
    # -----------------------------------------------------------------------
    def _send_joint_goal(self, group_name: str, joint_names: list, positions: list):
        goal = MoveGroup.Goal()
        goal.request.group_name                      = group_name
        goal.request.num_planning_attempts           = PLANNING_ATTEMPTS
        goal.request.allowed_planning_time           = PLANNING_TIME
        goal.request.max_velocity_scaling_factor     = VELOCITY_SCALE
        goal.request.max_acceleration_scaling_factor = ACCELERATION_SCALE

        con = Constraints()
        for name, pos in zip(joint_names, positions):
            jc = JointConstraint()
            jc.joint_name      = name
            jc.position        = float(pos)
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight          = 1.0
            con.joint_constraints.append(jc)

        goal.request.goal_constraints.append(con)
        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle and handle.accepted:
            res_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_future)
            result = res_future.result()
            if result and result.result.error_code.val != 1:
                self.get_logger().warn(
                    f"Joint goal error code {result.result.error_code.val}"
                )
        else:
            self.get_logger().error("Joint goal REJECTED")

    def _move_to_pose(self, x: float, y: float, z: float, label: str):
        """
        Move the flange so that the FINGERTIPS reach (x, y, z).
        TCP_OFFSET is added to z here — callers always pass the
        intended fingertip position, never the flange position.
        """
        flange_z = z + TCP_OFFSET
        self.get_logger().info(
            f"  → {label}  fingertip=({x:.3f}, {y:.3f}, {z:.3f})  "
            f"flange z={flange_z:.3f} (fingertip + {TCP_OFFSET} m TCP)"
        )

        goal = MoveGroup.Goal()
        goal.request.group_name                      = GROUP_NAME_ARM
        goal.request.num_planning_attempts           = PLANNING_ATTEMPTS
        goal.request.allowed_planning_time           = PLANNING_TIME
        goal.request.max_velocity_scaling_factor     = VELOCITY_SCALE
        goal.request.max_acceleration_scaling_factor = ACCELERATION_SCALE

        con = Constraints(name="goal")

        pos = PositionConstraint()
        pos.header.frame_id = BASE_LINK
        pos.link_name       = EEF_LINK
        bv   = BoundingVolume()
        prim = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.002])
        target_pose         = PoseStamped()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = flange_z     # ← flange target
        bv.primitives.append(prim)
        bv.primitive_poses.append(target_pose.pose)
        pos.constraint_region = bv

        ori = OrientationConstraint()
        ori.header.frame_id           = BASE_LINK
        ori.link_name                 = EEF_LINK
        ori.orientation               = DOWN_QUAT
        ori.absolute_x_axis_tolerance = 0.1
        ori.absolute_y_axis_tolerance = 0.1
        ori.absolute_z_axis_tolerance = 0.1
        ori.weight                    = 1.0

        con.position_constraints.append(pos)
        con.orientation_constraints.append(ori)
        goal.request.goal_constraints.append(con)

        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle and handle.accepted:
            res_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_future)
            result = res_future.result()
            if result and result.result.error_code.val != 1:
                self.get_logger().warn(
                    f"Pose goal '{label}' error code "
                    f"{result.result.error_code.val} "
                    f"(1=SUCCESS -5=NO_IK -6=UNREACHABLE)"
                )
        else:
            self.get_logger().error(f"Pose goal '{label}' REJECTED")

    # -----------------------------------------------------------------------
    # Gripper / home
    # -----------------------------------------------------------------------
    def _open_gripper(self):
        self.get_logger().info("  → Opening gripper")
        self._send_joint_goal(GROUP_NAME_GRIPPER, GRIPPER_JOINT_NAMES, [GRIPPER_OPEN])

    def _close_gripper(self):
        self.get_logger().info("  → Closing gripper (safety re-grip)")
        self._send_joint_goal(GROUP_NAME_GRIPPER, GRIPPER_JOINT_NAMES, [GRIPPER_CLOSED])

    def _go_home(self):
        self.get_logger().info("  → Returning to home")
        self._send_joint_goal(GROUP_NAME_ARM, ARM_JOINT_NAMES, HOME_JOINTS)

    # -----------------------------------------------------------------------
    # Place sequence
    # -----------------------------------------------------------------------
    def _place_sequence(self, box_in_base: PoseStamped, color: str):
        bx = box_in_base.pose.position.x
        by = box_in_base.pose.position.y
        bz = box_in_base.pose.position.z   # base/floor level of box

        # Compute key heights in terms of fingertip position.
        # _move_to_pose() will add TCP_OFFSET on top of each of these.
        box_rim_z    = bz + BOX_HEIGHT_M                      # top opening of box
        hover_z      = box_rim_z + SAFE_HOVER_ABOVE_RIM       # safe approach
        release_z    = box_rim_z - RELEASE_BELOW_RIM          # inside the box

        try:
            self.get_logger().info(
                f"=== PLACE — {color} box base at ({bx:.3f}, {by:.3f}, {bz:.3f}) ==="
            )
            self.get_logger().info(
                f"    rim z={box_rim_z:.3f}  "
                f"hover fingertip z={hover_z:.3f}  "
                f"release fingertip z={release_z:.3f}  "
                f"(flange will be {TCP_OFFSET} m higher)"
            )

            # 1 — hover fingertips above box rim
            self._move_to_pose(bx, by, hover_z, "hover above box")
            time.sleep(0.3)

            # 2 — lower fingertips to release point inside box
            self._move_to_pose(bx, by, release_z, "release point")
            time.sleep(0.3)

            # 3 — open gripper — cube drops
            self._open_gripper()
            time.sleep(0.5)

            # 4 — retreat fingertips back above rim
            self._move_to_pose(bx, by, hover_z, "retreat above box")
            time.sleep(0.3)

            # 5 — return arm to home
            self._go_home()

            self.get_logger().info("=== PLACE complete ✓ ===")
            with self._state_lock:
                self._state = State.DONE

        except Exception as e:
            self.get_logger().error(f"Place sequence exception: {e}")
            self._open_gripper()   # safety: don't hold cube if something goes wrong
            with self._state_lock:
                self._state = State.IDLE


# ===========================================================================
# Entry point
# ===========================================================================
def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    node     = CubePlaceNode()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
