#!/usr/bin/env python3
"""
cube_place_node.py
-------------------
Assumes the arm is already holding a cube (gripper closed).
Subscribes to /box_pose_camera from realsense_box_publisher.py,
transforms the box position into the robot base frame, and executes
a single drop sequence into the 20×20×20 cm box.

State machine:
  IDLE  ──►  PLACING  ──►  DONE
               │ (on error)
               └──────────►  IDLE  (allows retry)

Subscriptions:
  /box_pose_camera  (geometry_msgs/PoseStamped)  — from realsense_box_publisher
  /box_color        (std_msgs/String)

Requires:
  - MoveIt2 move_group node running
  - Arm already holding cube (gripper closed) before this node starts
  - Hand-eye calibration values filled in below
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

# Home joint angles — arm returns here after placing
# CHANGE: jog to a safe resting pose and read from `ros2 topic echo /joint_states`
HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Gripper values (matching cube_grasp_node.py)
GRIPPER_OPEN   =  0.0   # CHANGE
GRIPPER_CLOSED = -0.7   # CHANGE

# ------------------------------------------------------------------
# Box geometry
# The box is 20×20×20 cm.
# realsense_box_publisher gives us the position of the nearest face
# of the box as seen by the camera (side wall or top rim depending
# on camera angle).
#
# We convert that to the box opening centre by adding BOX_HEIGHT_M
# upward in the robot base Z direction — this is where we want to
# release the cube.
#
# BOX_WALL_M is already applied inside realsense_box_publisher
# (box_half_wall_m = 0.005), so we don't double-count it here.
# ------------------------------------------------------------------
BOX_HEIGHT_M = 0.20    # full height of the box (metres)

# ------------------------------------------------------------------
# Approach offsets (in robot base Z direction)
#
# The detected box pose Z is the floor-level position of the box face.
# We add BOX_HEIGHT_M to get to the top opening, then add further
# offsets to control approach and release height.
#
#   SAFE_HOVER_ABOVE_RIM — how far above the box rim to hover before
#                          descending.  Keeps the arm clear of the
#                          box walls if the box pose has some error.
#
#   RELEASE_INSIDE_BOX   — how far below the rim to open the gripper.
#                          Positive = inside the box.
#                          E.g. 0.05 = 5 cm below the rim.
#                          Keep this small so the cube doesn't fall far.
# ------------------------------------------------------------------
SAFE_HOVER_ABOVE_RIM = 0.10   # 10 cm above box opening
RELEASE_INSIDE_BOX   = 0.05   # 5 cm below box rim (inside box)

# Only accept a box detection if it is within this distance from
# the robot base.  Prevents acting on a far-away false positive.
MAX_PLACE_DIST = 0.70   # metres  — CHANGE to suit your workspace

# Motion tuning — keep conservative while testing
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
    IDLE    = "IDLE"     # waiting for a close-enough box detection
    PLACING = "PLACING"  # place sequence running
    DONE    = "DONE"     # complete — stays here until process is restarted


# ===========================================================================
# Node
# ===========================================================================
class CubePlaceNode(Node):

    def __init__(self):
        super().__init__("cube_place_node")

        # ------------------------------------------------------------------
        # 1. Hand-eye static transform (same values as cube_grasp_node.py)
        # ------------------------------------------------------------------
        self._publish_hand_eye_transform()

        # ------------------------------------------------------------------
        # 2. TF2
        # ------------------------------------------------------------------
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ------------------------------------------------------------------
        # 3. MoveGroup action client
        # ------------------------------------------------------------------
        self._action_client = ActionClient(self, MoveGroup, "move_action")
        self.get_logger().info("Waiting for MoveGroup action server …")
        self._action_client.wait_for_server()
        self.get_logger().info("MoveGroup connected ✓")

        # ------------------------------------------------------------------
        # 4. State
        # ------------------------------------------------------------------
        self._state_lock = threading.Lock()
        self._state      = State.IDLE

        self._latest_box_pose  : PoseStamped | None = None
        self._latest_box_color : str | None         = None

        # ------------------------------------------------------------------
        # 5. Subscriptions
        # ------------------------------------------------------------------
        self.create_subscription(
            PoseStamped, "/box_pose_camera", self._box_pose_cb, 10
        )
        self.create_subscription(
            String, "/box_color", self._box_color_cb, 10
        )

        # 2 Hz trigger timer
        self.create_timer(0.5, self._tick)

        self.get_logger().info(
            f"CubePlaceNode ready — will place into box within {MAX_PLACE_DIST} m"
        )

    # -----------------------------------------------------------------------
    # Hand-eye transform — CHANGE values to match your calibration
    # -----------------------------------------------------------------------
    def _publish_hand_eye_transform(self):
        self._static_broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = BASE_LINK
        t.child_frame_id  = "camera_depth_optical_frame"

        # CHANGE: paste your hand-eye calibration values
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
    # Tick (2 Hz) — triggers place when a close-enough box is seen
    # -----------------------------------------------------------------------
    def _tick(self):
        with self._state_lock:
            if self._state != State.IDLE:
                return
            if self._latest_box_pose is None:
                return
            pose_snap              = self._latest_box_pose
            color_snap             = self._latest_box_color
            self._latest_box_pose  = None   # consume
            self._latest_box_color = None

        # Transform to base frame first so we can measure real distance
        try:
            box_in_base: PoseStamped = self.tf_buffer.transform(
                pose_snap,
                BASE_LINK,
                timeout=Duration(seconds=1.0),
            )
        except Exception as e:
            self.get_logger().warn(f"TF transform failed in tick: {e}")
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
                f"Box is {dist:.3f} m away — beyond {MAX_PLACE_DIST} m threshold, ignoring"
            )
            return

        self.get_logger().info(
            f"Box within range ({dist:.3f} m) — starting place sequence"
        )

        with self._state_lock:
            self._state = State.PLACING

        thread = threading.Thread(
            target=self._place_sequence,
            args=(box_in_base, color_snap),
            daemon=True,
        )
        thread.start()

    # -----------------------------------------------------------------------
    # MoveGroup helpers (identical pattern to cube_grasp_node.py)
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
            if result:
                code = result.result.error_code.val
                if code != 1:
                    self.get_logger().warn(f"Joint goal finished with error code {code}")
        else:
            self.get_logger().error("Joint goal was REJECTED by MoveGroup")

    def _move_to_pose(self, x: float, y: float, z: float, label: str):
        self.get_logger().info(f"  → Moving to {label} ({x:.3f}, {y:.3f}, {z:.3f})")

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
        target_pose.pose.position.z = z
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
            if result:
                code = result.result.error_code.val
                if code != 1:
                    self.get_logger().warn(
                        f"Pose goal '{label}' finished with error code {code} "
                        f"(1=SUCCESS, -5=NO_IK, -6=UNREACHABLE)"
                    )
        else:
            self.get_logger().error(f"Pose goal '{label}' was REJECTED by MoveGroup")

    # -----------------------------------------------------------------------
    # Gripper / home helpers
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
    # Place sequence — runs once in its own thread
    # -----------------------------------------------------------------------
    def _place_sequence(self, box_in_base: PoseStamped, color: str):
        bx = box_in_base.pose.position.x
        by = box_in_base.pose.position.y
        bz = box_in_base.pose.position.z   # floor / base of box face from camera

        # The box publisher gives us the nearest face centre.
        # Since the camera is likely looking at the side of the box,
        # bz in the robot base frame corresponds to roughly floor level.
        # The box opening is at bz + BOX_HEIGHT_M.
        box_top_z    = bz + BOX_HEIGHT_M
        hover_z      = box_top_z + SAFE_HOVER_ABOVE_RIM     # safe approach height
        release_z    = box_top_z - RELEASE_INSIDE_BOX       # inside the box

        try:
            self.get_logger().info(
                f"=== PLACE sequence — {color} box at "
                f"({bx:.3f}, {by:.3f}, {bz:.3f}) ==="
            )
            self.get_logger().info(
                f"    box top z={box_top_z:.3f}  "
                f"hover z={hover_z:.3f}  "
                f"release z={release_z:.3f}"
            )

            # 1 — Hover above box opening at safe height
            self._move_to_pose(bx, by, hover_z, "hover above box")
            time.sleep(0.3)

            # 2 — Lower to release height (inside box)
            self._move_to_pose(bx, by, release_z, "release point")
            time.sleep(0.3)

            # 3 — Open gripper — cube drops into box
            self._open_gripper()
            time.sleep(0.5)

            # 4 — Retreat straight back up out of box
            self._move_to_pose(bx, by, hover_z, "retreat above box")
            time.sleep(0.3)

            # 5 — Return arm to home
            self._go_home()

            self.get_logger().info("=== PLACE complete ✓ ===")

            with self._state_lock:
                self._state = State.DONE

        except Exception as e:
            self.get_logger().error(f"Place sequence exception: {e}")
            # Safety: open gripper so cube doesn't stay trapped if arm stalls
            self._open_gripper()
            with self._state_lock:
                self._state = State.IDLE   # allow retry


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
