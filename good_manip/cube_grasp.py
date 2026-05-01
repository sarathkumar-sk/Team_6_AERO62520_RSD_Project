#!/usr/bin/env python3
"""
cube_grasp_node.py
-------------------
Detects a cube via /cube_pose_camera and executes a single pick sequence
if the cube is within MAX_GRASP_DIST metres of the robot base.

TCP offset
----------
There is 10 cm between joint6_flange (what MoveIt2 controls) and the
actual fingertip grip point.  Every Cartesian target is therefore raised
by TCP_OFFSET so that the FINGERS reach the intended position, not the
flange.

  flange_target_z = intended_grip_z + TCP_OFFSET

All offsets below are defined relative to the GRIP POINT (intuitive),
and TCP_OFFSET is added automatically in _move_to_pose().

State machine:
  IDLE  ──►  PICKING  ──►  DONE
               │ (on error)
               └──────────►  IDLE  (allows retry)

Subscriptions:
  /cube_pose_camera  (geometry_msgs/PoseStamped)
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

# Home joint angles (radians)
# CHANGE: jog to a safe resting pose and read from `ros2 topic echo /joint_states`
HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

GRIPPER_OPEN   =  0.0   # CHANGE
GRIPPER_CLOSED = -0.7   # CHANGE

# ------------------------------------------------------------------
# TCP (Tool Centre Point) offset
# Distance from joint6_flange to the actual fingertip grip point.
# MoveIt2 moves joint6_flange to the commanded position, so every
# target must be raised by this amount so the FINGERS reach the
# intended location.
#
# CHANGE if your gripper is a different length.
# ------------------------------------------------------------------
TCP_OFFSET = 0.10   # 10 cm

# Only attempt a grasp if the cube is closer than this (metres)
MAX_GRASP_DIST = 0.25   # 25 cm

# Motion tuning
VELOCITY_SCALE     = 0.15
ACCELERATION_SCALE = 0.15
PLANNING_TIME      = 5.0
PLANNING_ATTEMPTS  = 10

# ------------------------------------------------------------------
# Grasp offsets — all defined relative to the CUBE CENTRE (grip point).
# TCP_OFFSET is added automatically inside _move_to_pose().
# ------------------------------------------------------------------
PRE_GRASP_Z_OFFSET = 0.10   # fingertip hovers 10 cm above cube before descent
GRASP_Z_OFFSET     = 0.07   # fingertip descends to 7 cm above cube centre
LIFT_Z_OFFSET      = 0.20   # fingertip lifts to 20 cm above cube after gripping

# EEF pointing straight down  [x, y, z, w]
# CHANGE if your EEF convention differs — verify in RViz
DOWN_QUAT = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)


# ===========================================================================
# State enum
# ===========================================================================
class State(enum.Enum):
    IDLE    = "IDLE"
    PICKING = "PICKING"
    DONE    = "DONE"


# ===========================================================================
# Node
# ===========================================================================
class CubeGraspNode(Node):

    def __init__(self):
        super().__init__("cube_grasp_node")

        self._publish_hand_eye_transform()

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._action_client = ActionClient(self, MoveGroup, "move_action")
        self.get_logger().info("Waiting for MoveGroup action server …")
        self._action_client.wait_for_server()
        self.get_logger().info("MoveGroup connected ✓")

        self._state_lock = threading.Lock()
        self._state      = State.IDLE

        self._latest_cube_pose: PoseStamped | None = None

        self.create_subscription(
            PoseStamped, "/cube_pose_camera", self._cube_pose_cb, 10
        )
        self.create_timer(0.5, self._tick)

        self.get_logger().info(
            f"CubeGraspNode ready — TCP_OFFSET={TCP_OFFSET} m  "
            f"MAX_GRASP_DIST={MAX_GRASP_DIST} m"
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
    # Detection callback
    # -----------------------------------------------------------------------
    def _cube_pose_cb(self, msg: PoseStamped):
        self._latest_cube_pose = msg

    # -----------------------------------------------------------------------
    # Tick — triggers pick when cube is close enough
    # -----------------------------------------------------------------------
    def _tick(self):
        with self._state_lock:
            if self._state != State.IDLE:
                return
            if self._latest_cube_pose is None:
                return
            pose_snap              = self._latest_cube_pose
            self._latest_cube_pose = None

        try:
            cube_in_base: PoseStamped = self.tf_buffer.transform(
                pose_snap, BASE_LINK, timeout=Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return

        cx = cube_in_base.pose.position.x
        cy = cube_in_base.pose.position.y
        cz = cube_in_base.pose.position.z
        dist = math.sqrt(cx**2 + cy**2 + cz**2)

        self.get_logger().info(
            f"Cube at ({cx:.3f}, {cy:.3f}, {cz:.3f}) — dist={dist:.3f} m"
        )

        if dist > MAX_GRASP_DIST:
            self.get_logger().info(
                f"Cube {dist:.3f} m away — beyond {MAX_GRASP_DIST} m, ignoring"
            )
            return

        self.get_logger().info(f"Cube within range ({dist:.3f} m) — starting pick")

        with self._state_lock:
            self._state = State.PICKING

        threading.Thread(
            target=self._pick_sequence, args=(cube_in_base,), daemon=True
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
        intended grip position, never the flange position.
        """
        flange_z = z + TCP_OFFSET
        self.get_logger().info(
            f"  → {label}  grip=({x:.3f}, {y:.3f}, {z:.3f})  "
            f"flange z={flange_z:.3f} (grip + {TCP_OFFSET} m TCP)"
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
        self.get_logger().info("  → Closing gripper")
        self._send_joint_goal(GROUP_NAME_GRIPPER, GRIPPER_JOINT_NAMES, [GRIPPER_CLOSED])

    def _go_home(self):
        self.get_logger().info("  → Returning to home")
        self._send_joint_goal(GROUP_NAME_ARM, ARM_JOINT_NAMES, HOME_JOINTS)

    # -----------------------------------------------------------------------
    # Pick sequence
    # -----------------------------------------------------------------------
    def _pick_sequence(self, cube_in_base: PoseStamped):
        # These are the GRIP POINT positions (fingertip, not flange).
        # _move_to_pose() automatically adds TCP_OFFSET to reach the flange.
        cx = cube_in_base.pose.position.x
        cy = cube_in_base.pose.position.y
        cz = cube_in_base.pose.position.z

        try:
            self.get_logger().info(
                f"=== PICK — cube grip point ({cx:.3f}, {cy:.3f}, {cz:.3f}) ==="
            )

            # 1 — open gripper before approaching
            self._open_gripper()
            time.sleep(0.4)

            # 2 — hover fingertips PRE_GRASP_Z_OFFSET above cube
            self._move_to_pose(cx, cy, cz + PRE_GRASP_Z_OFFSET, "pre-grasp")
            time.sleep(0.3)

            # 3 — descend fingertips to GRASP_Z_OFFSET above cube
            self._move_to_pose(cx, cy, cz + GRASP_Z_OFFSET, "grasp")
            time.sleep(0.3)

            # 4 — grip
            self._close_gripper()
            time.sleep(0.6)

            # 5 — lift fingertips LIFT_Z_OFFSET above cube
            self._move_to_pose(cx, cy, cz + LIFT_Z_OFFSET, "lift")
            time.sleep(0.3)

            # 6 — return home
            self._go_home()

            self.get_logger().info("=== PICK complete ✓ ===")
            with self._state_lock:
                self._state = State.DONE

        except Exception as e:
            self.get_logger().error(f"Pick sequence exception: {e}")
            self._open_gripper()
            with self._state_lock:
                self._state = State.IDLE


# ===========================================================================
# Entry point
# ===========================================================================
def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    node     = CubeGraspNode()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
