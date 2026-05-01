#!/usr/bin/env python3
"""
cube_grasp_node.py
-------------------
Detects a cube via /cube_pose_camera, and if it is within MAX_GRASP_DIST
metres of the robot base, executes a single pick sequence.

Motion uses the raw MoveGroup action client (same pattern as
controller_that_worked.py) — no pymoveit2 required.

State machine:
  IDLE  ──►  PICKING  ──►  DONE
                │ (on error)
                └──────────►  IDLE

Subscriptions:
  /cube_pose_camera  (geometry_msgs/PoseStamped)

Requires:
  - MoveIt2 move_group node running
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
import tf2_geometry_msgs          # noqa: F401 — registers PoseStamped support
from tf2_ros import StaticTransformBroadcaster


# ===========================================================================
# CONFIGURATION — edit these to match your setup
# ===========================================================================

GROUP_NAME_ARM     = "arm_group"       # CHANGE: verify in your .srdf
GROUP_NAME_GRIPPER = "gripper"         # CHANGE: verify in your .srdf

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

# Home position joint angles (radians)
# CHANGE: jog the arm to a safe resting pose and read from `ros2 topic echo /joint_states`
HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Gripper joint values
GRIPPER_OPEN   =  0.0   # CHANGE: open position (from controller_that_worked.py)
GRIPPER_CLOSED = -0.7   # CHANGE: closed position (from controller_that_worked.py)

# Only attempt a grasp if the cube is closer than this distance (metres)
# from the robot base origin.
MAX_GRASP_DIST = 0.25   # 25 cm

# Motion tuning
VELOCITY_SCALE     = 0.15
ACCELERATION_SCALE = 0.15
PLANNING_TIME      = 5.0
PLANNING_ATTEMPTS  = 10

# Grasp offsets (metres, in robot base Z direction)
PRE_GRASP_Z_OFFSET = 0.10   # hover above cube before descending
GRASP_Z_OFFSET     = 0.07   # descend to this height above cube centre
LIFT_Z_OFFSET      = 0.20   # lift to this height after closing gripper

# EEF orientation: tool pointing straight down
# Quaternion [x=1, y=0, z=0, w=0] = 180° rotation around X
# CHANGE if your EEF convention differs — verify in RViz
DOWN_QUAT = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)


# ===========================================================================
# State enum
# ===========================================================================
class State(enum.Enum):
    IDLE    = "IDLE"     # waiting for a close-enough cube detection
    PICKING = "PICKING"  # pick sequence running in background thread
    DONE    = "DONE"     # pick complete — stays here until restarted


# ===========================================================================
# Node
# ===========================================================================
class CubeGraspNode(Node):

    def __init__(self):
        super().__init__("cube_grasp_node")

        # ------------------------------------------------------------------
        # 1. Hand-eye static transform
        # ------------------------------------------------------------------
        self._publish_hand_eye_transform()

        # ------------------------------------------------------------------
        # 2. TF2
        # ------------------------------------------------------------------
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ------------------------------------------------------------------
        # 3. MoveGroup action client (same as controller_that_worked.py)
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

        # ------------------------------------------------------------------
        # 5. Subscription
        # ------------------------------------------------------------------
        self.create_subscription(
            PoseStamped,
            "/cube_pose_camera",
            self._cube_pose_cb,
            10,
        )

        # 2 Hz trigger timer
        self.create_timer(0.5, self._tick)

        self.get_logger().info(
            f"CubeGraspNode ready — will grasp cubes within {MAX_GRASP_DIST} m"
        )

        self._latest_cube_pose: PoseStamped | None = None

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
    # Detection callback — just store latest
    # -----------------------------------------------------------------------
    def _cube_pose_cb(self, msg: PoseStamped):
        self._latest_cube_pose = msg

    # -----------------------------------------------------------------------
    # Tick (2 Hz) — decides whether to launch a pick
    # -----------------------------------------------------------------------
    def _tick(self):
        with self._state_lock:
            if self._state != State.IDLE:
                return
            if self._latest_cube_pose is None:
                return
            pose_snap              = self._latest_cube_pose
            self._latest_cube_pose = None   # consume

        # Transform to base frame to measure actual distance
        try:
            cube_in_base: PoseStamped = self.tf_buffer.transform(
                pose_snap,
                BASE_LINK,
                timeout=Duration(seconds=1.0),
            )
        except Exception as e:
            self.get_logger().warn(f"TF transform failed in tick: {e}")
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
                f"Cube is {dist:.3f} m away — beyond {MAX_GRASP_DIST} m threshold, ignoring"
            )
            return

        self.get_logger().info(f"Cube is within range ({dist:.3f} m) — starting pick")

        with self._state_lock:
            self._state = State.PICKING

        thread = threading.Thread(
            target=self._pick_sequence,
            args=(cube_in_base,),
            daemon=True,
        )
        thread.start()

    # -----------------------------------------------------------------------
    # MoveGroup helpers (directly from controller_that_worked.py)
    # -----------------------------------------------------------------------
    def _send_joint_goal(self, group_name: str, joint_names: list, positions: list):
        """Send a joint-space goal and block until complete."""
        goal = MoveGroup.Goal()
        goal.request.group_name                     = group_name
        goal.request.num_planning_attempts          = PLANNING_ATTEMPTS
        goal.request.allowed_planning_time          = PLANNING_TIME
        goal.request.max_velocity_scaling_factor    = VELOCITY_SCALE
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
        """Send a Cartesian pose goal and block until complete."""
        self.get_logger().info(f"  → Moving to {label} ({x:.3f}, {y:.3f}, {z:.3f})")

        goal = MoveGroup.Goal()
        goal.request.group_name                     = GROUP_NAME_ARM
        goal.request.num_planning_attempts          = PLANNING_ATTEMPTS
        goal.request.allowed_planning_time          = PLANNING_TIME
        goal.request.max_velocity_scaling_factor    = VELOCITY_SCALE
        goal.request.max_acceleration_scaling_factor = ACCELERATION_SCALE

        con = Constraints(name="goal")

        # Position constraint — 2 mm sphere (same as controller_that_worked.py)
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

        # Orientation constraint — tool pointing down
        ori = OrientationConstraint()
        ori.header.frame_id              = BASE_LINK
        ori.link_name                    = EEF_LINK
        ori.orientation                  = DOWN_QUAT
        ori.absolute_x_axis_tolerance    = 0.1
        ori.absolute_y_axis_tolerance    = 0.1
        ori.absolute_z_axis_tolerance    = 0.1
        ori.weight                       = 1.0

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
    # Gripper helpers
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
    # Pick sequence — runs once in its own thread
    # -----------------------------------------------------------------------
    def _pick_sequence(self, cube_in_base: PoseStamped):
        cx = cube_in_base.pose.position.x
        cy = cube_in_base.pose.position.y
        cz = cube_in_base.pose.position.z

        try:
            self.get_logger().info(
                f"=== PICK sequence — cube at ({cx:.3f}, {cy:.3f}, {cz:.3f}) ==="
            )

            # 1 — open gripper first so we don't knock the cube on approach
            self._open_gripper()
            time.sleep(0.4)

            # 2 — hover above cube
            self._move_to_pose(cx, cy, cz + PRE_GRASP_Z_OFFSET, "pre-grasp")
            time.sleep(0.3)

            # 3 — descend to grasp height
            self._move_to_pose(cx, cy, cz + GRASP_Z_OFFSET, "grasp")
            time.sleep(0.3)

            # 4 — close gripper
            self._close_gripper()
            time.sleep(0.6)

            # 5 — lift
            self._move_to_pose(cx, cy, cz + LIFT_Z_OFFSET, "lift")
            time.sleep(0.3)

            # 6 — return home
            self._go_home()

            self.get_logger().info("=== PICK complete ✓ ===")

            with self._state_lock:
                self._state = State.DONE

        except Exception as e:
            self.get_logger().error(f"Pick sequence exception: {e}")
            self._open_gripper()   # safety: don't stay closed
            with self._state_lock:
                self._state = State.IDLE   # allow retry


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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
