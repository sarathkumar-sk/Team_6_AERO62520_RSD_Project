#!/usr/bin/env python3
"""
cube_grasp_node.py
-------------------
Listens for a cube pose on /cube_pose_camera, transforms it into the robot
base frame using TF2, and executes a pick sequence with MoveIt2.

Also broadcasts the hand-eye calibration as a static transform so you no
longer need a separate terminal for that.

Subscriptions:
  /cube_pose_camera  (geometry_msgs/PoseStamped)

Requires:
  - MoveIt2 running for your MyCobot 280 Pi
  - pymoveit2 installed  (pip install pymoveit2  or build from source)
  - hand-eye calibration values filled in below
"""

import threading
import time

import rclpy
import rclpy.executors
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped transform support
from tf2_ros import StaticTransformBroadcaster

# pymoveit2 — https://github.com/AndyZe/pymoveit2
# Install: pip install pymoveit2   OR   colcon build from source
from pymoveit2 import MoveIt2


class CubeGraspNode(Node):
    def __init__(self):
        super().__init__("cube_grasp_node")

        # ------------------------------------------------------------------
        # 1. Broadcast hand-eye calibration as a static TF transform
        #    so we don't need a separate terminal for it.
        # ------------------------------------------------------------------
        self._publish_hand_eye_transform()

        # ------------------------------------------------------------------
        # 2. TF2 listener
        # ------------------------------------------------------------------
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ------------------------------------------------------------------
        # 3. MoveIt2 interface for the arm
        #
        # CHANGE joint_names: run `ros2 topic echo /joint_states` and copy
        #        the names array exactly as they appear.
        #
        # CHANGE base_link_name: the root link of your arm in the URDF.
        #        MyCobot 280 Pi is typically "base" — check your URDF.
        #
        # CHANGE end_effector_name: the tip link. Typically "joint6_flange"
        #        or "link6" for MyCobot 280 Pi — check your URDF.
        #
        # CHANGE group_name: the MoveIt2 planning group name set up in your
        #        MoveIt2 config. Often "arm" or "manipulator".
        # ------------------------------------------------------------------
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[
                "joint2_to_joint1",       # CHANGE: verify against /joint_states
                "joint3_to_joint2",
                "joint4_to_joint3",
                "joint5_to_joint4",
                "joint6_to_joint5",
                "joint6output_to_joint6",
            ],
            base_link_name=    "base",           # CHANGE: verify with your URDF
            end_effector_name= "joint6_flange",  # CHANGE: verify with your URDF
            group_name=        "arm",            # CHANGE: verify with your MoveIt2 config
            execute_via_moveit=True,
        )

        # Slow and safe for first tests — increase once validated
        self.moveit2.max_velocity_scaling_factor     = 0.2
        self.moveit2.max_acceleration_scaling_factor = 0.2

        # ------------------------------------------------------------------
        # 4. Grasp state — a lock prevents overlapping grasp attempts
        # ------------------------------------------------------------------
        self._grasp_lock       = threading.Lock()
        self._is_grasping      = False
        self._latest_cube_pose = None   # most recent detection

        # ------------------------------------------------------------------
        # 5. Subscribe to cube detections
        # ------------------------------------------------------------------
        self.create_subscription(
            PoseStamped,
            "/cube_pose_camera",
            self._cube_pose_callback,
            10,
        )

        # Check for a new pose every 0.5 s (2 Hz) and start grasp if idle
        self.create_timer(0.5, self._grasp_trigger_callback)

        self.get_logger().info("CubeGraspNode ready — waiting for cube poses on /cube_pose_camera")

    # ------------------------------------------------------------------
    # Static hand-eye transform
    # ------------------------------------------------------------------
    def _publish_hand_eye_transform(self):
        """
        Publishes the camera → robot-base static transform obtained from
        your hand-eye calibration.

        CHANGE: fill in your calibration values for translation and rotation.
                header.frame_id  = parent frame  (robot base)
                child_frame_id   = child  frame  (camera optical frame)
        """
        self._static_broadcaster = StaticTransformBroadcaster(self)  # keep as attribute!

        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = "g_base"                     # CHANGE: your robot base frame
        t.child_frame_id  = "camera_depth_optical_frame" # CHANGE: your camera frame

        # CHANGE: paste your hand-eye calibration translation (metres)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # CHANGE: paste your hand-eye calibration rotation (quaternion x y z w)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self._static_broadcaster.sendTransform(t)
        self.get_logger().info("Published static hand-eye transform (g_base → camera_depth_optical_frame)")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _cube_pose_callback(self, msg: PoseStamped):
        """Store the latest detection — do NO motion planning here."""
        self._latest_cube_pose = msg

    def _grasp_trigger_callback(self):
        """
        Fires at 2 Hz.  Spawns a grasp thread when:
          - a fresh cube pose is available, AND
          - no grasp is currently in progress.
        """
        with self._grasp_lock:
            if self._is_grasping or self._latest_cube_pose is None:
                return
            self._is_grasping      = True
            pose_snapshot          = self._latest_cube_pose
            self._latest_cube_pose = None  # consume — don't re-trigger

        # Run the motion in a daemon thread so the executor keeps spinning
        thread = threading.Thread(
            target=self._run_grasp_sequence,
            args=(pose_snapshot,),
            daemon=True,
        )
        thread.start()

    # ------------------------------------------------------------------
    # Full grasp sequence (runs in its own thread)
    # ------------------------------------------------------------------
    def _run_grasp_sequence(self, msg: PoseStamped):
        try:
            # ----------------------------------------------------------
            # Step 1 — Transform cube pose into the robot base frame
            # ----------------------------------------------------------
            try:
                cube_in_base: PoseStamped = self.tf_buffer.transform(
                    msg,
                    "g_base",                           # CHANGE: must match header.frame_id used above
                    timeout=Duration(seconds=1.0),
                )
            except Exception as e:
                self.get_logger().warn(f"TF transform failed: {e}")
                return

            cx = cube_in_base.pose.position.x
            cy = cube_in_base.pose.position.y
            cz = cube_in_base.pose.position.z
            self.get_logger().info(f"Cube in base frame:  x={cx:.3f}  y={cy:.3f}  z={cz:.3f} m")

            # ----------------------------------------------------------
            # End-effector orientation: tool Z pointing downward.
            # Quaternion [x=1, y=0, z=0, w=0] = 180° around X-axis.
            # CHANGE if your EEF's neutral orientation is different —
            # verify visually in RViz before running on real hardware.
            # ----------------------------------------------------------
            down_quat = [1.0, 0.0, 0.0, 0.0]  # [x, y, z, w]

            # ----------------------------------------------------------
            # Step 2 — Pre-grasp: hover 10 cm above the cube
            # ----------------------------------------------------------
            self.get_logger().info("Moving to pre-grasp pose …")
            self.moveit2.move_to_pose(
                position=[cx, cy, cz + 0.10],
                quat_xyzw=down_quat,
            )
            self.moveit2.execute()
            self.get_logger().info("Pre-grasp reached")
            time.sleep(0.5)

            # ----------------------------------------------------------
            # Step 3 — Descend to grasp pose (2 cm above cube top face)
            # CHANGE: adjust z offset to match your gripper finger length
            # ----------------------------------------------------------
            self.get_logger().info("Moving to grasp pose …")
            self.moveit2.move_to_pose(
                position=[cx, cy, cz + 0.02],
                quat_xyzw=down_quat,
            )
            self.moveit2.execute()
            self.get_logger().info("Grasp pose reached")
            time.sleep(0.3)

            # ----------------------------------------------------------
            # Step 4 — Close gripper
            # ----------------------------------------------------------
            self.get_logger().info("Closing gripper …")
            self._close_gripper()
            time.sleep(0.5)

            # ----------------------------------------------------------
            # Step 5 — Lift 20 cm above the cube's original position
            # ----------------------------------------------------------
            self.get_logger().info("Lifting cube …")
            self.moveit2.move_to_pose(
                position=[cx, cy, cz + 0.20],
                quat_xyzw=down_quat,
            )
            self.moveit2.execute()
            self.get_logger().info("Grasp sequence complete ✓")

        except Exception as e:
            self.get_logger().error(f"Grasp sequence failed with exception: {e}")

        finally:
            # Always release the lock so the node can attempt the next grasp
            with self._grasp_lock:
                self._is_grasping = False

    # ------------------------------------------------------------------
    # Gripper control
    # ------------------------------------------------------------------
    def _close_gripper(self):
        """
        Close the MyCobot adaptive gripper.

        CHANGE — pick the approach that matches your setup:

        ── Option A: mycobot_ros2 gripper service ──────────────────────
        from mycobot_interfaces.srv import GripperStatus  # check actual srv name
        if not hasattr(self, '_gripper_client'):
            self._gripper_client = self.create_client(GripperStatus, '/set_gripper_state')
        req = GripperStatus.Request()
        req.status = 1   # 1=close, 0=open — verify with your package docs
        req.speed  = 80  # gripper speed 0-100
        self._gripper_client.call(req)

        ── Option B: gripper as a second MoveIt2 group ─────────────────
        self.moveit2_gripper.move_to_configuration(
            {"gripper_controller_joint_name": 0.0}  # CHANGE: closed position value
        )
        self.moveit2_gripper.execute()

        ── Option C: direct topic / serial command ──────────────────────
        from std_msgs.msg import Int16
        if not hasattr(self, '_gripper_pub'):
            self._gripper_pub = self.create_publisher(Int16, '/gripper_control', 1)
        msg = Int16()
        msg.data = 1  # CHANGE: check your driver's convention
        self._gripper_pub.publish(msg)
        """
        self.get_logger().warn(
            "_close_gripper() is not implemented yet — "
            "add your gripper command (see comments above)"
        )

    def _open_gripper(self):
        """
        Open the MyCobot adaptive gripper.
        Mirror of _close_gripper() with the open value.
        CHANGE: implement alongside _close_gripper().
        """
        self.get_logger().warn(
            "_open_gripper() is not implemented yet — "
            "add your gripper command (see comments in _close_gripper)"
        )


# ------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)

    # MultiThreadedExecutor lets the grasp thread and ROS2 callbacks
    # run concurrently — essential because moveit2.execute() is blocking
    # and needs the node to keep spinning in the background.
    executor = rclpy.executors.MultiThreadedExecutor()
    node = CubeGraspNode()
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
