#!/usr/bin/env python3
"""
cube_grasp_node.py
-------------------
Full pick-and-place state machine:

  IDLE  ──►  PICKING  ──►  SEEKING_BOX  ──►  PLACING  ──►  IDLE

1. Waits for a cube detection on /cube_pose_camera.
2. Records the cube's colour from /cube_color.
3. Picks the cube with the MyCobot 280 Pi arm.
4. Waits for a box detection on /box_pose_camera whose colour matches the cube.
5. Moves to the box and drops the cube inside.
6. Returns to the IDLE home position ready for the next pick.

Also broadcasts the hand-eye calibration as a static transform so you no
longer need a separate terminal for that.

Subscriptions:
  /cube_pose_camera  (geometry_msgs/PoseStamped)
  /cube_color        (std_msgs/String)
  /box_pose_camera   (geometry_msgs/PoseStamped)
  /box_color         (std_msgs/String)

Requires:
  - MoveIt2 running for your MyCobot 280 Pi
  - pymoveit2 installed
  - hand-eye calibration values filled in below
"""

import enum
import threading
import time

import rclpy
import rclpy.executors
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs          # noqa: F401  registers PoseStamped support
from tf2_ros import StaticTransformBroadcaster

from pymoveit2 import MoveIt2     # pip install pymoveit2


# ---------------------------------------------------------------------------
# State machine enum
# ---------------------------------------------------------------------------
class State(enum.Enum):
    IDLE        = "IDLE"         # waiting for a cube detection
    PICKING     = "PICKING"      # arm is executing the pick sequence
    SEEKING_BOX = "SEEKING_BOX" # cube in hand, waiting for matching box pose
    PLACING     = "PLACING"      # arm is executing the place / drop sequence


class CubeGraspNode(Node):

    # -----------------------------------------------------------------------
    # Initialisation
    # -----------------------------------------------------------------------
    def __init__(self):
        super().__init__("cube_grasp_node")

        # ------------------------------------------------------------------
        # 1. Broadcast hand-eye calibration as a static TF transform
        # ------------------------------------------------------------------
        self._publish_hand_eye_transform()

        # ------------------------------------------------------------------
        # 2. TF2
        # ------------------------------------------------------------------
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ------------------------------------------------------------------
        # 3. MoveIt2
        #
        # CHANGE joint_names   — run `ros2 topic echo /joint_states`
        # CHANGE base_link_name       — root link in your URDF
        # CHANGE end_effector_name    — tip link in your URDF
        # CHANGE group_name           — MoveIt2 planning group (.srdf)
        # ------------------------------------------------------------------
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[
                "joint2_to_joint1",         # CHANGE
                "joint3_to_joint2",
                "joint4_to_joint3",
                "joint5_to_joint4",
                "joint6_to_joint5",
                "joint6output_to_joint6",
            ],
            base_link_name    ="g_base",          # CHANGE
            end_effector_name ="joint6_flange", # CHANGE
            group_name        ="arm",           # CHANGE
            execute_via_moveit=True,
        )
        self.moveit2.max_velocity_scaling_factor     = 0.2   # increase when confident
        self.moveit2.max_acceleration_scaling_factor = 0.2

        # ------------------------------------------------------------------
        # 4. State machine
        # ------------------------------------------------------------------
        self._state_lock  = threading.Lock()
        self._state       = State.IDLE

        # Latest detections (written from ROS callbacks, read from threads)
        self._latest_cube_pose  : PoseStamped | None = None
        self._latest_cube_color : str | None         = None
        self._latest_box_pose   : PoseStamped | None = None
        self._latest_box_color  : str | None         = None

        # The colour of the cube currently being carried
        self._carried_cube_color : str | None = None

        # ------------------------------------------------------------------
        # 5. Subscriptions
        # ------------------------------------------------------------------
        self.create_subscription(PoseStamped, "/cube_pose_camera",
                                 self._cube_pose_cb,  10)
        self.create_subscription(String,      "/cube_color",
                                 self._cube_color_cb, 10)
        self.create_subscription(PoseStamped, "/box_pose_camera",
                                 self._box_pose_cb,   10)
        self.create_subscription(String,      "/box_color",
                                 self._box_color_cb,  10)

        # ------------------------------------------------------------------
        # 6. 2 Hz trigger — decides when to launch each action thread
        # ------------------------------------------------------------------
        self.create_timer(0.5, self._state_machine_tick)

        # Periodic status log (every 5 s)
        self.create_timer(5.0, self._log_state)

        self.get_logger().info("CubeGraspNode ready")

    # -----------------------------------------------------------------------
    # Static hand-eye transform
    # -----------------------------------------------------------------------
    def _publish_hand_eye_transform(self):
        """
        Broadcasts the camera → robot-base static TF transform.

        CHANGE: fill in your hand-eye calibration values.
                header.frame_id = parent (robot base frame)
                child_frame_id  = child  (camera optical frame)
        """
        self._static_broadcaster = StaticTransformBroadcaster(self)

        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = "g_base"                       # CHANGE — robot base frame
        t.child_frame_id  = "camera_depth_optical_frame" # CHANGE — camera frame

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
        self.get_logger().info("Published static hand-eye transform")

    # -----------------------------------------------------------------------
    # Detection callbacks — just store the latest values
    # -----------------------------------------------------------------------
    def _cube_pose_cb(self, msg: PoseStamped):
        self._latest_cube_pose  = msg

    def _cube_color_cb(self, msg: String):
        self._latest_cube_color = msg.data

    def _box_pose_cb(self, msg: PoseStamped):
        self._latest_box_pose  = msg

    def _box_color_cb(self, msg: String):
        self._latest_box_color = msg.data

    # -----------------------------------------------------------------------
    # State machine tick (2 Hz timer — runs on executor thread, non-blocking)
    # -----------------------------------------------------------------------
    def _state_machine_tick(self):
        with self._state_lock:
            current_state = self._state

        if current_state == State.IDLE:
            # Need both a pose and a colour before we can pick
            if self._latest_cube_pose is not None and self._latest_cube_color is not None:
                pose_snap  = self._latest_cube_pose
                color_snap = self._latest_cube_color
                self._latest_cube_pose  = None
                self._latest_cube_color = None
                self._transition(State.PICKING)
                self._spawn(self._pick_sequence, pose_snap, color_snap)

        elif current_state == State.SEEKING_BOX:
            # Accept a box pose only if its colour matches the cube we're holding
            if (self._latest_box_pose  is not None and
                    self._latest_box_color is not None and
                    self._latest_box_color == self._carried_cube_color):

                pose_snap = self._latest_box_pose
                self._latest_box_pose  = None
                self._latest_box_color = None
                self._transition(State.PLACING)
                self._spawn(self._place_sequence, pose_snap)

            elif self._latest_box_color is not None and \
                    self._latest_box_color != self._carried_cube_color:
                # Detected a box but wrong colour — ignore silently
                self.get_logger().debug(
                    f"Box colour '{self._latest_box_color}' does not match "
                    f"cube colour '{self._carried_cube_color}' — ignoring"
                )
                self._latest_box_pose  = None
                self._latest_box_color = None

    def _log_state(self):
        self.get_logger().info(f"State: {self._state.value}  "
                               f"| carrying: {self._carried_cube_color or 'nothing'}")

    # -----------------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------------
    def _transition(self, new_state: State):
        with self._state_lock:
            self.get_logger().info(f"State: {self._state.value} → {new_state.value}")
            self._state = new_state

    def _spawn(self, fn, *args):
        """Run a blocking motion sequence in a daemon thread."""
        t = threading.Thread(target=fn, args=args, daemon=True)
        t.start()

    def _transform_pose(self, msg: PoseStamped, target_frame: str) -> PoseStamped | None:
        try:
            return self.tf_buffer.transform(
                msg, target_frame, timeout=Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().warn(f"TF transform → '{target_frame}' failed: {e}")
            return None

    def _move_and_execute(self, position, quat_xyzw):
        """Helper: plan and execute one Cartesian target."""
        self.moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw)
        self.moveit2.execute()

    # -----------------------------------------------------------------------
    # Pick sequence  (runs in its own thread)
    # -----------------------------------------------------------------------
    def _pick_sequence(self, cube_pose: PoseStamped, cube_color: str):
        try:
            self.get_logger().info(f"=== PICK sequence starting (colour: {cube_color}) ===")

            # Transform cube pose to robot base frame
            # CHANGE: "base" must match your robot's base_link frame
            cube_in_base = self._transform_pose(cube_pose, "g_base")
            if cube_in_base is None:
                self.get_logger().error("Could not transform cube pose — aborting pick")
                self._transition(State.IDLE)
                return

            cx = cube_in_base.pose.position.x
            cy = cube_in_base.pose.position.y
            cz = cube_in_base.pose.position.z
            self.get_logger().info(f"Cube in base: x={cx:.3f} y={cy:.3f} z={cz:.3f}")

            # EEF pointing straight down
            # CHANGE: verify in RViz — depends on your URDF EEF convention
            down = [1.0, 0.0, 0.0, 0.0]   # quaternion [x, y, z, w]

            # 1 — Open gripper before approaching
            self.get_logger().info("Opening gripper")
            self._open_gripper()
            time.sleep(0.4)

            # 2 — Pre-grasp: 10 cm above cube
            self.get_logger().info("Moving to pre-grasp …")
            self._move_and_execute([cx, cy, cz + 0.10], down)
            time.sleep(0.3)

            # 3 — Descend to grasp height
            # CHANGE: adjust +0.07 z offset to match your gripper finger length
            self.get_logger().info("Descending to grasp pose …")
            self._move_and_execute([cx, cy, cz + 0.07], down)
            time.sleep(0.3)

            # 4 — Close gripper
            self.get_logger().info("Closing gripper")
            self._close_gripper()
            time.sleep(0.6)

            # 5 — Lift straight up
            self.get_logger().info("Lifting …")
            self._move_and_execute([cx, cy, cz + 0.20], down)
            time.sleep(0.3)

            # Record what we're carrying and transition
            self._carried_cube_color = cube_color
            self.get_logger().info(f"=== PICK complete — now seeking '{cube_color}' box ===")
            self._transition(State.SEEKING_BOX)

        except Exception as e:
            self.get_logger().error(f"Pick sequence exception: {e}")
            self._open_gripper()
            self._transition(State.IDLE)

    # -----------------------------------------------------------------------
    # Place (drop) sequence  (runs in its own thread)
    # -----------------------------------------------------------------------
    def _place_sequence(self, box_pose: PoseStamped):
        try:
            self.get_logger().info(f"=== PLACE sequence starting (box: {self._carried_cube_color}) ===")

            # Transform box pose to robot base frame
            # CHANGE: "base" must match your robot's base_link frame
            box_in_base = self._transform_pose(box_pose, "g_base")
            if box_in_base is None:
                self.get_logger().error("Could not transform box pose — aborting place")
                self._transition(State.SEEKING_BOX)   # keep trying
                return

            bx = box_in_base.pose.position.x
            by = box_in_base.pose.position.y
            bz = box_in_base.pose.position.z

            self.get_logger().info(f"Box in base: x={bx:.3f} y={by:.3f} z={bz:.3f}")

            # EEF pointing straight down
            # CHANGE: same orientation convention as pick sequence
            down = [1.0, 0.0, 0.0, 0.0]

            # ------------------------------------------------------------------
            # Approach heights
            #
            # The detected pose Z is the depth to the nearest face of the box
            # (likely its outer side wall when the camera faces it, or the top
            # rim when the camera is above).
            #
            # We use two offsets measured in the robot base Z direction:
            #
            #   pre_place_z_offset  — hover above the box top rim before
            #                         committing to descent (safety margin)
            #
            #   drop_z_offset       — height above the detected box face at
            #                         which we open the gripper.
            #                         Should be INSIDE the box:
            #                           bz + (box_height/2) puts EEF at mid-box
            #
            # CHANGE: both values to suit your box geometry and robot config.
            # ------------------------------------------------------------------
            pre_place_z_offset = 0.15   # 25 cm above detected box face centre
            drop_z_offset      = 0.08   # 8 cm above — should be inside the box

            # 1 — Move to position directly above the box at safe height
            self.get_logger().info("Moving to pre-place pose …")
            self._move_and_execute([bx, by, bz + pre_place_z_offset], down)
            time.sleep(0.3)

            # # 2 — Lower to drop position (inside box)
            # self.get_logger().info("Lowering into box …")
            # self._move_and_execute([bx, by, bz + drop_z_offset], down)
            # time.sleep(0.3)

            # 3 — Open gripper → cube drops
            self.get_logger().info("Opening gripper — dropping cube")
            self._open_gripper()
            time.sleep(0.5)

            # 4 — Lift back out of the box (important — avoids knocking the box)
            self.get_logger().info("Retreating from box …")
            self._move_and_execute([bx, by, bz + pre_place_z_offset], down)
            time.sleep(0.3)

            # 5 — Return to a safe home / neutral position
            self.get_logger().info("Returning to home position …")
            self._go_home()
            time.sleep(0.3)

            self._carried_cube_color = None
            self.get_logger().info("=== PLACE complete — returning to IDLE ===")
            self._transition(State.IDLE)

        except Exception as e:
            self.get_logger().error(f"Place sequence exception: {e}")
            self._open_gripper()   # safety: don't stay closed on failure
            self._transition(State.IDLE)

    # -----------------------------------------------------------------------
    # Home position
    # -----------------------------------------------------------------------
    def _go_home(self):
        """
        Move the arm to a safe resting configuration.

        CHANGE: replace with your robot's actual safe home joint angles
                (in radians), or replace with a Cartesian home pose.

        You can find good joint values by jogging the arm to a safe pose and
        running: ros2 topic echo /joint_states
        """
        # CHANGE: these joint angles are a placeholder (all-zero = likely
        #         NOT a safe home for MyCobot 280 Pi — verify with your URDF)
        home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.moveit2.move_to_configuration(home_joints)
        self.moveit2.execute()

    # -----------------------------------------------------------------------
    # Gripper control
    # -----------------------------------------------------------------------
    def _close_gripper(self):
        """
        Close the MyCobot adaptive gripper.

        CHANGE — uncomment and fill in the approach that matches your setup:

        ── Option A: mycobot_ros2 gripper service ───────────────────────────
        from mycobot_interfaces.srv import GripperStatus  # verify srv name
        if not hasattr(self, '_gripper_cli'):
            self._gripper_cli = self.create_client(
                GripperStatus, '/set_gripper_state')
        req        = GripperStatus.Request()
        req.status = 1    # 1 = close
        req.speed  = 80   # 0–100
        self._gripper_cli.call(req)

        Probably not this one
        ── Option B: gripper as a MoveIt2 planning group ───────────────────
        self.moveit2_gripper.move_to_configuration([0.0])  # CHANGE: closed value
        self.moveit2_gripper.execute()

        ── Option C: direct topic command ──────────────────────────────────
        from std_msgs.msg import Int16
        if not hasattr(self, '_gripper_pub'):
            self._gripper_pub = self.create_publisher(Int16, '/gripper_control', 1)
        msg      = Int16()
        msg.data = 1       # CHANGE: check your driver's convention
        self._gripper_pub.publish(msg)
        """
        self.get_logger().warn("_close_gripper() not implemented — add your gripper command")

    def _open_gripper(self):
        """
        Open the MyCobot adaptive gripper.
        CHANGE: mirror of _close_gripper() with the open value.
        """
        self.get_logger().warn("Opening gripper")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
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
