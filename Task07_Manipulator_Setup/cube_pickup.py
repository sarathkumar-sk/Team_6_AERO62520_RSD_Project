#!/usr/bin/env python3
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped

from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException
from tf2_geometry_msgs import do_transform_pose

from pymoveit2 import MoveIt2
from pymycobot.mycobot import MyCobot


class CubeGraspNode(Node):
    """
    Single node that:
      1. Listens to TF for the camera → robot base transform (from easyhandeye2)
      2. Subscribes to the raw detected cube pose in the camera frame
      3. Transforms the pose into the robot base frame
      4. Executes the full grasp sequence via MoveIt2
    """

    def __init__(self):
        super().__init__("cube_grasp_node")

        # --- Frame names ---
        self.robot_base_frame  = "g_base"                    # robot_base_frame from easyhandeye2 calibration
        self.camera_frame      = "camera_color_optical_frame" # tracking_base_frame from easyhandeye2 calibration
        self.tf_timeout        = Duration(seconds=1.0)

        # --- TF2: used to look up the easyhandeye2 calibration transform ---
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- MoveIt2 setup ---
        # Verify end_effector_name and group_name against your URDF / SRDF:
        #   ros2 run tf2_tools view_frames          (for link names)
        #   cat <moveit_config>/config/*.srdf        (for group name)
        joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=joint_names,
            base_link_name=self.robot_base_frame,
            end_effector_name="joint6_flange",  # adjust to your URDF EE link
            group_name="arm"                    # adjust to your MoveIt SRDF group
        )
        self.moveit2.max_velocity    = 0.2
        self.moveit2.max_acceleration = 0.2

        # --- myCobot280Pi gripper via pymycobot SDK ---
        # Adjust port if needed: /dev/ttyAMA0 or /dev/ttyUSB0
        self.mc = MyCobot("/dev/ttyAMA0", 1000000)

        # --- Grasp state guard ---
        self.is_grasping  = False
        self._grasp_lock  = threading.Lock()

        # --- Grasp parameters ---
        self.pregrasp_height = 0.10   # 10 cm above cube
        self.approach_height = 0.02   # 2 cm above cube
        self.lift_height     = 0.15   # 15 cm above cube

        # --- Subscribe to raw cube pose in camera frame ---
        self.create_subscription(
            msg_type=PoseStamped,
            topic="/cube_pose_camera",   # raw detection output, in camera_color_optical_frame
            callback=self.cube_callback,
            qos_profile=1
        )

        self.get_logger().info(
            f"CubeGraspNode started.\n"
            f"  Listening for cube detections on : /cube_pose_camera\n"
            f"  Transforming from               : {self.camera_frame}\n"
            f"  Into robot frame                : {self.robot_base_frame}"
        )


    def cube_callback(self, msg: PoseStamped):
        """
        Called on every incoming cube detection. Transforms the pose and
        spawns a grasp thread if one is not already running.
        """
        # --- Step 1: Transform pose from camera frame into robot base frame ---
        # Make sure the incoming message has the correct frame_id set.
        # If your detection node doesn't set it, override it here:
        if not msg.header.frame_id:
            msg.header.frame_id = self.camera_frame

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame=self.robot_base_frame,
                source_frame=msg.header.frame_id,
                time=msg.header.stamp,
                timeout=self.tf_timeout
            )
        except LookupException as e:
            self.get_logger().warn(
                f"TF lookup failed — is easyhandeye2 publish.launch running? {e}"
            )
            return
        except ExtrapolationException:
            # Timestamp in the message may be slightly ahead/behind — use latest transform
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame=self.robot_base_frame,
                    source_frame=msg.header.frame_id,
                    time=rclpy.time.Time(),   # latest available
                    timeout=self.tf_timeout
                )
            except Exception as e:
                self.get_logger().warn(f"TF fallback also failed: {e}")
                return
        except Exception as e:
            self.get_logger().error(f"Unexpected TF error: {e}")
            return

        pose_in_base = do_transform_pose(msg, transform)
        pose_in_base.header.stamp    = self.get_clock().now().to_msg()
        pose_in_base.header.frame_id = self.robot_base_frame

        self.get_logger().info(
            f"Cube in robot frame → "
            f"x={pose_in_base.pose.position.x:.3f}  "
            f"y={pose_in_base.pose.position.y:.3f}  "
            f"z={pose_in_base.pose.position.z:.3f}"
        )

        # --- Step 2: Spawn grasp thread if not already grasping ---
        with self._grasp_lock:
            if self.is_grasping:
                self.get_logger().info("Grasp already in progress — ignoring new detection.")
                return
            self.is_grasping = True

        thread = threading.Thread(
            target=self._grasp_sequence,
            args=(pose_in_base,),
            daemon=True
        )
        thread.start()


    def _grasp_sequence(self, msg: PoseStamped):
        """
        Full grasp sequence in its own thread so the ROS2 executor
        is never blocked by wait_until_executed() calls.
        """
        try:
            cx = msg.pose.position.x
            cy = msg.pose.position.y
            cz = msg.pose.position.z
            self.get_logger().info(
                f"Starting grasp sequence for cube at ({cx:.3f}, {cy:.3f}, {cz:.3f})"
            )

            # Orientation: tool pointing straight down.
            # If planning fails, try (roll=0, pitch=pi, yaw=0) instead —
            # the correct values depend on your URDF EE frame convention.
            roll, pitch, yaw = math.pi, 0.0, 0.0
            quat = rpy_to_quat(roll, pitch, yaw)

            pos_pre      = [cx, cy, cz + self.pregrasp_height]
            pos_approach = [cx, cy, cz + self.approach_height]
            pos_lift     = [cx, cy, cz + self.lift_height]

            # 1) Open gripper before moving
            self.get_logger().info("Opening gripper.")
            self._set_gripper(open=True)

            # 2) Pre-grasp
            self.get_logger().info("Moving to pre-grasp.")
            self.moveit2.move_to_pose(position=pos_pre, quat_xyzw=quat)
            self.moveit2.wait_until_executed()

            # 3) Approach
            self.get_logger().info("Approaching cube.")
            self.moveit2.move_to_pose(position=pos_approach, quat_xyzw=quat)
            self.moveit2.wait_until_executed()

            # 4) Close gripper
            self.get_logger().info("Closing gripper.")
            self._set_gripper(open=False)

            # 5) Lift
            self.get_logger().info("Lifting cube.")
            self.moveit2.move_to_pose(position=pos_lift, quat_xyzw=quat)
            self.moveit2.wait_until_executed()

            self.get_logger().info("Grasp sequence complete.")

        except Exception as e:
            self.get_logger().error(f"Grasp sequence failed: {e}")

        finally:
            with self._grasp_lock:
                self.is_grasping = False


    def _set_gripper(self, open: bool):
        """
        Control the myCobot280Pi gripper via the pymycobot SDK.

        set_gripper_value(value, speed)
          value: 0   = fully closed
                 100 = fully open
          speed: 0–100

        Adjust the closed value to suit your cube size.
        """
        if open:
            self.mc.set_gripper_value(100, 80)
        else:
            self.mc.set_gripper_value(30, 80)   # tune for your cube


def rpy_to_quat(roll, pitch, yaw):
    """Convert roll, pitch, yaw (radians) to quaternion [qx, qy, qz, qw]."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return [qx, qy, qz, qw]


def main(args=None):
    try:
        rclpy.init(args=args)

        cube_grasp_node = CubeGraspNode()

        rclpy.spin(cube_grasp_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()