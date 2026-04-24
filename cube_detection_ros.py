#!/usr/bin/env python3
"""
realsense_cube_publisher.py
----------------------------
Detects a small coloured cube (green / blue / yellow) using an Intel RealSense
D435 RGBD camera and publishes its 3-D pose in the camera_depth_optical_frame.

Subscriptions:
  /camera/camera/color/image_raw        (sensor_msgs/Image)
  /camera/camera/depth/image_rect_raw   (sensor_msgs/Image)
  /camera/camera/depth/camera_info      (sensor_msgs/CameraInfo)

Publications:
  /cube_pose_camera  (geometry_msgs/PoseStamped)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

import numpy as np
import cv2


def rpy_to_quat(roll: float, pitch: float, yaw: float):
    """Convert roll-pitch-yaw (radians) to quaternion [x, y, z, w]."""
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


class RealSenseCubePublisher(Node):
    def __init__(self):
        super().__init__("realsense_cube_publisher")

        self.bridge = CvBridge()

        self.color_image = None
        self.depth_image = None
        self.depth_info = None

        self._warned_shape_mismatch = False

        # ------------------------------------------------------------------
        # Subscriptions — topic names match a standard RealSense ROS2 launch
        # CHANGE if your topics differ (check: ros2 topic list)
        # ------------------------------------------------------------------
        self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self._color_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/camera/camera/depth/image_rect_raw",
            self._depth_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            "/camera/camera/depth/camera_info",
            self._depth_info_cb,
            qos_profile_sensor_data,
        )

        self.pub = self.create_publisher(PoseStamped, "/cube_pose_camera", 10)

        # Run detection at 10 Hz
        self.create_timer(0.1, self._process)

        # ------------------------------------------------------------------
        # Size thresholds for a 3×3×3 cm cube (in metres)
        # CHANGE: adjust if your cube is a different size
        # ------------------------------------------------------------------
        self.min_diam_m = 0.015   # ~1.5 cm — anything smaller is noise
        self.max_diam_m = 0.080   # ~8 cm  — anything bigger is probably the base box

        # Half the cube depth; used to offset from the detected face to the centre
        self.cube_half_size_m = 0.015  # 3 cm cube → 1.5 cm half-depth

        self.get_logger().info(
            f"RealSenseCubePublisher started | "
            f"min_diam={self.min_diam_m} m  max_diam={self.max_diam_m} m  "
            f"half_depth={self.cube_half_size_m} m"
        )

    # ------------------------------------------------------------------
    # Sensor callbacks — store latest frames
    # ------------------------------------------------------------------
    def _color_cb(self, msg: Image):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def _depth_cb(self, msg: Image):
        # D435 depth is 16UC1 in mm
        depth_mm = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.depth_image = depth_mm.astype(np.float32)

    def _depth_info_cb(self, msg: CameraInfo):
        self.depth_info = msg

    # ------------------------------------------------------------------
    # Main detection loop (10 Hz timer)
    # ------------------------------------------------------------------
    def _process(self):
        # Guard: wait until all streams have delivered at least one frame
        if self.color_image is None or self.depth_image is None or self.depth_info is None:
            missing = [
                name
                for name, val in [
                    ("color_image", self.color_image),
                    ("depth_image", self.depth_image),
                    ("depth_info", self.depth_info),
                ]
                if val is None
            ]
            self.get_logger().info(f"Waiting for: {', '.join(missing)}")
            return

        color = self.color_image.copy()
        depth_m = self.depth_image.copy() / 1000.0  # mm → m

        # Align depth to colour if shapes differ (should not happen with RealSense
        # align_depth:=true launch arg, but handled gracefully just in case)
        if depth_m.shape != color.shape[:2]:
            if not self._warned_shape_mismatch:
                self.get_logger().warn(
                    f"Depth/colour shape mismatch: depth={depth_m.shape}, "
                    f"color={color.shape[:2]}. Resizing depth. "
                    "Consider launching with align_depth:=true."
                )
                self._warned_shape_mismatch = True
            h, w = color.shape[:2]
            depth_m = cv2.resize(depth_m, (w, h))

        # ------------------------------------------------------------------
        # 1. Colour segmentation in HSV — green, blue, yellow
        # CHANGE: tweak HSV ranges under your lighting conditions
        # ------------------------------------------------------------------
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)

        mask_g = cv2.inRange(hsv, np.array([35, 60,  40]), np.array([85,  255, 255]))
        mask_b = cv2.inRange(hsv, np.array([90, 60,  40]), np.array([135, 255, 255]))
        mask_y = cv2.inRange(hsv, np.array([20, 60,  40]), np.array([35,  255, 255]))

        mask = cv2.bitwise_or(mask_g, cv2.bitwise_or(mask_b, mask_y))

        # Morphological cleanup
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # ------------------------------------------------------------------
        # 2. Find contours and pick the best candidate
        # ------------------------------------------------------------------
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.get_logger().debug(f"Found {len(contours)} coloured blobs")

        if not contours:
            return

        # Camera intrinsics (from depth camera_info)
        fx   = self.depth_info.k[0]
        fy   = self.depth_info.k[4]
        cx_i = self.depth_info.k[2]
        cy_i = self.depth_info.k[5]

        best_contour = None
        best_score   = 0.0
        best_Z       = None
        best_debug   = None

        for idx, c in enumerate(contours):
            area_px = cv2.contourArea(c)
            if area_px < 100:
                continue

            x_bb, y_bb, w_bb, h_bb = cv2.boundingRect(c)
            if w_bb < 4 or h_bb < 4:
                continue

            # Robust depth estimate from the bounding-box ROI
            roi   = depth_m[y_bb : y_bb + h_bb, x_bb : x_bb + w_bb]
            valid = roi[roi > 0.0]
            if valid.size < 30:
                continue

            Z = float(np.median(valid))
            if Z <= 0.0:
                continue

            # Back-project pixel dimensions to metres at depth Z
            width_m  = (w_bb * Z) / fx
            height_m = (h_bb * Z) / fy
            diam     = 0.5 * (width_m + height_m)

            if not (self.min_diam_m <= diam <= self.max_diam_m):
                self.get_logger().debug(
                    f"Contour {idx}: diam={diam:.3f} m out of "
                    f"[{self.min_diam_m}, {self.max_diam_m}] — skipped"
                )
                continue

            # Score: favour larger blobs that are closer
            score = area_px / (Z + 1e-3)

            if score > best_score:
                best_score   = score
                best_contour = c
                best_Z       = Z
                best_debug   = (area_px, width_m, height_m, diam)

        if best_contour is None:
            self.get_logger().info("No contour passed filters this frame")
            return

        area_px, width_m, height_m, diam = best_debug
        self.get_logger().info(
            f"Best contour: area={area_px:.0f} px²  Z_face={best_Z:.3f} m  "
            f"w={width_m:.3f} m  h={height_m:.3f} m  diam={diam:.3f} m"
        )

        # ------------------------------------------------------------------
        # 3. Compute 3-D position of the cube centre
        # ------------------------------------------------------------------
        M = cv2.moments(best_contour)
        if M["m00"] == 0:
            self.get_logger().warn("Zero-area contour moments — skipping")
            return

        cx_px = int(M["m10"] / M["m00"])
        cy_px = int(M["m01"] / M["m00"])

        Z_face = best_Z

        # Add half cube depth to get from detected face to cube centre
        Z_center = Z_face + self.cube_half_size_m
        X_center = (cx_px - cx_i) * Z_center / fx
        Y_center = (cy_px - cy_i) * Z_center / fy

        self.get_logger().info(
            f"Cube centre (camera_depth_optical_frame): "
            f"X={X_center:.3f}  Y={Y_center:.3f}  Z={Z_center:.3f} m"
        )

        # ------------------------------------------------------------------
        # 4. Publish PoseStamped
        # ------------------------------------------------------------------
        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = "camera_depth_optical_frame"

        pose.pose.position.x = float(X_center)
        pose.pose.position.y = float(Y_center)
        pose.pose.position.z = float(Z_center)

        # Orientation: tool pointing roughly towards camera (roll = π)
        # CHANGE: adjust if your arm EEF convention is different
        qx, qy, qz, qw = rpy_to_quat(math.pi, 0.0, 0.0)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseCubePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
