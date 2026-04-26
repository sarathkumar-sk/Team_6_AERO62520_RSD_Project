#!/usr/bin/env python3
"""
realsense_cube_publisher.py
----------------------------
Detects a small coloured cube (green / blue / yellow) using an Intel RealSense
D435 RGBD camera and publishes its 3-D pose + detected colour.

Subscriptions:
  /camera/camera/color/image_raw        (sensor_msgs/Image)
  /camera/camera/depth/image_rect_raw   (sensor_msgs/Image)
  /camera/camera/depth/camera_info      (sensor_msgs/CameraInfo)

Publications:
  /cube_pose_camera  (geometry_msgs/PoseStamped)
  /cube_color        (std_msgs/String)  — "green" | "blue" | "yellow"
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
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


# HSV colour definitions — (lower, upper, name)
# CHANGE: tune ranges under your actual lighting conditions
COLOUR_RANGES = [
    (np.array([35,  60,  40]), np.array([85,  255, 255]), "green"),
    (np.array([90,  60,  40]), np.array([135, 255, 255]), "blue"),
    (np.array([20,  60,  40]), np.array([35,  255, 255]), "yellow"),
]


class RealSenseCubePublisher(Node):
    def __init__(self):
        super().__init__("realsense_cube_publisher")

        self.bridge      = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.depth_info  = None
        self._warned_shape_mismatch = False

        # ------------------------------------------------------------------
        # Subscriptions
        # CHANGE: verify topic names with `ros2 topic list`
        # ------------------------------------------------------------------
        self.create_subscription(Image,      
                                 "/camera/camera/color/image_raw",
                                 self._color_cb,      
                                 qos_profile_sensor_data)
        self.create_subscription(Image,      
                                 "/camera/camera/depth/image_rect_raw",
                                 self._depth_cb,      
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, 
                                 "/camera/camera/depth/camera_info",
                                 self._depth_info_cb, 
                                 qos_profile_sensor_data)

        # Publications
        self.pose_pub  = self.create_publisher(PoseStamped, "/cube_pose_camera", 10)
        self.color_pub = self.create_publisher(String,      "/cube_color",       10)

        self.create_timer(0.1, self._process)   # 10 Hz

        # ------------------------------------------------------------------
        # Size thresholds for a 3×3×3 cm cube (metres)
        # CHANGE: adjust for your cube dimensions
        # ------------------------------------------------------------------
        self.min_diam_m       = 0.015   # ~1.5 cm  below this is noise
        self.max_diam_m       = 0.080   # ~8 cm    above this is probably the box
        self.cube_half_size_m = 0.015   # half-depth of cube (face → centre offset)

        self.get_logger().info("RealSenseCubePublisher started")

    # ------------------------------------------------------------------
    # Sensor callbacks
    # ------------------------------------------------------------------
    def _color_cb(self, msg: Image):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def _depth_cb(self, msg: Image):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough").astype(np.float32)

    def _depth_info_cb(self, msg: CameraInfo):
        self.depth_info = msg

    # ------------------------------------------------------------------
    # Detection loop (10 Hz)
    # ------------------------------------------------------------------
    def _process(self):
        if self.color_image is None or self.depth_image is None or self.depth_info is None:
            return

        color   = self.color_image.copy()
        depth_m = self.depth_image.copy() / 1000.0

        if depth_m.shape != color.shape[:2]:
            if not self._warned_shape_mismatch:
                self.get_logger().warn("Depth/colour shape mismatch — resizing depth.")
                self._warned_shape_mismatch = True
            h, w    = color.shape[:2]
            depth_m = cv2.resize(depth_m, (w, h))

        hsv          = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        fx,   fy     = self.depth_info.k[0], self.depth_info.k[4]
        cx_i, cy_i   = self.depth_info.k[2], self.depth_info.k[5]
        kernel       = np.ones((5, 5), np.uint8)

        best_score  = 0.0
        best_result = None   # (cx_px, cy_px, Z_face, colour_name)

        for lower, upper, colour_name in COLOUR_RANGES:
            mask = cv2.inRange(hsv, lower, upper)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for c in contours:
                area_px = cv2.contourArea(c)
                if area_px < 100:
                    continue

                x_bb, y_bb, w_bb, h_bb = cv2.boundingRect(c)
                if w_bb < 4 or h_bb < 4:
                    continue

                roi   = depth_m[y_bb:y_bb + h_bb, x_bb:x_bb + w_bb]
                valid = roi[roi > 0.0]
                if valid.size < 30:
                    continue

                Z = float(np.median(valid))
                if Z <= 0.0:
                    continue

                diam = 0.5 * ((w_bb * Z) / fx + (h_bb * Z) / fy)
                if not (self.min_diam_m <= diam <= self.max_diam_m):
                    continue

                score = area_px / (Z + 1e-3)
                if score > best_score:
                    M = cv2.moments(c)
                    if M["m00"] == 0:
                        continue
                    best_score  = score
                    best_result = (
                        int(M["m10"] / M["m00"]),
                        int(M["m01"] / M["m00"]),
                        Z,
                        colour_name,
                    )

        if best_result is None:
            return

        cx_px, cy_px, Z_face, colour_name = best_result

        Z_center = Z_face + self.cube_half_size_m
        X_center = (cx_px - cx_i) * Z_center / fx
        Y_center = (cy_px - cy_i) * Z_center / fy

        self.get_logger().info(
            f"Cube ({colour_name})  X={X_center:.3f}  Y={Y_center:.3f}  Z={Z_center:.3f} m"
        )

        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = "camera_depth_optical_frame"
        pose.pose.position.x = float(X_center)
        pose.pose.position.y = float(Y_center)
        pose.pose.position.z = float(Z_center)
        # CHANGE: verify this matches your EEF down-pointing convention
        qx, qy, qz, qw = rpy_to_quat(math.pi, 0.0, 0.0)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.pose_pub.publish(pose)

        col_msg      = String()
        col_msg.data = colour_name
        self.color_pub.publish(col_msg)


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
