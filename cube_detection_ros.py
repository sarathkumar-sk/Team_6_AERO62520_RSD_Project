#!/usr/bin/env python3
import math

from rclpy.qos import qos_profile_sensor_data

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

import numpy as np
import cv2


def rpy_to_quat(roll, pitch, yaw):
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

        self.warned_shape_mismatch = False

        # ------------------------------------------------------------------
        # YOUR NUC TOPICS (from ros2 topic list):
        #   /camera/camera/color/image_raw
        #   /camera/camera/depth/image_rect_raw
        #   /camera/camera/depth/camera_info
        # ------------------------------------------------------------------
        self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.color_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/camera/camera/depth/image_rect_raw",
            self.depth_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            "/camera/camera/depth/camera_info",
            self.depth_info_cb,
            qos_profile_sensor_data,
        )

        self.pub = self.create_publisher(PoseStamped, "/cube_pose_camera", 10)

        # Run at 10 Hz
        self.timer = self.create_timer(0.1, self.process)

        # Tunable size thresholds (in metres) for a 3×3×3 cm cube
        # "diam" is approx average of width/height in metres.
        self.min_diam_m = 0.015   # smaller than ~1.5 cm is noise
        self.max_diam_m = 0.08    # bigger than 8 cm is probably the base box

        # Approx half depth of cube (3 cm / 2)
        self.cube_half_size_m = 0.015  # 1.5 cm

        self.get_logger().info(
            f"RealSenseCubePublisher running: min_diam={self.min_diam_m} m, "
            f"max_diam={self.max_diam_m} m, half_depth={self.cube_half_size_m} m"
        )

    # -------------------------
    # Callbacks
    # -------------------------
    def color_cb(self, msg: Image):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_cb(self, msg: Image):
        # D435 depth is typically 16UC1 in mm
        depth_mm = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.depth_image = depth_mm.astype(np.float32)

    def depth_info_cb(self, msg: CameraInfo):
        self.depth_info = msg

    # -------------------------
    # Main processing
    # -------------------------
    def process(self):
        if self.color_image is None or self.depth_image is None or self.depth_info is None:
            missing = []
            if self.color_image is None:
                missing.append("color_image")
            if self.depth_image is None:
                missing.append("depth_image")
            if self.depth_info is None:
                missing.append("depth_info")
            self.get_logger().info(f"Waiting for: {', '.join(missing)}")
            return

        color = self.color_image.copy()
        depth_mm = self.depth_image.copy()  # mm
        depth_m = depth_mm / 1000.0         # metres

        # Shapes should usually match; if not, we resize depth once and warn
        if depth_m.shape != color.shape[:2]:
            if not self.warned_shape_mismatch:
                self.get_logger().warn(
                    f"Depth and color shapes differ: depth={depth_m.shape}, color={color.shape[:2]}. "
                    "Resizing depth, but check your RealSense config / alignment."
                )
                self.warned_shape_mismatch = True
            h, w = color.shape[:2]
            depth_m = cv2.resize(depth_m, (w, h))
            depth_mm = cv2.resize(depth_mm, (w, h))

        # ---------------------------------------------------------
        # 1) Colour segmentation: green, blue, yellow in HSV
        # ---------------------------------------------------------
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)

        # Green
        lower_g = np.array([35, 60, 40])
        upper_g = np.array([85, 255, 255])
        # Blue
        lower_b = np.array([90, 60, 40])
        upper_b = np.array([135, 255, 255])
        # Yellow
        lower_y = np.array([20, 60, 40])
        upper_y = np.array([35, 255, 255])

        mask_g = cv2.inRange(hsv, lower_g, upper_g)
        mask_b = cv2.inRange(hsv, lower_b, upper_b)
        mask_y = cv2.inRange(hsv, lower_y, upper_y)

        mask = cv2.bitwise_or(mask_g, mask_b)
        mask = cv2.bitwise_or(mask, mask_y)

        # Clean up mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # ---------------------------------------------------------
        # 2) Find candidate cube contours in the colour mask
        # ---------------------------------------------------------
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.get_logger().info(f"Found {len(contours)} coloured blobs")

        if not contours:
            return

        # Intrinsics of the DEPTH camera (because we used depth/camera_info)
        fx = self.depth_info.k[0]
        fy = self.depth_info.k[4]
        cx_i = self.depth_info.k[2]
        cy_i = self.depth_info.k[5]

        best_contour = None
        best_score = 0.0
        best_Z = None
        best_debug = None

        for idx, c in enumerate(contours):
            area_px = cv2.contourArea(c)
            if area_px < 100:  # reject tiny blobs
                self.get_logger().debug(f"Contour {idx}: too small (area={area_px:.1f})")
                continue

            x, y, w, h = cv2.boundingRect(c)
            if w < 4 or h < 4:
                self.get_logger().debug(f"Contour {idx}: w/h too small (w={w}, h={h})")
                continue

            roi = depth_m[y:y+h, x:x+w]
            valid = roi[roi > 0.0]
            if valid.size < 30:
                self.get_logger().debug(f"Contour {idx}: insufficient valid depth pixels ({valid.size})")
                continue

            Z = float(np.median(valid))  # robust depth estimate
            if Z <= 0.0:
                self.get_logger().debug(f"Contour {idx}: non-positive median depth {Z}")
                continue

            # Approximate real-world width/height at depth Z
            width_m = (w * Z) / fx
            height_m = (h * Z) / fy
            diam = 0.5 * (width_m + height_m)

            if diam < self.min_diam_m or diam > self.max_diam_m:
                self.get_logger().debug(
                    f"Contour {idx}: diam out of range (diam={diam:.3f} m, "
                    f"w_m={width_m:.3f}, h_m={height_m:.3f})"
                )
                continue

            # Optional loose aspect ratio check if needed
            aspect_ratio = float(w) / float(h + 1e-6)
            # self.get_logger().debug(f"Contour {idx}: aspect_ratio={aspect_ratio:.2f}")
            # If base box has similar colour and is slipping through, you can enforce:
            # if not (0.6 <= aspect_ratio <= 1.6):
            #     continue

            # Scoring: favour larger, closer blobs
            score = area_px / (Z + 1e-3)

            self.get_logger().debug(
                f"Contour {idx}: area={area_px:.1f}, Z={Z:.3f}, "
                f"w_m={width_m:.3f}, h_m={height_m:.3f}, diam={diam:.3f}, score={score:.1f}"
            )

            if score > best_score:
                best_score = score
                best_contour = c
                best_Z = Z
                best_debug = (area_px, width_m, height_m, diam)

        if best_contour is None:
            self.get_logger().info("No contour passed filters this frame")
            return

        area_px, width_m, height_m, diam = best_debug
        self.get_logger().info(
            f"Selected contour: area={area_px:.1f}, Z_face={best_Z:.3f} m, "
            f"w_m={width_m:.3f}, h_m={height_m:.3f}, diam={diam:.3f}"
        )

        # ---------------------------------------------------------
        # 3) Compute 3D position of cube centre (depth optical frame)
        # ---------------------------------------------------------
        M = cv2.moments(best_contour)
        if M["m00"] == 0:
            self.get_logger().warn("Best contour has zero moments (m00==0); skipping")
            return

        cx_px = int(M["m10"] / M["m00"])
        cy_px = int(M["m01"] / M["m00"])

        Z_face = best_Z
        if Z_face <= 0.0:
            self.get_logger().warn("Best contour has non-positive Z_face; skipping")
            return

        # Deproject the face centre
        X_face = (cx_px - cx_i) * Z_face / fx
        Y_face = (cy_px - cy_i) * Z_face / fy

        # Approximate cube centre by adding half cube size along camera Z
        Z_center = Z_face + self.cube_half_size_m
        X_center = (cx_px - cx_i) * Z_center / fx
        Y_center = (cy_px - cy_i) * Z_center / fy

        self.get_logger().info(
            f"Cube centre (camera_depth_optical_frame): "
            f"X={X_center:.3f}, Y={Y_center:.3f}, Z={Z_center:.3f} m"
        )

        # ---------------------------------------------------------
        # 4) Publish pose in camera_depth_optical_frame
        # ---------------------------------------------------------
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "camera_depth_optical_frame"

        pose.pose.position.x = float(X_center)
        pose.pose.position.y = float(Y_center)
        pose.pose.position.z = float(Z_center)

        # Orientation: tool pointing roughly towards camera (roll = pi)
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