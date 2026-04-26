#!/usr/bin/env python3
"""
realsense_box_publisher.py
---------------------------
Detects a 20×20 cm coloured storage box (green / blue / yellow) using an Intel
RealSense D435 RGBD camera and publishes its 3-D centre pose and colour.

The box is assumed to be sitting on the floor with its open top facing up.
Detection looks for a large, roughly square coloured region.

Subscriptions:
  /camera/camera/color/image_raw        (sensor_msgs/Image)
  /camera/camera/depth/image_rect_raw   (sensor_msgs/Image)
  /camera/camera/depth/camera_info      (sensor_msgs/CameraInfo)

Publications:
  /box_pose_camera   (geometry_msgs/PoseStamped)
    — centre of the detected box face nearest to the camera, in
      camera_depth_optical_frame
  /box_color         (std_msgs/String)   — "green" | "blue" | "yellow"
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from cv_bridge import CvBridge

import numpy as np
import cv2


# HSV colour definitions — (lower, upper, name)
# CHANGE: tune ranges to match your box colours under your lighting conditions.
#         These are the same ranges used for cube detection — if box and cube
#         share the same hue band, no changes are needed here.
COLOUR_RANGES = [
    (np.array([35,  60,  40]), np.array([85,  255, 255]), "green"),
    (np.array([90,  60,  40]), np.array([135, 255, 255]), "blue"),
    (np.array([20,  60,  40]), np.array([35,  255, 255]), "yellow"),
]


class RealSenseBoxPublisher(Node):
    def __init__(self):
        super().__init__("realsense_box_publisher")

        self.bridge      = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.depth_info  = None
        self._warned_shape_mismatch = False

        # ------------------------------------------------------------------
        # Subscriptions
        # CHANGE: verify topic names with `ros2 topic list`
        # ------------------------------------------------------------------
        self.create_subscription(Image,      "/camera/camera/color/image_raw",
                                 self._color_cb,      qos_profile_sensor_data)
        self.create_subscription(Image,      "/camera/camera/depth/image_rect_raw",
                                 self._depth_cb,      qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/camera/depth/camera_info",
                                 self._depth_info_cb, qos_profile_sensor_data)

        self.pose_pub  = self.create_publisher(PoseStamped, "/box_pose_camera", 10)
        self.color_pub = self.create_publisher(String,      "/box_color",       10)

        self.create_timer(0.1, self._process)   # 10 Hz

        # ------------------------------------------------------------------
        # Size thresholds for a 20×20 cm box (metres).
        #
        # We use the average of projected width and height ("diam") just like
        # the cube publisher, but with larger bounds.
        #
        # At 0.5 m:  a 20 cm object ≈ 240 px wide  (D435, fx ≈ 600)
        # At 1.5 m:  a 20 cm object ≈  80 px wide
        #
        # CHANGE: widen / tighten if your box is at unusual distances.
        # ------------------------------------------------------------------
        self.min_diam_m = 0.12   # 12 cm — smaller blobs are probably the cube
        self.max_diam_m = 0.40   # 40 cm — larger blobs are structural features

        # ------------------------------------------------------------------
        # Squareness gate: box is square so w/h should be close to 1.
        # Applies to the bounding rectangle of the detected contour.
        # CHANGE: relax if the box is often seen at a steep angle.
        # ------------------------------------------------------------------
        self.min_aspect = 0.45   # allow up to ~2:1 ratio (partial visibility)
        self.max_aspect = 2.20

        # ------------------------------------------------------------------
        # Solidity gate: ratio of contour area to bounding-rect area.
        # A clean square gives ~1.0; ragged or L-shaped blobs give much less.
        # CHANGE: lower if the box face is often partially occluded.
        # ------------------------------------------------------------------
        self.min_solidity = 0.50

        # Half-depth of the box wall / rim (used to offset detected face →
        # the approximate opening centre).
        # CHANGE: set to half your actual box wall thickness if you want
        #         the pose to sit at the very centre of the opening.
        #         For a drop task, the face centre is usually precise enough.
        self.box_half_wall_m = 0.005  # 5 mm wall thickness

        self.get_logger().info(
            f"RealSenseBoxPublisher started | "
            f"diam=[{self.min_diam_m}, {self.max_diam_m}] m  "
            f"aspect=[{self.min_aspect}, {self.max_aspect}]  "
            f"min_solidity={self.min_solidity}"
        )

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

        # Larger kernel for the bigger blobs we expect from a 20 cm box
        kernel = np.ones((9, 9), np.uint8)

        best_score  = 0.0
        best_result = None   # (cx_px, cy_px, Z_face, colour_name)

        for lower, upper, colour_name in COLOUR_RANGES:
            mask = cv2.inRange(hsv, lower, upper)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for c in contours:
                area_px = cv2.contourArea(c)
                if area_px < 400:   # minimum ~20×20 px blob
                    continue

                x_bb, y_bb, w_bb, h_bb = cv2.boundingRect(c)
                if w_bb < 10 or h_bb < 10:
                    continue

                # ---- Aspect ratio filter (squareness check) ----
                aspect = float(w_bb) / float(h_bb + 1e-6)
                if not (self.min_aspect <= aspect <= self.max_aspect):
                    self.get_logger().debug(
                        f"{colour_name} contour rejected: aspect={aspect:.2f}"
                    )
                    continue

                # ---- Solidity filter (reject irregular blobs) ----
                solidity = area_px / float(w_bb * h_bb + 1e-6)
                if solidity < self.min_solidity:
                    self.get_logger().debug(
                        f"{colour_name} contour rejected: solidity={solidity:.2f}"
                    )
                    continue

                # ---- Depth estimate ----
                roi   = depth_m[y_bb:y_bb + h_bb, x_bb:x_bb + w_bb]
                valid = roi[roi > 0.0]
                if valid.size < 50:
                    continue

                Z = float(np.median(valid))
                if Z <= 0.0:
                    continue

                # ---- Real-world size check ----
                width_m  = (w_bb * Z) / fx
                height_m = (h_bb * Z) / fy
                diam     = 0.5 * (width_m + height_m)

                if not (self.min_diam_m <= diam <= self.max_diam_m):
                    self.get_logger().debug(
                        f"{colour_name} contour rejected: diam={diam:.3f} m "
                        f"(w={width_m:.3f}, h={height_m:.3f})"
                    )
                    continue

                self.get_logger().debug(
                    f"{colour_name} box candidate: area={area_px:.0f} px²  "
                    f"Z={Z:.3f} m  w={width_m:.3f} m  h={height_m:.3f} m  "
                    f"aspect={aspect:.2f}  solidity={solidity:.2f}"
                )

                # Prefer larger, closer candidates
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

        # Project to 3-D — we keep Z at the detected face depth.
        # A small wall-thickness offset is added so the pose sits at the
        # inside surface rather than the outer face.
        Z_pub = Z_face + self.box_half_wall_m
        X_pub = (cx_px - cx_i) * Z_pub / fx
        Y_pub = (cy_px - cy_i) * Z_pub / fy

        self.get_logger().info(
            f"Box ({colour_name})  X={X_pub:.3f}  Y={Y_pub:.3f}  Z={Z_pub:.3f} m"
        )

        # Orientation: identity — the grasp node will override this with its
        # own EEF orientation anyway.
        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = "camera_depth_optical_frame"
        pose.pose.position.x = float(X_pub)
        pose.pose.position.y = float(Y_pub)
        pose.pose.position.z = float(Z_pub)
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

        col_msg      = String()
        col_msg.data = colour_name
        self.color_pub.publish(col_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseBoxPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
