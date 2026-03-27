#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

import numpy as np
import cv2
import math

# ---------------------------------------------------------
# Helper functions (your originals — unchanged)
# ---------------------------------------------------------
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

def get_dominant_color(image, mask):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    pixels = hsv_image[mask == 255].reshape(-1, 3)
    if len(pixels) < 50:
        return (0, 0, 0), "unknown"
    from sklearn.cluster import KMeans
    kmeans = KMeans(n_clusters=1, n_init=3)
    kmeans.fit(pixels)
    dominant_hsv = kmeans.cluster_centers_[0].astype(int)
    dominant_bgr = cv2.cvtColor(np.uint8([[dominant_hsv]]), cv2.COLOR_HSV2BGR)[0][0]
    return tuple(map(int, dominant_bgr)), classify_color(dominant_hsv)

def classify_color(hsv_color):
    h, s, v = hsv_color
    if s < 50 or v < 50:
        return "other"
    if (0 <= h <= 10 or 170 <= h <= 180) and s > 100:
        if 0 <= h <= 10 and s > 150 and v > 100:
            return "c52c2e_red"
        return "red"
    elif 11 <= h <= 20 and s > 100:
        return "orange"
    elif 21 <= h <= 35 and s > 50:
        return "yellow"
    elif 36 <= h <= 85 and s > 50:
        return "green"
    elif 86 <= h <= 125 and s > 50:
        return "blue"
    elif 126 <= h <= 140 and s > 50:
        return "purple"
    return "other"

def preprocess_depth(depth_frame):
    depth_image = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    depth_image = cv2.bilateralFilter(depth_image, 9, 75, 75)
    kernel = np.ones((3,3), np.uint8)
    depth_image = cv2.morphologyEx(depth_image, cv2.MORPH_OPEN, kernel)
    depth_image = cv2.morphologyEx(depth_image, cv2.MORPH_CLOSE, kernel)
    return depth_image

def detect_edges(image, depth_data):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    depth_norm = cv2.normalize(depth_data, None, 0, 255, cv2.NORM_MINMAX)
    edges_color = cv2.Canny(gray, 50, 150)
    edges_depth = cv2.Canny(depth_norm.astype(np.uint8), 25, 75)
    combined = cv2.bitwise_or(edges_color, edges_depth)
    kernel = np.ones((3,3), np.uint8)
    combined = cv2.dilate(combined, kernel, 1)
    combined = cv2.erode(combined, kernel, 1)
    return combined

def classify_3d_shape(contour, depth_roi):
    epsilon = 0.02 * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True)
    vertices = len(approx)
    depth_vals = depth_roi[depth_roi > 0].flatten()
    if len(depth_vals) < 50:
        return "unknown", 0, 0, 0
    depth_mean = np.mean(depth_vals)
    depth_std = np.std(depth_vals)
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    circularity = 4 * math.pi * (area / (perimeter * perimeter + 1e-5))
    x, y, w, h = cv2.boundingRect(contour)
    aspect_ratio = float(w) / (h + 1e-5)

    if circularity > 0.85:
        return ("sphere" if depth_std > 1500 else "cylinder"), depth_mean, depth_std, vertices
    elif vertices == 3:
        return ("pyramid" if depth_std > 1000 else "triangle"), depth_mean, depth_std, vertices
    elif vertices == 4:
        if depth_std > 2000:
            return "cube", depth_mean, depth_std, vertices
        return ("square" if 0.9 < aspect_ratio < 1.1 else "rectangle"), depth_mean, depth_std, vertices
    elif 5 <= vertices <= 8:
        return ("prism" if depth_std > 1800 else f"{vertices}-polygon"), depth_mean, depth_std, vertices
    return "complex", depth_mean, depth_std, vertices

# ---------------------------------------------------------
# ROS2 Node (topic‑based RealSense)
# ---------------------------------------------------------
class RealSenseCubePublisher(Node):
    def __init__(self):
        super().__init__("realsense_cube_publisher")

        self.bridge = CvBridge()

        self.color_image = None
        self.depth_image = None
        self.depth_info = None

        # Subscribers
        self.create_subscription(Image, "/camera/camera/color/image_raw", self.color_cb, 10)
        self.create_subscription(Image, "/camera/camera/depth/image_rect_raw", self.depth_cb, 10)
        self.create_subscription(CameraInfo, "/camera/camera/depth/camera_info", self.depth_info_cb, 10)

        # Publisher
        self.pub = self.create_publisher(PoseStamped, "/cube_pose_camera", 10)

        # Timer
        self.timer = self.create_timer(0.1, self.process)

        self.get_logger().info("RealSenseCubePublisher running (ROS2 topic mode)")

    # -------------------------
    # Callbacks
    # -------------------------
    def color_cb(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough").astype(np.float32)

    def depth_info_cb(self, msg):
        self.depth_info = msg

    # -------------------------
    # Main processing
    # -------------------------
    def process(self):
        if self.color_image is None or self.depth_image is None or self.depth_info is None:
            return

        color = self.color_image.copy()
        depth_mm = self.depth_image.copy()
        depth_m = depth_mm / 1000.0

        # ---------------------------------------------------------
        # FIX: Align depth to color resolution
        # ---------------------------------------------------------
        if depth_m.shape != color.shape[:2]:
            depth_m = cv2.resize(depth_m, (color.shape[1], color.shape[0]))
            depth_mm = cv2.resize(depth_mm, (color.shape[1], color.shape[0]))

        processed_depth = preprocess_depth(depth_mm)
        edges = detect_edges(color, processed_depth)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_area = 0

        for c in contours:
            if cv2.contourArea(c) < 300:
                continue

            x, y, w, h = cv2.boundingRect(c)
            roi = depth_mm[y:y+h, x:x+w]
            shape, dm, ds, v = classify_3d_shape(c, roi)
            if shape != "cube":
                continue

            area = cv2.contourArea(c)
            if area > best_area:
                best_area = area
                best = c

        if best is None:
            return

        M = cv2.moments(best)
        if M["m00"] == 0:
            return

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        Z = depth_m[cy, cx]
        if Z <= 0:
            return

        # -------------------------
        # Deprojection
        # -------------------------
        fx = self.depth_info.k[0]
        fy = self.depth_info.k[4]
        cx_i = self.depth_info.k[2]
        cy_i = self.depth_info.k[5]

        X = (cx - cx_i) * Z / fx
        Y = (cy - cy_i) * Z / fy

        # -------------------------
        # Publish pose
        # -------------------------
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "camera_color_optical_frame"

        pose.pose.position.x = float(X)
        pose.pose.position.y = float(Y)
        pose.pose.position.z = float(Z)

        qx, qy, qz, qw = rpy_to_quat(math.pi, 0, 0)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.pub.publish(pose)
        self.get_logger().info(f"Cube pose: {X:.3f}, {Y:.3f}, {Z:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseCubePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
