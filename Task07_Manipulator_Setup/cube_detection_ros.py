#!/usr/bin/env python3
import time
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time as RosTime

import cv2
import numpy as np
import pyrealsense2 as rs
from sklearn.cluster import KMeans

# -------------------------
# Helper functions (adapted)
# -------------------------
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
    kmeans = KMeans(n_clusters=1, n_init=3)
    kmeans.fit(pixels)
    dominant_hsv = kmeans.cluster_centers_[0].astype(int)
    dominant_bgr = cv2.cvtColor(np.uint8([[dominant_hsv]]), cv2.COLOR_HSV2BGR)[0][0]
    color_name = classify_color(dominant_hsv)
    return tuple(map(int, dominant_bgr)), color_name

def classify_color(hsv_color):
    if isinstance(hsv_color, (list, tuple, np.ndarray)) and len(hsv_color) == 3:
        h, s, v = hsv_color
    else:
        hsv = cv2.cvtColor(np.uint8([[hsv_color]]), cv2.COLOR_BGR2HSV)[0][0]
        h, s, v = hsv
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
    combined_edges = cv2.bitwise_or(edges_color, edges_depth)
    kernel = np.ones((3,3), np.uint8)
    combined_edges = cv2.dilate(combined_edges, kernel, iterations=1)
    combined_edges = cv2.erode(combined_edges, kernel, iterations=1)
    return combined_edges

def classify_3d_shape(contour, depth_roi):
    epsilon = 0.02 * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True)
    vertices = len(approx)
    depth_values = depth_roi[depth_roi > 0].flatten()
    if len(depth_values) < 50:
        return "unknown", 0, 0, 0
    depth_mean = np.mean(depth_values)
    depth_std = np.std(depth_values)
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    circularity = 4 * math.pi * (area / (perimeter * perimeter + 1e-5))
    x, y, w, h = cv2.boundingRect(contour)
    aspect_ratio = float(w) / (h + 1e-5)
    if circularity > 0.85:
        if depth_std > 1500:
            return "sphere", depth_mean, depth_std, vertices
        else:
            return "cylinder", depth_mean, depth_std, vertices
    elif vertices == 3:
        if depth_std > 1000:
            return "pyramid", depth_mean, depth_std, vertices
        else:
            return "triangle", depth_mean, depth_std, vertices
    elif vertices == 4:
        if depth_std > 2000:
            return "cube", depth_mean, depth_std, vertices
        elif 0.9 < aspect_ratio < 1.1:
            return "square", depth_mean, depth_std, vertices
        else:
            return "rectangle", depth_mean, depth_std, vertices
    elif 5 <= vertices <= 8:
        if depth_std > 1800:
            return "prism", depth_mean, depth_std, vertices
        else:
            return f"{vertices}-sided polygon", depth_mean, depth_std, vertices
    else:
        return "complex shape", depth_mean, depth_std, vertices

# -------------------------
# ROS2 Node
# -------------------------
class RealSenseCubePublisher(Node):
    def __init__(self,
                 topic_name: str = "/cube_pose_camera",
                 camera_frame: str = "camera_color_optical_frame",
                 show_debug: bool = True):
        super().__init__("realsense_cube_publisher")
        self.publisher = self.create_publisher(PoseStamped, topic_name, 10)
        self.camera_frame = camera_frame
        self.show_debug = show_debug

        # RealSense setup
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.profile = self.pipeline.start(self.config)
        self.align = rs.align(rs.stream.color)

        # Timer to poll frames
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        self.get_logger().info("RealSenseCubePublisher started, publishing on: " + topic_name)

    def timer_callback(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=500)
        except Exception as e:
            self.get_logger().warn(f"RealSense wait_for_frames failed: {e}")
            return

        aligned = self.align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_scale = self.profile.get_device().first_depth_sensor().get_depth_scale()

        processed_depth = preprocess_depth(depth_image)
        edges = detect_edges(color_image, processed_depth)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Choose best candidate (largest cube-like contour)
        best_contour = None
        best_area = 0
        best_info = None

        for contour in contours:
            if cv2.contourArea(contour) < 300:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            padding = 5
            x0 = max(0, x - padding)
            y0 = max(0, y - padding)
            w = min(color_image.shape[1] - x0, w + 2*padding)
            h = min(color_image.shape[0] - y0, h + 2*padding)
            if w < 10 or h < 10:
                continue
            roi = color_image[y0:y0+h, x0:x0+w]
            depth_roi = depth_image[y0:y0+h, x0:x0+w]
            mask = np.zeros((h, w), dtype=np.uint8)
            # draw contour relative to ROI
            contour_shifted = contour - [x0, y0]
            cv2.drawContours(mask, [contour_shifted], -1, 255, -1)
            color_bgr, color_name = get_dominant_color(roi, mask)
            if color_name not in ["blue", "green", "yellow", "orange", "c52c2e_red"]:
                continue
            shape_name, depth_mean, depth_std, vertices = classify_3d_shape(contour, depth_roi)
            if shape_name == "unknown":
                continue
            area = cv2.contourArea(contour)
            if area > best_area:
                best_area = area
                best_contour = contour
                best_info = (x0, y0, w, h, color_name, shape_name)

        if best_contour is not None:
            x0, y0, w, h, color_name, shape_name = best_info
            M = cv2.moments(best_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                # depth in meters at pixel (cx, cy)
                depth_m = depth_frame.get_distance(cx, cy)
                if depth_m is None or depth_m <= 0.0:
                    # fallback: median of ROI
                    depth_vals = depth_image[cy-2:cy+3, cx-2:cx+3].astype(float) * depth_scale
                    depth_vals = depth_vals[depth_vals > 0]
                    if depth_vals.size == 0:
                        return
                    depth_m = float(np.median(depth_vals))

                # Build PoseStamped in camera frame
                pose = PoseStamped()
                now = self.get_clock().now().to_msg()
                pose.header.stamp = now
                pose.header.frame_id = self.camera_frame

                # Convert pixel + depth to 3D point in camera frame using intrinsics
                depth_intrin = depth_frame.profile.as_video_stream_profile().get_intrinsics()
                # deproject expects pixel coords as [u, v]
                point3d = rs.rs2_deproject_pixel_to_point(depth_intrin, [cx, cy], depth_m)
                # point3d is [X, Y, Z] in meters in camera frame
                pose.pose.position.x = float(point3d[0])
                pose.pose.position.y = float(point3d[1])
                pose.pose.position.z = float(point3d[2])

                # Orientation: tool pointing down (this is just a suggestion for downstream)
                roll, pitch, yaw = math.pi, 0.0, 0.0
                qx, qy, qz, qw = rpy_to_quat(roll, pitch, yaw)
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw

                # Publish
                self.publisher.publish(pose)
                self.get_logger().info(
                    f"Published cube pose: x={pose.pose.position.x:.3f} y={pose.pose.position.y:.3f} z={pose.pose.position.z:.3f} (frame={self.camera_frame})"
                )

                # Debug drawing
                if self.show_debug:
                    cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)
                    info_text = f"{shape_name} | {color_name} | {depth_m*1000:.1f}mm"
                    cv2.putText(color_image, info_text, (x0, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        # Show debug windows if enabled
        if self.show_debug:
            cv2.imshow("3D Object Detection", color_image)
            cv2.imshow("Depth Map", cv2.applyColorMap(
                cv2.normalize(processed_depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8),
                cv2.COLORMAP_JET
            ))
            cv2.imshow("Edge Detection", edges)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                # graceful shutdown requested by user
                rclpy.shutdown()

    def destroy_node(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseCubePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ == "__main__":
    main()
