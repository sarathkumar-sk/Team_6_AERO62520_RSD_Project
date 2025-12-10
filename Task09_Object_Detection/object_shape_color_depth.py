import cv2
import numpy as np
import pyrealsense2 as rs
from sklearn.cluster import KMeans
from ultralytics import YOLO
import torch

# Check if CUDA is available
device = 'cuda' if torch.cuda.is_available() else 'cpu'
print(f"Using device: {device}")

# Initialize YOLOv8n model
model = YOLO('yolov8n.pt').to(device)

# ---------- RealSense Setup ----------
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
profile = pipeline.start(config)

align = rs.align(rs.stream.color)

# ---------- Helper Functions ----------

def get_dominant_color(image, mask):
    """Use KMeans to detect dominant color in ROI using HSV color space."""
    # Convert to HSV for better color clustering
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    pixels = hsv_image[mask == 255].reshape(-1, 3)
    
    if len(pixels) < 50:
        return (0, 0, 0), "unknown"
    
    # Use KMeans to find dominant color in HSV space
    kmeans = KMeans(n_clusters=1, n_init=3)
    kmeans.fit(pixels)
    dominant_hsv = kmeans.cluster_centers_[0].astype(int)
    
    # Convert back to BGR for display
    dominant_bgr = cv2.cvtColor(np.uint8([[dominant_hsv]]), cv2.COLOR_HSV2BGR)[0][0]
    color_name = classify_color(dominant_hsv)
    
    return tuple(map(int, dominant_bgr)), color_name

def classify_color(hsv_color):
    """
    Classify color using HSV color space.
    
    Args:
        hsv_color: A 3-element list/tuple/array containing HSV values
                  (H: 0-180, S: 0-255, V: 0-255)
    
    Returns:
        str: Color name or "other" if no match found
    """
    if isinstance(hsv_color, (list, tuple, np.ndarray)) and len(hsv_color) == 3:
        h, s, v = hsv_color
    else:
        # If input is BGR, convert to HSV
        hsv = cv2.cvtColor(np.uint8([[hsv_color]]), cv2.COLOR_BGR2HSV)[0][0]
        h, s, v = hsv
    
    # Define color ranges in HSV
    # Hue range: 0-180 (OpenCV uses 0-180 for H in HSV)
    # Saturation: 0-255
    # Value: 0-255
    
    # Check for low saturation or value (grayscale/black/white)
    if s < 50 or v < 50:
        return "other"
    
    # Define color ranges with saturation and value thresholds
    if (0 <= h <= 10 or 170 <= h <= 180) and s > 100:  # Red range
        # C52C2E is a deep red with high saturation
        if 0 <= h <= 10 and s > 150 and v > 100:
            return "c52c2e_red"
        return "red"
    elif 11 <= h <= 20 and s > 100:  # Orange
        return "orange"
    elif 21 <= h <= 35 and s > 50:  # Yellow
        return "yellow"
    elif 36 <= h <= 85 and s > 50:  # Green
        return "green"
    elif 86 <= h <= 125 and s > 50:  # Blue
        return "blue"
    elif 126 <= h <= 140 and s > 50:  # Purple
        return "purple"
   
    
    return "other"

def preprocess_depth(depth_frame):
    """Enhance depth data with filtering and noise reduction."""
    # Convert to 8-bit for processing
    depth_image = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    
    # Apply bilateral filter for edge-preserving smoothing
    depth_image = cv2.bilateralFilter(depth_image, 9, 75, 75)
    
    # Apply morphological operations to reduce noise
    kernel = np.ones((3,3), np.uint8)
    depth_image = cv2.morphologyEx(depth_image, cv2.MORPH_OPEN, kernel)
    depth_image = cv2.morphologyEx(depth_image, cv2.MORPH_CLOSE, kernel)
    
    return depth_image

def detect_edges(image, depth_data):
    """Enhanced edge detection combining color and depth information."""
    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Normalize depth data
    depth_norm = cv2.normalize(depth_data, None, 0, 255, cv2.NORM_MINMAX)
    
    # Edge detection on color image
    edges_color = cv2.Canny(gray, 50, 150)
    
    # Edge detection on depth map
    edges_depth = cv2.Canny(depth_norm.astype(np.uint8), 25, 75)
    
    # Combine both edge maps
    combined_edges = cv2.bitwise_or(edges_color, edges_depth)
    
    # Apply edge enhancement
    kernel = np.ones((3,3), np.uint8)
    combined_edges = cv2.dilate(combined_edges, kernel, iterations=1)
    combined_edges = cv2.erode(combined_edges, kernel, iterations=1)
    
    return combined_edges

def classify_3d_shape(contour, depth_roi):
    """Enhanced 3D shape classification using contour and depth analysis."""
    # Fit polygon to contour
    epsilon = 0.02 * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True)
    vertices = len(approx)
    
    # Process depth data
    depth_values = depth_roi[depth_roi > 0].flatten()
    if len(depth_values) < 50:  # Minimum points for reliable analysis
        return "unknown", 0, 0, 0
    
    # Calculate depth statistics
    depth_mean = np.mean(depth_values)
    depth_std = np.std(depth_values)
    depth_var = np.var(depth_values)
    
    # Calculate contour properties
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    circularity = 4 * np.pi * (area / (perimeter * perimeter + 1e-5))
    
    # Aspect ratio
    x, y, w, h = cv2.boundingRect(contour)
    aspect_ratio = float(w) / (h + 1e-5)
    
    # Shape classification with depth analysis
    if circularity > 0.85:  # Close to circle
        if depth_std > 1500:  # Significant depth variation
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

# ---------- Main Loop ----------
try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()

        if not depth_frame or not color_frame:
            continue
            
        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        
        # Get depth scale for converting depth units to meters
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        
        # Preprocess depth data
        processed_depth = preprocess_depth(depth_image)
        
        # Enhanced edge detection
        edges = detect_edges(color_image, processed_depth)
        
        # Find contours from edges
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Process each detected contour
        for contour in contours:
            # Filter small contours
            if cv2.contourArea(contour) < 300:  # Reduced minimum area for better detection
                continue
                
            # Get bounding box with padding
            padding = 5
            x, y, w, h = cv2.boundingRect(contour)
            x = max(0, x - padding)
            y = max(0, y - padding)
            w = min(color_image.shape[1] - x, w + 2*padding)
            h = min(color_image.shape[0] - y, h + 2*padding)
            
            # Skip if ROI is too small
            if w < 10 or h < 10:
                continue
                
            # Get ROI for color and depth analysis
            roi = color_image[y:y+h, x:x+w]
            depth_roi = depth_image[y:y+h, x:x+w]
            
            # Create mask for the current contour
            mask = np.zeros_like(edges)
            cv2.drawContours(mask, [contour], -1, 255, -1)
            
            # Get dominant color
            color, color_name = get_dominant_color(roi, mask[y:y+h, x:x+w])
            
            # Skip if not in our target colors
            if color_name not in ["blue", "green", "yellow", "orange", "c52c2e_red"]:
                continue
                
            # Classify 3D shape with enhanced depth analysis
            shape_name, depth_mean, depth_std, vertices = classify_3d_shape(contour, depth_roi)
            
            # Skip if shape classification failed
            if shape_name == "unknown":
                continue
            
            # Draw contour with color based on detected color
            contour_color = {
                "blue": (255, 0, 0),       # Blue in BGR
                "green": (0, 255, 0),      # Green in BGR
                "yellow": (0, 255, 255),   # Yellow in BGR
                "orange": (0, 165, 255),   # Orange in BGR
                "c52c2e_red": (46, 44, 197)  # C52C2E in BGR
            }.get(color_name, (0, 255, 0))
            
            cv2.drawContours(color_image, [contour], -1, contour_color, 2)
            
            # Calculate 3D position (center of mass)
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                depth_mm = depth_image[cy, cx] * depth_scale * 1000  # Convert to mm
                
                # Draw 3D position
                pos_text = f"({cx}, {cy}, {depth_mm:.1f}mm)"
                cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(color_image, pos_text, (cx + 10, cy), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
            # Draw shape and depth info with color-coded text
            info_text = f"{shape_name} | {color_name} | {depth_mean:.1f}mm"
            cv2.putText(color_image, info_text, (x, y),
                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, contour_color, 2)
            
            # Draw edges with enhanced visibility
            edge_mask = np.zeros_like(color_image)
            edge_mask[edges > 0] = (0, 0, 255)  # Red edges
            color_image = cv2.addWeighted(color_image, 0.8, edge_mask, 0.2, 0)
        
        # Display the results
        cv2.imshow("3D Object Detection", color_image)
        cv2.imshow("Depth Map", cv2.applyColorMap(
            cv2.normalize(processed_depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8),
            cv2.COLORMAP_JET
        ))
        cv2.imshow("Edge Detection", edges)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    cv2.destroyAllWindows()
