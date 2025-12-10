# Object Detection for Leo Rover

A real-time 3D object detection and analysis system designed for the Leo Rover platform. This system uses an Intel RealSense depth camera to detect objects, identify their colors, and classify their 3D shapes, providing spatial awareness for autonomous navigation and interaction.

## 🚀 Features

- **3D Object Detection**: Detect and track objects in 3D space
- **Color Recognition**: Identify object colors using K-means clustering in HSV space
- **Shape Classification**: Classify 3D shapes using contour and depth analysis
- **Real-time Processing**: Optimized for real-time performance on edge devices
- **Depth Sensing**: Utilizes Intel RealSense depth camera for accurate distance measurement
- **Edge Detection**: Combines color and depth data for robust edge detection

## 🛠️ Hardware Requirements

- Intel RealSense D400 series depth camera
- NVIDIA GPU (recommended for better performance)
- Leo Rover platform (compatible with other robotic platforms)

## 📦 Dependencies

- Python 3.8+
- OpenCV
- PyRealSense2
- PyTorch
- YOLOv8
- scikit-learn
- NumPy

See `requirements.txt` for the complete list of dependencies with specific versions.

## 🚀 Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/prathapselvakumar/Object-Detection-for-Leo-Rover.git
   cd Object-Detection-for-Leo-Rover
   ```

2. Create and activate a virtual environment (recommended):
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. Install the required packages:
   ```bash
   pip install -r requirements.txt
   ```

4. Connect your Intel RealSense camera and ensure it's properly recognized by your system.

## 🏃‍♂️ Usage

1. Run the main detection script:
   ```bash
   python object_shape_color_depth.py
   ```

2. The system will open three windows:
   - **3D Object Detection**: Main window showing detected objects with annotations
   - **Depth Map**: Visual representation of the depth data
   - **Edge Detection**: Output of the edge detection algorithm

3. Press 'q' to quit the application.

## 🎯 Object Requirements

For optimal performance, objects should:
- Have distinct colors (blue, green, yellow, orange, or red)
- Be well-lit
- Have clear edges and features
- Be at least 300 pixels in size (adjustable in code)

## 🛠️ Customization

### Color Detection
Modify the `classify_color()` function to adjust color ranges and add new colors.

### Object Detection
- Adjust minimum contour area in the main loop to change sensitivity
- Modify YOLO model parameters for different object detection needs

### Edge Detection
Tweak parameters in `detect_edges()` for different lighting conditions.

## 📊 Output

For each detected object, the system provides:
- Bounding box with color-coded edges
- 3D position (x, y, z) in millimeters
- Detected color
- Classified 3D shape
- Average depth from the camera

## 📂 Project Structure

```
Object-Detection-for-Leo-Rover/
├── object_shape_color_depth.py  # Main detection script
├── requirements.txt             # Python dependencies
└── README.md                   # This file
```
## 📝 Code Overview

The main script `object_shape_color_depth.py` implements the following workflow:

1. **Initialization**:
   - Sets up RealSense camera pipeline
   - Initializes YOLOv8 model
   - Configures color and depth streams

2. **Main Loop**:
   - Captures frames from the camera
   - Processes depth and color data
   - Detects edges and contours
   - Analyzes each detected object
   - Displays results in real-time

3. **Key Functions**:
   - `get_dominant_color()`: Identifies the dominant color in an object
   - `classify_color()`: Classifies colors using HSV color space
   - `preprocess_depth()`: Enhances depth data with filtering
   - `detect_edges()`: Combines color and depth for edge detection
   - `classify_3d_shape()`: Classifies 3D shapes using contour analysis

on GitHub.
