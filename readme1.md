# PointCloudPoseEstimator

## Overview
The `point_cloud_pose_estimator` package provides two ROS 2 nodes for detecting a football in camera images and estimating its 3D pose using a point cloud:
- **FootballDetectorNode**: Uses YOLO to detect a football in images from `/camera_image`, publishing pixel coordinates to `/pixel_coordinates`.
- **PointEstimatorNode**: Converts pixel coordinates to 3D poses using point cloud data from `/depth_camera/points`, publishing to `/object_pose`.

## Dependencies
- ROS 2 (tested on Humble/Iron)
- Python 3
- ROS 2 packages: `rclpy`, `sensor_msgs`, `geometry_msgs`, `tf2_ros`, `tf2_geometry_msgs`
- Python libraries: `numpy`, `opencv-python`, `ultralytics`

Install ROS 2 dependencies:
```bash
sudo apt install ros-<distro>-sensor-msgs ros-<distro>-geometry-msgs ros-<distro>-tf2-ros ros-<distro>-tf2-geometry-msgs
```
Install Python dependencies:
```bash
pip install numpy opencv-python ultralytics
```

## Package Structure
- `point_cloud_pose_estimator/football_detector_node.py`: Detects football in images and publishes pixel coordinates.
- `point_cloud_pose_estimator/point_estimator_node.py`: Estimates 3D pose from pixel coordinates and point cloud.

## Setup
1. **Create a ROS 2 workspace** (if not already created):
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. **Clone or create the package**:
   If cloning, use:
   ```bash
   git clone <repository_url>
   ```
   Or create manually (see package creation steps).

3. **Build the workspace**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select point_cloud_pose_estimator
   ```

4. **Source the workspace**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage
1. **Run the FootballDetectorNode**:
   ```bash
   ros2 run point_cloud_pose_estimator football_detector_node
   ```
   - Subscribes to `/camera_image` (sensor_msgs/Image).
   - Publishes to `/pixel_coordinates` (geometry_msgs/Point) and `/football_detector/image` (sensor_msgs/Image).
   - Parameters:
     - `model_path`: Path to YOLO model (default: `yolov8n.pt`).
     - `class_id`: YOLO class ID for football (default: 32).
     - `conf_threshold`: Confidence threshold for detection (default: 0.5).

2. **Run the PointEstimatorNode**:
   ```bash
   ros2 run point_cloud_pose_estimator point_estimator_node
   ```
   - Subscribes to `/depth_camera/points` (sensor_msgs/PointCloud2) and `/pixel_coordinates` (geometry_msgs/Point).
   - Publishes to `/object_pose` (geometry_msgs/PoseStamped).
   - Parameters:
     - `base_frame`: Target frame for pose (default: `panda_linkG`).
     - `camera_frame`: Camera frame (default: `camera_link`).
     - `transform_timeout`: Timeout for transform lookup (default: 1.0).
     - `search_radius`: Pixel radius for averaging points (default: 10).
     - `point_cloud_topic`: Point cloud topic (default: `/depth_camera/points`).
     - `pixel_coordinates_topic`: Pixel coordinates topic (default: `/pixel_coordinates`).
     - `object_pose_topic`: Output pose topic (default: `/object_pose`).

3. **Example launch**:
   Run both nodes together (create a launch file or run separately):
   ```bash
   ros2 run point_cloud_pose_estimator football_detector_node &
   ros2 run point_cloud_pose_estimator point_estimator_node
   ```

## Notes
- Ensure a camera publishes to `/camera_image` and a depth sensor to `/depth_camera/points`.
- A valid transform between `camera_frame` and `base_frame` must exist (use `tf2` tools to verify).
- Download a YOLO model (e.g., `yolov8n.pt`) and specify its path via the `model_path` parameter.
- Adjust `class_id` if the football class ID differs in your YOLO model.