# ROS 2 Phone Camera Object Detection Project

This ROS 2 workspace contains a project demonstrating real-time object detection using a smartphone camera stream and YOLOv5/v8.

Currently achieved status (as of YYYY-MM-DD):
- Integration of Android phone camera using `ros2-android-sensor-bridge`.
- Image stream published on `/camera/image_raw/compressed`.
- Use of `image_transport/republish` node to convert compressed stream to `/image_raw` (raw `sensor_msgs/msg/Image`).
- Integration of `yolo_ros` package (`mgonzs13/yolo_ros` fork) running YOLOv5n/YOLOv8n on CPU.
- YOLO node subscribes to `/image_raw` and publishes detections on `/yolo/detections`.
- Basic detection confirmed working via `ros2 topic echo`.

## Packages Included

*   `src/mobile_sensor`: Clone of [ros2-android-sensor-bridge](https://github.com/VedantC2307/ros2-android-sensor-bridge)
*   `src/yolo_ros`: Clone of [yolo_ros](https://github.com/mgonzs13/yolo_ros) (contains `yolo_ros`, `yolo_msgs`, `yolo_bringup` internal packages)
*   (TODO: Add `detection_visualizer` package once created)

## Setup

1.  Install ROS 2 Humble, Node.js (v20+), `colcon`, `git`.
2.  Install Python dependencies for yolo_ros: `pip install -r src/yolo_ros/requirements.txt`
3.  Install Node.js dependencies for mobile_sensor: `cd src/mobile_sensor && npm install && cd ../..`
4.  Generate SSL certs for mobile_sensor: `cd src/mobile_sensor/src && chmod +x generate_ssl_cert.sh && ./generate_ssl_cert.sh && cd ../../..`
5.  Install ROS dependencies: `rosdep install --from-paths src --ignore-src -r -y`
6.  Build the workspace: `colcon build --symlink-install`
7.  Source the workspace: `source install/setup.bash`
8.  Install and configure the Android app from `ros2-android-sensor-bridge`.

## How to Run (Current State)

*Terminal 1: Start Android Bridge*
```bash
ros2 launch mobile_sensor mobile_sensors.launch.py
