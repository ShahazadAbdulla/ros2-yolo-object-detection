# ROS 2 Phone Camera Object Detection with YOLO

This ROS 2 workspace contains a project demonstrating real-time object detection using a smartphone camera stream (via Android) integrated with YOLOv5/v8 through community ROS packages. This project is part of a guided learning journey into ROS 2.

## Current Status (as of 2025-04-22)

*   **Working:** Integration of an Android phone camera stream using the [`ros2-android-sensor-bridge`](https://github.com/VedantC2307/ros2-android-sensor-bridge) package (by Vedant Consul, inspired by Enzo Ghisoni). Phone publishes `sensor_msgs/msg/CompressedImage` on `/camera/image_raw/compressed`.
*   **Working:** Use of the standard ROS 2 `image_transport/republish` node to convert the compressed stream into a raw `sensor_msgs/msg/Image` stream published on `/image_raw`. This was necessary as a workaround for issues encountered when subscribing directly to the compressed topic with the YOLO node.
*   **Working:** Integration of the [`yolo_ros`](https://github.com/mgonzs13/yolo_ros) package (by `mgonzs13`) running YOLOv5n/YOLOv8n on CPU.
*   **Working:** The core `/yolo/yolo_node` successfully subscribes to the republished `/image_raw` topic.
*   **Working:** The `/yolo/yolo_node` publishes object detection results (bounding boxes, class names, scores) on the `/yolo/detections` topic (type `yolo_msgs/msg/DetectionArray`). Detection confirmed functional via `ros2 topic echo`.
*   **Known Issue:** The debug visualization topic (`/yolo/dbg_image`) published by the `debug_node` within the `yolo_ros` package did not appear correctly in `rqt_image_view` during testing in this setup, even when detections were being published. Therefore, the `debug_node` and `tracking_node` are currently disabled in the launch file for stable operation.
*   **Performance Note:** Running YOLOv5n/v8n purely on the CPU (Intel i5 10th Gen) results in significant CPU load and noticeable system lag, as expected. Real-time performance at high frame rates is limited by the CPU.

## Packages Used

This workspace utilizes the following key packages (located in `src/`):

1.  **`mobile_sensor`:** Clone of [`ros2-android-sensor-bridge`](https://github.com/VedantC2307/ros2-android-sensor-bridge). Provides the node and web interface to stream sensor data from an Android phone.
2.  **`yolo_ros`:** Clone of [`yolo_ros`](https://github.com/mgonzs13/yolo_ros). This repository actually contains 3 distinct ROS packages discovered by `colcon`:
    *   `yolo_msgs`: Defines custom message and service types for detections, tracking, etc.
    *   `yolo_ros`: Contains the core Python nodes for running YOLO inference (`yolo_node.py`), tracking (`tracking_node.py`), and debugging (`debug_node.py`).
    *   `yolo_bringup`: Contains launch files for starting the YOLO nodes.

## Key Concepts Demonstrated / Learned

*   ROS 2 Nodes (Python, Node.js via `mobile_sensor`)
*   ROS 2 Topics (Publish/Subscribe)
*   ROS 2 Messages (`sensor_msgs/msg/Image`, `sensor_msgs/msg/CompressedImage`, custom `yolo_msgs`)
*   ROS 2 Launch Files (Python-based, parameters, node execution, event handling - basic)
*   ROS 2 Parameters (Configuring nodes via launch file)
*   ROS 2 Packages (Structure, dependencies, building with `colcon`)
*   `image_transport` (Understanding compressed vs. raw, using `republish` node)
*   `cv_bridge` (Implicitly used by nodes for image conversion)
*   Integrating external hardware (Android phone) into ROS
*   Integrating Machine Learning models (YOLO) via community packages
*   Debugging ROS systems (`ros2 topic`, `ros2 node`, `rqt_graph`, `rqt_image_view`, log analysis)
*   Basic Git for version control

## Prerequisites

*   ROS 2 Humble Hawksbill installed.
*   Git.
*   Node.js (v20+) and npm (v8+). Check installation via `node -v` and `npm -v`. ([NodeSource Setup](https://github.com/nodesource/distributions#debian-and-ubuntu-based-distributions) recommended for Ubuntu).
*   Python 3 and Pip.
*   `colcon`, `rosdep`.
*   OpenSSL (usually default on Ubuntu, check `openssl version`).
*   Android smartphone (tested with Samsung S23) with a modern web browser (e.g., Chrome).
*   `rqt_image_view` and `image_transport_plugins`: `sudo apt update && sudo apt install ros-humble-rqt-image-view ros-humble-image-transport-plugins`

## Setup Instructions

1.  **Clone this Repository (Your Workspace):**
    ```bash
    git clone https://github.com/ShahazadAbdulla/ros2-yolo-object-detection.git ~/ros2_ws # Or your desired location
    cd ~/ros2_ws
    ```
    *(Note: This repo already includes the source for `mobile_sensor` and `yolo_ros` in the `src` directory, assuming they were committed previously).*

2.  **Install Node.js Dependencies:**
    ```bash
    cd src/mobile_sensor
    npm install
    cd ../..
    ```

3.  **Generate SSL Certificates:**
    ```bash
    cd src/mobile_sensor/src
    chmod +x generate_ssl_cert.sh
    ./generate_ssl_cert.sh
    cd ../../..
    ```

4.  **Install Python Dependencies:**
    ```bash
    pip install -r src/yolo_ros/requirements.txt
    ```

5.  **Install ROS Dependencies:**
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```

6.  **Build the Workspace:**
    ```bash
    colcon build --symlink-install
    ```

7.  **Install Android App:** Follow instructions in the `mobile_sensor` README/GitHub to install the companion app on your phone.

## How to Run (Current Working Configuration)

Ensure your phone and computer are on the same **reliable** network (use USB Tethering if WiFi is unstable and you configure it).

1.  **Source Workspace:** Open *every new terminal* and run:
    ```bash
    cd ~/ros2_ws
    source install/setup.bash
    ```

2.  **Terminal 1: Start Android Bridge:**
    ```bash
    ros2 launch mobile_sensor mobile_sensors.launch.py
    ```
    *   Find your computer's IP (`ip addr show`).
    *   Connect phone browser to `https://<COMPUTER_IP>:4000`.
    *   Accept security warning.
    *   Grant permissions in the app.
    *   Enable **Camera** and press **Start**.

3.  **Terminal 2: Start Image Republisher (Compressed -> Raw):**
    ```bash
    # Republishes to /image_raw
    ros2 run image_transport republish compressed raw --ros-args -r in/compressed:=/camera/image_raw/compressed -r out:=/image_raw
    ```
    *(Note: We are republishing directly to `/image_raw` here for simplicity)*

4.  **Terminal 3: Start YOLO Node:**
    ```bash
    ros2 launch yolo_bringup yolo.launch.py \
        model:=yolov5n.pt \
        device:=cpu \
        input_image_topic:=/image_raw \
        threshold:=0.15 \
        use_tracking:=False \
        use_debug:=False
    ```

5.  **Terminal 4: Monitor Detections:**
    ```bash
    ros2 topic echo /yolo/detections
    ```
    *   Point phone camera at common objects (cup, bottle, phone, keyboard, face). Detections should appear here.

## Known Issues & Challenges

*   The `/yolo/dbg_image` topic published by `yolo_ros`'s `debug_node` was not viewable in `rqt_image_view` in this setup. Debug node is currently disabled.
*   Significant CPU load and system lag when running YOLOv5n/v8n on CPU.
*   WiFi instability can disrupt the phone camera stream. USB Tethering recommended for reliability (requires configuring ROS 2 networking/DDS over USB).

## Next Steps

1.  **Create `detection_visualizer` Node:** Develop a new Python node in a separate package (`perception_utils` or similar) that subscribes to `/image_raw` and `/yolo/detections`, draws bounding boxes using OpenCV (`cv2`), and publishes the annotated image to a new topic (e.g., `/yolo/annotated_image`).
2.  **Performance Tuning:** Experiment with lower image resolution or framerate from the camera/`usb_cam`.
3.  **GPU Acceleration (Optional):** Investigate setting up CUDA/cuDNN and running YOLO on the MX130 GPU (`device:=cuda:0`) after installing necessary drivers and libraries.
4.  **Gazebo Integration (Optional):** Replicate this pipeline using a simulated camera in Gazebo with a TurtleBot3 or other simulated robot.
