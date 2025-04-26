# ROS 2 Phone Camera Object Detection with YOLO (and Visualization)

This ROS 2 workspace contains a project demonstrating real-time object detection using a smartphone camera stream (via Android) integrated with YOLOv5/v8 through community ROS packages. The detections are then visualized by drawing bounding boxes on the image stream using a custom ROS 2 node. This project is part of a guided learning journey into ROS 2.

## Current Status (as of 27-04-2025)

*   **Functional Pipeline:** Completed the full pipeline: Phone Camera -> Compressed Image Topic -> Image Republisher -> Raw Image Topic -> YOLO Node -> Detection Topic -> **Visualizer Node** -> Annotated Image Topic.
*   **Phone Camera Input:** Integration of an Android phone camera stream using [`ros2-android-sensor-bridge`](https://github.com/VedantC2307/ros2-android-sensor-bridge). Phone publishes `sensor_msgs/msg/CompressedImage`.
*   **Image Transport Workaround:** Use of `image_transport/republish` node converts compressed stream to raw `sensor_msgs/msg/Image` published on `/image_raw`.
*   **YOLO Detection:** Integration of [`yolo_ros`](https://github.com/mgonzs13/yolo_ros) package running YOLOv5n on CPU. `/yolo/yolo_node` subscribes to `/image_raw` and publishes `yolo_msgs/msg/DetectionArray` on `/yolo/detections`.
*   **Custom Visualization:** A custom Python node (`detection_visualizer/visualizer_node.py`) subscribes to `/image_raw` and `/yolo/detections`, draws bounding boxes and labels using OpenCV, and publishes the result as `sensor_msgs/msg/Image` on `/yolo/annotated_image`.
*   **Launch Automation:** A consolidated launch file (`yolo_bringup/launch/phone_yolo_pipeline.launch.py`) starts and sequences all required nodes (`mobile_sensor`, `republish`, `yolo_node`, `visualizer_node`) with appropriate delays.
*   **Performance Note:** Running YOLOv5n/v8n purely on the CPU (tested on Intel i5 10th Gen) results in significant CPU load and noticeable system lag. Visualization adds further processing load.

## Packages Used / Structure

This workspace (`ros2_ws`) utilizes the following key packages located in `src/`:

1.  **`mobile_sensor`:** Clone of [`ros2-android-sensor-bridge`](https://github.com/VedantC2307/ros2-android-sensor-bridge). Streams Android phone sensors.
2.  **`yolo_ros` (Git Repo containing multiple packages):** Clone of [`yolo_ros`](https://github.com/mgonzs13/yolo_ros). Builds the following ROS packages:
    *   `yolo_msgs`: Defines custom message/service types.
    *   `yolo_ros`: Contains the core Python nodes for inference (`yolo_node.py`) etc.
    *   `yolo_bringup`: Contains launch files, including the primary `phone_yolo_pipeline.launch.py`.
3.  **`detection_visualizer`:** Custom package containing the Python node (`visualizer_node.py`) for drawing detection bounding boxes.

## Key Concepts Demonstrated / Learned

*   ROS 2 Workspaces, Packages (Python, CMake), Build System (`colcon`, `rosdep`)
*   ROS 2 Nodes (Python, Node.js)
*   ROS 2 Topics (Publish/Subscribe) & Messages (`sensor_msgs/msg/Image`, `CompressedImage`, custom `yolo_msgs`)
*   ROS 2 Launch Files (Python syntax, Parameters, Remapping, Node Execution, Launch File Inclusion, Timers for Sequencing)
*   ROS 2 Parameters (Declaring & Getting in Python, Setting via Launch)
*   Lifecycle Nodes (Observing activation stages)
*   `image_transport` library (Compressed vs. Raw streams, `republish` node)
*   `cv_bridge` (ROS Image <-> OpenCV conversion)
*   OpenCV (Basic drawing: `cv2.rectangle`, `cv2.putText`)
*   Integrating external hardware (Android phone) and community ROS packages.
*   Integrating Machine Learning models (YOLO) via pre-trained weights.
*   Debugging ROS systems (`ros2 topic`, `ros2 node`, `rqt_graph`, `rqt_image_view`, logs, `htop`).
*   Basic Git for version control.

## Prerequisites

*   ROS 2 Humble Hawksbill installed (Desktop recommended).
*   Git.
*   Node.js (v20+) and npm (v8+). ([NodeSource Setup](https://github.com/nodesource/distributions#debian-and-ubuntu-based-distributions) for Ubuntu).
*   Python 3 and Pip (compatible with ROS 2 Humble).
*   Standard ROS 2 build tools (`colcon-common-extensions`, `rosdep`). Run `sudo rosdep init` and `rosdep update` if first time using.
*   OpenSSL (`openssl version`).
*   Android smartphone with a modern web browser (e.g., Chrome).
*   Required ROS visualization/plugin tools:
    ```bash
    sudo apt update && sudo apt install ros-humble-rqt-image-view ros-humble-image-transport-plugins ros-humble-rqt-graph python3-opencv && sudo apt clean
    ```

## Setup Instructions

1.  **Clone this Workspace Repository:**
    *(Replace URL with your actual GitHub repo URL)*
    ```bash
    git clone https://github.com/ShahazadAbdulla/ros2_yolo_object_detection.git ~/ros2_ws # Or your desired location
    cd ~/ros2_ws
    ```
    *(This assumes the repo contains `src/mobile_sensor`, `src/yolo_ros`, and `src/detection_visualizer`).*

2.  **Install Node.js Dependencies:**
    ```bash
    cd src/mobile_sensor
    npm install
    cd ../.. # Back to workspace root
    ```

3.  **Generate SSL Certificates:**
    ```bash
    cd src/mobile_sensor/src
    chmod +x generate_ssl_cert.sh
    ./generate_ssl_cert.sh
    cd ../../.. # Back to workspace root
    ```

4.  **Install Python Dependencies:**
    ```bash
    pip install -r src/yolo_ros/requirements.txt
    # Install OpenCV if not already present (needed for cv_bridge and visualizer)
    pip install opencv-python
    ```

5.  **Initialize and Install ROS Dependencies:**
    ```bash
    sudo rosdep init # Run once if you've never used rosdep
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    ```

6.  **Build the Workspace:**
    ```bash
    colcon build --symlink-install
    ```

7.  **Install Android App:** Follow instructions in `src/mobile_sensor/README.md` to install the companion app on your phone.

## How to Run the Full Pipeline

Ensure your phone and computer are on the same **reliable** network (USB Tethering is recommended if WiFi is unstable and can be configured).

1.  **Source Workspace:** Open *every new terminal* and run:
    ```bash
    cd ~/ros2_ws
    source install/setup.bash
    ```

2.  **Terminal 1: Run the Pipeline Launch File:** This file starts the bridge, republisher, YOLO node, and visualizer node in sequence.
    ```bash
    # Make sure filename matches what's in src/yolo_ros/yolo_bringup/launch/
    ros2 launch yolo_bringup phone_yolo_pipeline.launch.py
    ```

3.  **Terminal 1 (Continued): Connect Phone:**
    *   Wait for the mobile sensor bridge node to log its startup message (`HTTPS server running...`).
    *   Find your computer's IP (`ip addr show` or via tethering settings).
    *   Connect phone browser to `https://<COMPUTER_IP>:4000`.
    *   Accept security warning.
    *   Grant permissions & Start Camera stream in the app.

4.  **(Wait)** Allow time for all nodes to start sequentially (~9 seconds total defined in the launch file).

5.  **Terminal 2: View Annotated Image:**
    ```bash
    # Ensure workspace is sourced in this terminal too
    rqt_image_view /yolo/annotated_image
    ```
    *   Point phone camera at common objects. The `rqt_image_view` window should show the video feed with bounding boxes drawn.

## Credits & Acknowledgements

*   **`ros2-android-sensor-bridge` Package:** Created by **Vedant Consul**. Inspired by **Enzo Ghisoni**'s demonstration post [Optional: Add link here]. ([GitHub Repo](https://github.com/VedantC2307/ros2-android-sensor-bridge))
*   **`yolo_ros` Package:** Created by **`mgonzs13`**. ([GitHub Repo](https://github.com/mgonzs13/yolo_ros))
*   **YOLO Models & Ultralytics:** Based on the work by Ultralytics and the original YOLO authors. ([Ultralytics](https://ultralytics.com/))
*   **ROS 2 Tutorials & Community:** Leveraging concepts and tools from the broader ROS 2 ecosystem and documentation.

## Next Steps / Future Work

*   **Performance Tuning:** Experiment with lower image resolution/framerate.
*   **Explore GPU Acceleration (Optional):** Investigate CUDA setup for available GPU and run YOLO with `device:=cuda:0`.
*   **Gazebo Integration (Project 3b):** Replicate a similar detection pipeline using a simulated camera in Gazebo.
*   **C++ Implementation:** Potentially reimplement parts of this pipeline in C++ for performance comparison and learning.
