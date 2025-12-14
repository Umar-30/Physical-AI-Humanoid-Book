# Chapter 2: Hardware-Accelerated Perception (Isaac ROS)

This chapter focuses on integrating Isaac ROS into your ROS 2 pipeline on a Jetson Orin Nano/NX to perform hardware-accelerated perception tasks.

## 2.1 Setting Up Isaac ROS on Jetson Orin Nano/NX

NVIDIA Isaac ROS is a collection of hardware-accelerated packages for ROS 2, leveraging the power of NVIDIA GPUs (especially Jetson platforms) for robotics perception and navigation tasks. This section guides you through setting up your Jetson Orin Nano/NX for Isaac ROS development.

### 2.1.1 Prerequisites (Review)

Before proceeding, ensure your Jetson Orin Nano/NX meets the following:
*   Latest JetPack SDK (5.x or newer, compatible with Ubuntu 20.04/22.04 base for ROS 2 Humble) flashed and configured. This includes CUDA, cuDNN, and TensorRT.
*   ROS 2 Humble installed (refer to `environment-setup.md`).
*   Docker and NVIDIA Container Toolkit installed and configured (refer to `environment-setup.md`).

### 2.1.2 Installing Isaac ROS Packages

Isaac ROS packages are typically installed either from pre-built Docker containers or by building from source within a ROS 2 workspace. For simplicity and consistency, we will focus on using Docker containers for deployment and development.

1.  **Create a ROS 2 Workspace (if you don't have one)**:
    ```bash
    mkdir -p ~/isaac_ros_ws/src
    cd ~/isaac_ros_ws
    vcs import src < https://github.com/NVIDIA-AI-ROBOTICS/isaac_ros_common/raw/main/isaac_ros_common.repos
    ```
    This imports the `isaac_ros_common` repository, which contains helper scripts and Dockerfiles.

2.  **Build the Isaac ROS Development Container**:
    The `isaac_ros_common` package provides scripts to build a Docker development environment.
    ```bash
    cd ~/isaac_ros_ws/src/isaac_ros_common
    ./scripts/build_ros_workspace.sh
    ```
    This script will build a Docker image named `isaac_ros_dev` (or similar) containing ROS 2 Humble and the necessary Isaac ROS dependencies. This might take a significant amount of time.

3.  **Launch the Isaac ROS Development Container**:
    Once the image is built, you can launch a shell inside it.
    ```bash
    cd ~/isaac_ros_ws/src/isaac_ros_common
    ./scripts/run_dev_container.sh
    ```
    You should now be inside the Docker container, with your `isaac_ros_ws` workspace mounted and sourced. Your prompt might change to something like `(isaac_ros_dev) user@hostname:~/workspaces/isaac_ros_ws/src/isaac_ros_common$`.

4.  **Install Specific Isaac ROS ROS 2 Packages**:
    Inside the container, you can now install specific Isaac ROS ROS 2 packages relevant to your perception tasks (e.g., `isaac_ros_vslam`, `isaac_ros_dnn_image_encoder`).
    ```bash
    cd ~/isaac_ros_ws
    rosdep install --from-paths src --ignore-src --rosdistro humble -y
    colcon build --packages-up-to isaac_ros_vslam isaac_ros_dnn_image_encoder # Add other packages as needed
    source install/setup.bash
    ```
    *Note*: Isaac ROS packages are also available as pre-built Debian packages, but building from source in the dev container ensures compatibility with your specific JetPack version.

### 2.1.3 Understanding Isaac ROS Architecture

Isaac ROS packages are designed to optimize common robotics algorithms for NVIDIA hardware. Key aspects:
*   **ROS 2 Compatibility**: Seamlessly integrates with the ROS 2 ecosystem.
*   **Hardware Acceleration**: Utilizes CUDA, cuDNN, TensorRT, and other NVIDIA libraries to accelerate computationally intensive tasks (e.g., image processing, DNN inference, VSLAM).
*   **Graph-based Processing**: Many Isaac ROS components leverage NVIDIA's `Graph Composer` or are designed to be part of a larger graph for efficient data flow.
*   **Containerization**: Often provided within Docker containers for easy deployment and reproducible environments on Jetson devices.

By completing this setup, your Jetson Orin Nano/NX is ready to run hardware-accelerated perception pipelines.

## 2.2 Implementing Visual SLAM with Isaac ROS

Visual SLAM (Simultaneous Localization and Mapping) is a crucial technology for autonomous robots, allowing them to build a map of an unknown environment while simultaneously estimating their own position within that map. Isaac ROS provides hardware-accelerated Visual SLAM solutions that are highly optimized for NVIDIA Jetson platforms.

### 2.2.1 Visual SLAM vs. Wheel Odometry

Traditional wheel odometry estimates a robot's pose by integrating wheel encoder data. While simple, it suffers from **drift**, where small errors accumulate over time, leading to significant positional inaccuracies. This drift is particularly problematic for humanoid robots or robots operating on uneven terrain where wheel slip is common.

**Visual SLAM offers significant advantages**:
*   **Reduced Drift**: By using visual features from a camera, VSLAM constantly corrects its pose estimate against observed landmarks in the map, significantly reducing cumulative error.
*   **Environment Understanding**: VSLAM simultaneously builds a map of the environment, which can be used for navigation, obstacle avoidance, and interaction.
*   **No Dependence on Wheel Encoders**: Critical for humanoid robots or drones that lack traditional wheels.

### 2.2.2 Isaac ROS VSLAM Module

Isaac ROS offers `isaac_ros_vslam`, a package that provides high-performance visual odometry and mapping capabilities. It typically uses an RGB-D camera (RGB image + Depth image) to estimate pose and reconstruct the environment.

1.  **Prepare Your Data Source**:
    You'll need a camera publishing RGB and Depth images to ROS 2 topics. This can be:
    *   A simulated camera in Isaac Sim or Gazebo (as configured in Module 2, Chapter 2).
    *   A real RGB-D camera connected to your Jetson (e.g., ZED camera, RealSense).
    *   For this example, we assume you have topics `/my_robot/camera/image_raw` (sensor_msgs/msg/Image) and `/my_robot/camera/depth/image_raw` (sensor_msgs/msg/Image), and corresponding `/my_robot/camera/camera_info` (sensor_msgs/msg/CameraInfo). If using Isaac Sim, you'll need to bridge these topics from Isaac Sim to your ROS 2 environment.

2.  **Launch `isaac_ros_vslam`**:
    Isaac ROS VSLAM comes with a launch file for easy setup. You'll typically configure it to subscribe to your camera topics.

    First, ensure you are in the Isaac ROS development container on your Jetson.

    ```bash
    # Assuming you are inside the isaac_ros_dev Docker container
    # Source your ROS 2 workspace (if not already sourced)
    source install/setup.bash

    # Example: Launch VSLAM with a simulated camera (adjust topic names as needed)
    ros2 launch isaac_ros_vslam isaac_ros_vslam_isaac_sim.launch.xml \
        camera_topic:=/my_robot/camera/image_raw \
        camera_info_topic:=/my_robot/camera/camera_info \
        depth_topic:=/my_robot/camera/depth/image_raw
    ```
    *   **Explanation**:
        *   `isaac_ros_vslam_isaac_sim.launch.xml`: A pre-configured launch file for VSLAM, often tailored for simulated camera inputs.
        *   `camera_topic`, `camera_info_topic`, `depth_topic`: Remaps the default VSLAM input topics to match your camera's output topics.

3.  **Visualize VSLAM Output in RViz2**:
    VSLAM publishes various outputs to ROS 2 topics, including:
    *   `/vslam/camera_pose`: Estimated camera pose (geometry_msgs/msg/PoseStamped).
    *   `/vslam/map_points`: 3D points representing the map (sensor_msgs/msg/PointCloud2).
    *   `/vslam/odometry`: Odometry information (nav_msgs/msg/Odometry).

    To visualize:
    ```bash
    # In a new terminal on your workstation (if Isaac Sim/Gazebo are running there)
    # or on your Jetson (if running a real camera or forwarding display)
    rviz2
    ```
    In RViz2:
    *   Set `Fixed Frame` to `odom` or `map`.
    *   Add a `Pose` display for `/vslam/camera_pose`.
    *   Add a `PointCloud2` display for `/vslam/map_points`.
    *   As the camera moves (either simulated or real), you should see the robot's estimated trajectory and the reconstructed 3D map points in RViz2.

### 2.2.3 Considerations for Humanoid Robots

For bipedal or humanoid robots, VSLAM is particularly valuable as it eliminates dependence on often unreliable wheel odometry. However, specific considerations apply:
*   **Camera Placement**: Strategic camera placement to capture distinctive features in the environment.
*   **Motion Model**: Integrating IMU data with VSLAM (often done internally by the VSLAM algorithm or via sensor fusion) can improve robustness, especially during rapid movements or falls.
*   **Dynamic Environments**: VSLAM performance can degrade in highly dynamic environments (many moving objects).

Isaac ROS VSLAM provides a robust foundation for building autonomous navigation and perception capabilities for a wide range of robots, including complex humanoid platforms.

## 2.3 Hardware-Accelerated DNN Inference with Isaac ROS (TensorRT)

Deep Neural Networks (DNNs) are at the heart of modern robotic perception, enabling tasks like object detection, segmentation, and pose estimation. Running these complex models efficiently on edge devices like the Jetson Orin Nano/NX requires hardware acceleration. Isaac ROS leverages NVIDIA's TensorRT to optimize and execute DNNs at high performance.

### 2.3.1 The Need for TensorRT

Training a DNN often happens on powerful desktop GPUs or in the cloud using frameworks like PyTorch or TensorFlow. However, deploying these models to an embedded system like the Jetson presents challenges:
*   **Limited Resources**: Jetsons have less compute power and memory than desktop GPUs.
*   **Real-time Requirements**: Robotics applications often demand very low latency inference.

**TensorRT is NVIDIA's SDK for high-performance deep learning inference.** It optimizes trained neural networks for deployment on NVIDIA GPUs by:
*   **Graph Optimization**: Fusing layers, eliminating redundant operations.
*   **Precision Calibration**: Quantizing models to lower precision (FP16, INT8) with minimal accuracy loss.
*   **Kernel Auto-tuning**: Selecting the best performing GPU kernels for specific layers.

### 2.3.2 Model Conversion from PyTorch/TensorFlow to TensorRT

To use TensorRT, you typically convert your trained model from its original framework format (e.g., `.pt` for PyTorch, `.pb` for TensorFlow) into a TensorRT engine.

1.  **Export to ONNX**:
    Most frameworks can export models to the Open Neural Network Exchange (ONNX) format, which acts as an intermediate representation.
    *   **PyTorch Example**:
        ```python
        import torch
        import torch.onnx

        # Assuming 'model' is your trained PyTorch model
        # Assuming 'dummy_input' is a tensor with the expected input shape
        torch.onnx.export(model, dummy_input, "model.onnx", verbose=True, opset_version=11)
        ```
    *   **TensorFlow/Keras Example**:
        You would typically use `tf.saved_model.save()` and then a tool like `tf2onnx` to convert.
        ```bash
        # Example using tf2onnx (install with pip install tf2onnx)
        python -m tf2onnx.convert --saved-model /path/to/your/tf_saved_model --output model.onnx
        ```

2.  **Convert ONNX to TensorRT Engine**:
    NVIDIA provides tools like `trtexec` (part of the TensorRT SDK) or Python APIs to convert ONNX models into optimized TensorRT engines (`.trt` or `.engine` files).
    ```bash
    # Assuming you are on your Jetson or a machine with TensorRT installed
    # Example using trtexec (for ONNX model)
    trtexec --onnx=model.onnx --saveEngine=model.engine --fp16 --verbose
    ```
    *   **Explanation**:
        *   `--onnx=model.onnx`: Specifies the input ONNX model.
        *   `--saveEngine=model.engine`: Saves the optimized TensorRT engine to this file.
        *   `--fp16`: Enables FP16 (half-precision float) inference, which often provides a good balance between speed and accuracy on Jetsons. `--int8` can be used for even greater acceleration but requires careful calibration.
        *   `--verbose`: Provides detailed output during the conversion process.

### 2.3.3 Isaac ROS for DNN Deployment

Isaac ROS simplifies deploying TensorRT-optimized models into ROS 2 pipelines on Jetson. Key packages include:
*   `isaac_ros_image_proc`: For basic image processing.
*   `isaac_ros_dnn_image_encoder`: Encodes raw images into a format suitable for DNN inference.
*   `isaac_ros_tensor_rt`: Provides a ROS 2 node to run TensorRT engines.

1.  **Prepare Your ROS 2 Workspace**:
    Ensure your Jetson is in the Isaac ROS development container.

2.  **Create a ROS 2 Package for Your DNN Node**:
    ```bash
    cd ~/isaac_ros_ws/src
    ros2 pkg create --build-type ament_python my_dnn_node --dependencies rclpy sensor_msgs cv_bridge vision_msgs
    ```

3.  **Implement Your DNN Inference Node**:
    Create a Python node in your `my_dnn_node` package (e.g., `~/isaac_ros_ws/src/my_dnn_node/my_dnn_node/dnn_inference_node.py`) that uses the `isaac_ros_tensor_rt` node.
    This typically involves:
    *   Subscribing to camera image topics (e.g., `/my_robot/camera/image_raw`).
    *   Passing the image through `isaac_ros_dnn_image_encoder`.
    *   Using the `isaac_ros_tensor_rt` node to perform inference with your `.engine` file.
    *   Publishing the inference results (e.g., bounding boxes as `vision_msgs/msg/Detection2DArray`).

    **Conceptual `launch.py` (example for running a TensorRT engine):**
    ```python
    from launch import LaunchDescription
    from launch_ros.actions import Node
    from launch_ros.substitutions import FindPackageShare
    import os

    def generate_launch_description():
        tensorrt_engine_path = os.path.join(
            FindPackageShare('my_dnn_node').perform(None),
            'model',
            'model.engine'
        )

        return LaunchDescription([
            Node(
                package='isaac_ros_dnn_image_encoder',
                executable='isaac_ros_dnn_image_encoder',
                name='image_encoder',
                namespace='my_robot',
                output='screen',
                parameters=[{
                    'input_image_width': 640,
                    'input_image_height': 480,
                    'input_image_channels': 3,
                    'output_binding_names': ['input_tensor'], # Adjust based on your model's input
                    'tensor_name_map_path': '', # Path to a JSON map if needed
                }],
                remappings=[
                    ('encoded_tensor', 'image_tensor'),
                    ('image', 'camera/image_raw')
                ]
            ),
            Node(
                package='isaac_ros_tensor_rt',
                executable='isaac_ros_tensor_rt',
                name='tensor_rt_node',
                namespace='my_robot',
                output='screen',
                parameters=[{
                    'model_file_path': tensorrt_engine_path,
                    'input_tensor_names': ['input_tensor'], # Adjust based on your model's input
                    'output_tensor_names': ['output_detections'], # Adjust based on your model's output
                }],
                remappings=[
                    ('tensor_pub', 'image_tensor'),
                    ('tensor_sub', 'inference_output')
                ]
            ),
            # Add your custom post-processing node here to convert 'inference_output' to vision_msgs
            # Node(
            #     package='my_dnn_node',
            #     executable='post_processor',
            #     name='post_processor_node',
            #     namespace='my_robot',
            #     output='screen',
            #     remappings=[
            #         ('input_tensor', 'inference_output'),
            #         ('output_detections', 'detections')
            #     ]
            # )
        ])
    ```
    *   **Explanation**: This launch file sets up a pipeline:
        *   `image_encoder`: Takes the raw camera image and converts it to a tensor format expected by the DNN.
        *   `tensor_rt_node`: Loads and runs your `model.engine` using TensorRT, taking the encoded tensor as input and producing raw tensor outputs.
        *   (Optional) `post_processor_node`: A custom Python node to interpret the raw tensor output from `tensor_rt_node` and convert it into structured ROS 2 messages like `vision_msgs/msg/Detection2DArray` for further use (e.g., visualization in RViz2, input to Nav2).

By leveraging TensorRT and Isaac ROS, you can achieve impressive DNN inference speeds on your Jetson, enabling real-time perception for complex robotic tasks.

## 2.4 Chapter 2 Project/Checkpoint: VSLAM and DNN Inference

This project integrates Visual SLAM and hardware-accelerated DNN inference to create a robust perception pipeline on your Jetson Orin Nano/NX.

### Project Goal

Demonstrate the simultaneous operation of Isaac ROS VSLAM for robot localization and mapping, and a TensorRT-optimized DNN for object detection (or a similar perception task) on a live camera stream on your Jetson Orin Nano/NX. Visualize the results in RViz2.

### Requirements

1.  **Environment Setup**: Ensure your Jetson Orin Nano/NX is configured with Isaac ROS development environment and necessary packages, as described in Section 2.1.
2.  **Camera Input**: Have a source of RGB-D camera data on your Jetson. This can be a real camera connected to the Jetson, or a simulated camera publishing to ROS 2 topics if you have a bridge set up (e.g., from Isaac Sim running on a workstation, with topics forwarded to the Jetson).
3.  **VSLAM Launch**:
    *   Launch the `isaac_ros_vslam` node subscribing to your camera's RGB, Depth, and CameraInfo topics.
    *   Ensure the VSLAM is able to track the robot's pose and build a map as the camera moves.
4.  **DNN Inference Launch**:
    *   Select a pre-trained object detection model (e.g., YOLOv4, SSD) or a custom model.
    *   Convert this model to an ONNX format, and then to a TensorRT engine optimized for your Jetson (as described in Section 2.3).
    *   Launch a ROS 2 pipeline using `isaac_ros_dnn_image_encoder` and `isaac_ros_tensor_rt` nodes to perform inference on your camera's RGB stream.
    *   Implement a simple post-processing ROS 2 node (if needed) to parse the raw TensorRT output into a meaningful ROS 2 message (e.g., `vision_msgs/msg/Detection2DArray`).
5.  **RViz2 Visualization**:
    *   On a display connected to your Jetson (or a workstation connected via network), launch RViz2.
    *   Display the VSLAM output (robot pose, map points/trajectory).
    *   Display the DNN inference results (e.g., bounding boxes overlayed on the camera image, or markers for detected objects).

### Verification Steps

To verify the successful completion of this project:

1.  **Launch All Nodes**: Start your camera driver (if real), VSLAM, DNN inference pipeline, and RViz2.
2.  **Move Camera/Robot**: Slowly move your Jetson (or the robot to which the camera is attached) in an environment with sufficient visual features.
3.  **Inspect RViz2 Output**:
    *   **VSLAM**: Observe a continuous and stable trajectory for the robot's pose. The point cloud display should show a consistent and growing map of the environment. The map should not drift significantly.
    *   **DNN Inference**: Observe bounding boxes (or other visual indicators) appearing around detected objects in the camera feed overlay or separate display in RViz2. Confirm that detections are happening in real-time and with reasonable accuracy for your chosen model.
    *   **Performance**: Monitor the frame rates and latency of the perception pipeline. The hardware acceleration should enable near real-time performance.

By completing this project, you will have a fully functional and hardware-accelerated perception pipeline on your Jetson, capable of both simultaneous localization and mapping and advanced object recognition.

## Visual Aids & Diagrams

To enhance understanding and engagement in Chapter 2, the following visual aids and diagrams are recommended:

*   **Figure 2.1: Visual SLAM vs. Wheel Odometry Comparison**:
    *   **Description**: A diagram illustrating the concept of drift in wheel odometry (e.g., a straight line becoming curved over distance) compared to Visual SLAM's ability to correct drift using visual landmarks. Show a robot trajectory with and without VSLAM.
    *   **Placement Hint**: After Section 2.2.1, "Visual SLAM vs. Wheel Odometry".

*   **Figure 2.2: Isaac ROS VSLAM Pipeline**:
    *   **Description**: A system architecture diagram showing the flow of data through the `isaac_ros_vslam` node. Inputs (RGB-D camera topics, camera info), the VSLAM node, and outputs (pose, map points, odometry) should be clearly depicted, highlighting hardware acceleration.
    *   **Placement Hint**: After Section 2.2.2, "Isaac ROS VSLAM Module".

*   **Figure 2.3: RViz2 VSLAM Visualization Screenshot**:
    *   **Description**: A screenshot of RViz2 displaying the output of `isaac_ros_vslam`. This should include the robot's estimated trajectory (path), a sparse point cloud map, and the camera's current pose.
    *   **Placement Hint**: After Section 2.2.2, "Visualize VSLAM Output in RViz2".

*   **Figure 2.4: TensorRT Optimization Process**:
    *   **Description**: A flowchart illustrating the typical model conversion process: Trained Model (PyTorch/TensorFlow) -> ONNX Export -> TensorRT Optimizer -> TensorRT Engine. Highlight the benefits like layer fusion and precision calibration.
    *   **Placement Hint**: After Section 2.3.1, "The Need for TensorRT".

*   **Figure 2.5: Isaac ROS DNN Inference Pipeline**:
    *   **Description**: A system architecture diagram showing the ROS 2 pipeline for hardware-accelerated DNN inference: Camera -> `isaac_ros_dnn_image_encoder` -> `isaac_ros_tensor_rt` (with TensorRT engine) -> (Optional) Post-processing Node -> Output Detections (e.g., bounding boxes).
    *   **Placement Hint**: After Section 2.3.3, "Isaac ROS for DNN Deployment".

*   **Figure 2.6: RViz2 DNN Inference Visualization Screenshot**:
    *   **Description**: A screenshot of RViz2 displaying the output of the DNN inference pipeline, with bounding boxes (or other visual overlays) clearly shown on a camera image stream.
    *   **Placement Hint**: After Section 2.4, "Chapter 2 Project/Checkpoint: VSLAM and DNN Inference", as part of the verification.

These visual aids will help students grasp the complex data flows and visualizations involved in hardware-accelerated perception.

## Troubleshooting & Debugging Guide

This section addresses common issues encountered when implementing Visual SLAM and DNN inference with Isaac ROS on Jetson platforms.

*   **1. Isaac ROS Node Launch Failures or Immediate Exits**:
    *   **Problem**: `ros2 launch` commands for Isaac ROS nodes fail, or the nodes start and immediately crash.
    *   **Possible Causes**: Incorrect JetPack version, missing dependencies, issues with NVIDIA container runtime, or insufficient memory.
    *   **Solution**:
        *   **Verify JetPack Version**: Ensure your Jetson's JetPack SDK version is compatible with the Isaac ROS version you are using. Refer to the Isaac ROS documentation for compatibility matrices.
        *   **Check Docker Container Environment**: Confirm you are running within the `isaac_ros_dev` Docker container or a similar environment with all necessary dependencies sourced.
        *   **Review Logs**: Use `ros2 launch --debug` or check the node's output for specific error messages. Look for `segmentation fault` or `CUDA error` messages, which often point to GPU/memory issues.
        *   **Memory Management**: Jetson devices have unified memory. Ensure no other applications are consuming too much RAM or GPU memory.

*   **2. VSLAM Loses Tracking or Produces Drifting Maps**:
    *   **Problem**: The VSLAM pipeline fails to track the camera's pose, resulting in a stationary robot in RViz2, or the generated map gradually shifts and distorts.
    *   **Possible Causes**: Poor texture in the environment, rapid camera motion, insufficient feature points, incorrect camera calibration, or inadequate computing resources.
    *   **Solution**:
        *   **Environment Texture**: Operate in environments with rich visual texture. Featureless walls or highly repetitive patterns can challenge VSLAM.
        *   **Camera Motion**: Maintain smooth camera movements. Avoid very rapid rotations or translations if not configured for high-speed motion.
        *   **Camera Calibration**: Ensure your camera is accurately calibrated, especially its intrinsic parameters (focal length, principal point, distortion coefficients). Incorrect calibration severely impacts VSLAM.
        *   **VSLAM Parameters**: Tune VSLAM parameters (e.g., feature detector settings, keyframe thresholds) in its YAML configuration file to suit your environment and camera.
        *   **Loop Closure**: For long-term mapping, ensure loop closure is active and effective to correct accumulated drift when the robot revisits a previously mapped area.

*   **3. DNN Inference is Too Slow on Jetson (Low Frame Rate)**:
    *   **Problem**: Object detection or other DNN tasks are not running at real-time frame rates, or the latency is too high.
    *   **Possible Causes**: Model not optimized with TensorRT, incorrect TensorRT engine precision, competition for GPU resources, or inefficient ROS 2 data transfer.
    *   **Solution**:
        *   **TensorRT Optimization**: **Crucial step.** Ensure your DNN model is converted to a TensorRT engine using `trtexec` or the Python API, specifically targeting FP16 or INT8 precision if possible (refer to Section 2.3).
        *   **Profile Performance**: Use NVIDIA's profiling tools (e.g., Nsight Systems, `nvprof`) to identify bottlenecks in your pipeline.
        *   **Jetson Power Modes**: Set your Jetson to its maximum power mode (`sudo jetson_clocks` or use `nvpmodel`) to ensure full performance.
        *   **Batching**: If your model supports it, process multiple frames in a batch to improve GPU utilization.
        *   **Zero-Copy ROS 2 Transport**: Leverage `isaac_ros_image_proc` and `isaac_ros_common` utilities that use NVIDIA's `RCL_CUDA_RMW_PROVIDER` for zero-copy data transfer between ROS 2 nodes, reducing CPU overhead and latency.
        *   **Model Complexity**: If all optimizations are applied and performance is still an issue, consider using a lighter-weight DNN model.

These troubleshooting tips should help you debug common issues and optimize the performance of your Isaac ROS-based perception pipelines on the Jetson Orin Nano/NX.