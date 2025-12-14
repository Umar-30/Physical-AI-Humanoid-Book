# Unity Engine Installation and ROS-Unity Integration

This section guides you through setting up Unity for robotics development and integrating it with ROS 2.

## 1. Install Unity Hub and Unity Editor

Unity Hub is a management tool that helps you manage multiple versions of Unity Editor and your Unity projects.

1.  **Download Unity Hub**: Go to the [Unity Download Archive](https://unity3d.com/get-unity/download/archive) and download Unity Hub for Linux.
    *   Alternatively, you can install it via Snap: `sudo snap install unityhub --classic`
2.  **Install Unity Editor**: After installing Unity Hub, open it.
    *   Go to the "Installs" tab.
    *   Click "Install Editor".
    *   Select a Unity Editor version (e.g., 2022.3 LTS recommended for stability). Ensure you include the **Linux Build Support (Mono)** module during installation.
    *   For this module, we will assume you have **Unity 2022.3 LTS** installed.

## 2. Install ROS-Unity Integration Package

The ROS-Unity Integration package (also known as Unity Robotics ROS-TCP-Endpoint) allows communication between ROS 2 and Unity.

1.  **Create a New Unity Project**:
    *   Open Unity Hub, click "New Project".
    *   Select a 3D Core template.
    *   Name your project (e.g., `ROS2UnityDigitalTwin`) and choose a location.
    *   Click "Create Project".
2.  **Install ROS-TCP-Endpoint via Unity Package Manager**:
    *   Once your Unity project is open, go to `Window > Package Manager`.
    *   Click the `+` icon in the top-left corner and select "Add package from git URL...".
    *   Enter the URL: `https://github.com/Unity-Technologies/Unity-Robotics-Hub.git?path=/com.unity.robotics.ros-tcp-endpoint`
    *   Click "Add". This will install the necessary communication components.
3.  **Install ROS-TCP-Connector via Unity Package Manager (Optional but Recommended)**:
    *   This package provides scripts for handling ROS message serialization/deserialization.
    *   In the Package Manager, click `+` and enter: `https://github.com/Unity-Technologies/Unity-Robotics-Hub.git?path=/com.unity.robotics.ros-tcp-connector`
    *   Click "Add".
4.  **Install ROS 2 Messages**:
    *   The `com.unity.robotics.ros-messages` package is also crucial. It needs to be generated from your ROS 2 environment.
    *   **In your ROS 2 environment (Ubuntu terminal)**:
        *   Navigate to your ROS 2 workspace (e.g., `~/ros2_ws`).
        *   Make sure `ros-humble-ros-tcp-endpoint` (or similar) is installed: `sudo apt install ros-humble-ros-tcp-endpoint`
        *   Follow the instructions from Unity Robotics Hub to generate custom ROS messages. Typically, this involves using the `ros2 run ros_tcp_endpoint generate_msgs.py` command or similar, after installing the `ros-tcp-endpoint` ROS package. The generated C# message files will then be placed into your Unity project.

    *   **Note**: Detailed steps for generating C# messages from your custom ROS 2 messages will be covered as needed in later chapters. For now, ensure the `ros-tcp-endpoint` Unity package is installed.

## 3. Basic Connectivity Test

To verify Unity's ability to connect to ROS 2, you can use the provided example scenes in the Unity Robotics Hub.

1.  **In Unity**: Navigate to `Assets/Samples/Unity Robotics ROS TCP Endpoint/X.X.X/ROS-TCP-Endpoint-Samples/SimpleSubscriber`. Open the `SimpleSubscriber` scene.
2.  **In ROS 2 (Ubuntu terminal)**:
    *   Source your ROS 2 environment: `source /opt/ros/humble/setup.bash`
    *   Run the simple publisher: `ros2 run ros_tcp_endpoint simple_publisher`
3.  **In Unity**: Press Play in the Editor. You should see messages appearing in the Unity console, indicating successful communication.
