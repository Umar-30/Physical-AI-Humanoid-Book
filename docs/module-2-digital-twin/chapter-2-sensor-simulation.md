    To quickly visualize in RViz2:
    ```bash
    rviz2
    ```
    In RViz2:
    *   Set `Fixed Frame` to `lidar_link` (or `base_link` if you prefer).
    *   Add a `LaserScan` display.
    *   Set `Topic` to `/my_robot/scan`.
    *   You should see the laser scan points appear as your box robot interacts with the environment. Add an obstacle in Gazebo to see the rays.

## 2.2 ROS 2 Interfaces for Sensor Data

Once sensors are integrated into your robot model within Gazebo, the next crucial step is to access and process their data using ROS 2. This involves understanding how Gazebo-ROS 2 bridges work and writing simple ROS 2 Python nodes to subscribe to sensor topics.

### 2.2.1 Gazebo-ROS 2 Bridging

Gazebo itself does not natively publish data to ROS 2 topics. A dedicated bridge is required. For Ignition Gazebo (Gazebo Fortress/Harmonic/etc.), the `ros_gz_bridge` package (part of `ros_gz` suite) handles this.

*   **How it works**: The `ros_gz_bridge` creates ROS 2 nodes that subscribe to Gazebo's internal topics (e.g., `/camera/image`, `/lidar`) and republish them as ROS 2 topics (e.g., `/ros_camera_topic`, `/ros_lidar_topic`), or vice-versa for control commands.
*   **Automatic Bridging via URDF Plugins**: As seen in Section 2.1, the `<plugin>` tags within the `<gazebo>` element of your URDF/SDF automatically configure some of these bridges (e.g., `libignition-gazebo-ros2-camera-bridge-plugin.so`, `libignition-gazebo-ros2-laser-bridge-plugin.so`). These plugins effectively create the necessary ROS 2 interface for the sensor.

### 2.2.2 Subscribing to Camera Data in ROS 2 (Python Node)

Let's create a simple ROS 2 Python node that subscribes to the camera image topic and saves images to disk.

1.  **Create a ROS 2 Package**:
    If you don't already have a ROS 2 workspace (`~/ros2_ws`), create one:
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_python my_sensor_reader --dependencies rclpy sensor_msgs cv_bridge
    ```
    *   `rclpy`: ROS Client Library for Python.
    *   `sensor_msgs`: ROS messages for sensor data (e.g., `Image`).
    *   `cv_bridge`: A library to convert between ROS `sensor_msgs/Image` and OpenCV image formats.

2.  **Add `package.xml` dependencies**:
    Ensure `cv_bridge` is listed as a dependency in `package.xml`.
    ```xml
    # in ~/ros2_ws/src/my_sensor_reader/package.xml
    <depend>cv_bridge</depend>
    <depend>sensor_msgs</depend>
    ```

3.  **Create the Image Subscriber Node**:
    Create a Python file `image_subscriber.py` in `~/ros2_ws/src/my_sensor_reader/my_sensor_reader/`:
    ```python
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2
    import os

    class ImageSubscriber(Node):
        def __init__(self):
            super().__init__('image_subscriber')
            self.subscription = self.create_subscription(
                Image,
                '/my_robot/camera',  # Replace with your camera topic from URDF
                self.listener_callback,
                10)
            self.subscription  # prevent unused variable warning
            self.br = CvBridge()
            self.get_logger().info("Image subscriber node started.")
            self.image_count = 0
            self.output_dir = os.path.expanduser('~/ros2_ws/saved_images') # Define output directory
            os.makedirs(self.output_dir, exist_ok=True) # Create directory if it doesn't exist

        def listener_callback(self, data):
            self.get_logger().info('Receiving video frame')
            current_frame = self.br.imgmsg_to_cv2(data, "bgr8")
            
            # Save the image
            image_filename = os.path.join(self.output_dir, f'image_{self.image_count:04d}.png')
            cv2.imwrite(image_filename, current_frame)
            self.image_count += 1
            self.get_logger().info(f'Saved {image_filename}')

            # You can also display the image (optional, might require X server forwarding)
            # cv2.imshow("camera_feed", current_frame)
            # cv2.waitKey(1)

    def main(args=None):
        rclpy.init(args=args)
        image_subscriber = ImageSubscriber()
        rclpy.spin(image_subscriber)
        image_subscriber.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

4.  **Add Entry Point in `setup.py`**:
    Edit `~/ros2_ws/src/my_sensor_reader/setup.py` and add the following entry point:
    ```python
    # ...
    entry_points={
        'console_scripts': [
            'image_subscriber = my_sensor_reader.image_subscriber:main',
        ],
    },
    # ...
    ```

5.  **Build and Run**:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_sensor_reader
    source install/setup.bash

    # Launch your Gazebo simulation (from Module 1) first
    ros2 launch my_robot_description spawn_box_robot.launch.py

    # In a new terminal, run your image subscriber
    ros2 run my_sensor_reader image_subscriber
    ```
    You should see messages indicating image frames are being received and saved. Check `~/ros2_ws/saved_images` for the saved `.png` files.

### 2.2.3 Subscribing to LiDAR Data in ROS 2 (Python Node)

Similarly, you can subscribe to LiDAR data (published as `sensor_msgs/msg/LaserScan`).

1.  **Create a LiDAR Subscriber Node**:
    Create a Python file `lidar_subscriber.py` in `~/ros2_ws/src/my_sensor_reader/my_sensor_reader/`:
    ```python
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan

    class LidarSubscriber(Node):
        def __init__(self):
            super().__init__('lidar_subscriber')
            self.subscription = self.create_subscription(
                LaserScan,
                '/my_robot/scan',  # Replace with your LiDAR topic from URDF
                self.listener_callback,
                10)
            self.subscription  # prevent unused variable warning
            self.get_logger().info("LiDAR subscriber node started.")

        def listener_callback(self, msg):
            # Process LaserScan data
            # msg.ranges is a list of distances (in meters) for each ray
            # msg.angle_min, msg.angle_max, msg.angle_increment give angular info
            
            # For demonstration, we'll just log the number of valid readings and min/max range
            valid_ranges = [r for r in msg.ranges if r > msg.range_min and r < msg.range_max]
            
            if valid_ranges:
                self.get_logger().info(
                    f'Received {len(valid_ranges)} valid LiDAR readings. '
                    f'Min: {min(valid_ranges):.2f}m, Max: {max(valid_ranges):.2f}m'
                )
            else:
                self.get_logger().info('Received LiDAR data with no valid ranges.')

    def main(args=None):
        rclpy.init(args=args)
        lidar_subscriber = LidarSubscriber()
        rclpy.spin(lidar_subscriber)
        lidar_subscriber.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

2.  **Add Entry Point in `setup.py`**:
    Edit `~/ros2_ws/src/my_sensor_reader/setup.py` and add the new entry point:
    ```python
    # ...
    entry_points={
        'console_scripts': [
            'image_subscriber = my_sensor_reader.image_subscriber:main',
            'lidar_subscriber = my_sensor_reader.lidar_subscriber:main', # New entry
        ],
    },
    # ...
    ```

3.  **Build and Run**:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_sensor_reader
    source install/setup.bash

    # Launch your Gazebo simulation (from Module 1) first
    ros2 launch my_robot_description spawn_box_robot.launch.py

    # In a new terminal, run your LiDAR subscriber
    ros2 run my_sensor_reader lidar_subscriber
    ```
    You should see messages indicating LiDAR data is being received and processed.

By creating these basic ROS 2 subscriber nodes, you've established the fundamental interface between your simulated Gazebo sensors and your robot's ROS 2 software stack, ready for more complex perception tasks.