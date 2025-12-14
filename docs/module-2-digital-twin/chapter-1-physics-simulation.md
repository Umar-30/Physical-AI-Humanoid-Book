# Chapter 1: Foundations of Physics Simulation (Gazebo)

This chapter introduces the fundamentals of robotics simulation using Gazebo, focusing on physics and robot modeling.

## 1.1 Introduction to Gazebo & ROS 2 Physics Simulation

Welcome to the foundational chapter on using Gazebo for robotics simulation within the ROS 2 ecosystem. Gazebo is a powerful 3D robotics simulator capable of accurately simulating populations of robots, sensors, and objects in a high-fidelity environment. It's an indispensable tool for robotics research and development, allowing you to test algorithms, design robots, and perform training without the need for expensive or time-consuming physical hardware.

### Why Gazebo?

Gazebo's strength lies in its **physics engine**, which provides realistic simulation of:
*   **Rigid body dynamics**: Simulating the movement and interaction of solid objects.
*   **Collision detection**: Preventing objects from passing through each other unrealistically.
*   **Gravity**: Applying gravitational forces to all objects in the world.
*   **Joints and actuators**: Modeling how robot parts connect and move.
*   **Sensors**: Emulating data from cameras, LiDARs, IMUs, and more.

This physics-first approach contrasts with other simulators that might prioritize graphical fidelity (like Unity, which we'll explore later). Gazebo focuses on the accurate physical behavior crucial for robotics.

### Gazebo and ROS 2: A Synergistic Relationship

ROS 2 (Robot Operating System 2) provides the communication backbone for your robot's software. When combined with Gazebo, ROS 2 allows your simulated robot to interact with its virtual environment and other ROS 2 components (like navigation stacks, perception algorithms, and control systems) exactly as a real robot would.

Key aspects of this synergy include:
*   **ROS 2 Nodes**: Your robot's behaviors, sensors, and actuators in Gazebo can be represented as ROS 2 nodes, communicating via topics, services, and actions.
*   **Message Passing**: Gazebo publishes sensor data (e.g., camera images, LiDAR scans) to ROS 2 topics, and ROS 2 publishes control commands (e.g., motor velocities) to Gazebo.
*   **Configuration**: ROS 2 launch files are commonly used to start Gazebo worlds and spawn robots, streamlining the simulation setup.

In this chapter, you will learn how to set up your first Gazebo simulation, define simple robot models using URDF, and control them within the ROS 2 framework.

## 1.2 Basic URDF Modeling for Gazebo

The Unified Robot Description Format (URDF) is an XML format used in ROS to describe all elements of a robot. This includes its physical properties (mass, inertia), visual characteristics (color, shape), and kinematic structure (joints and links). While URDF is great for describing the robot's kinematics and visuals, for full Gazebo simulation, we often augment it with Gazebo-specific tags to define physics properties, sensors, and plugins.

### Key URDF Concepts

*   **Link**: A rigid body of the robot. Each link has physical and visual properties.
*   **Joint**: Connects two links, defining their relative motion (e.g., revolute, prismatic, fixed).
*   **Origin**: Defines the position and orientation of a child frame relative to a parent frame.
*   **Inertial**: Defines the mass and inertia tensor of a link, crucial for physics simulation.
*   **Visual**: Describes the graphical representation of a link.
*   **Collision**: Describes the collision geometry of a link, used by the physics engine.

### Creating a Simple Robot: A Single Box

Let's create a minimal URDF for a simple floating box.

1.  **Create a New ROS 2 Package for Your Robot**:
    First, ensure you are in your ROS 2 workspace (e.g., `~/ros2_ws/src`) and create a new package for your robot's description.

    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_cmake my_robot_description
    ```

2.  **Define the URDF File**:
    Inside your new package, create a `urdf` directory: `mkdir -p ~/ros2_ws/src/my_robot_description/urdf`.
    Then, create `box_robot.urdf` inside `~/ros2_ws/src/my_robot_description/urdf` with the following content:

    ```xml
    <?xml version="1.0"?>
    <robot name="box_robot">

      <link name="base_link">
        <visual>
          <geometry>
            <box size="0.5 0.5 0.5"/>
          </geometry>
          <material name="red">
            <color rgba="1 0 0 1"/>
          </material>
        </visual>
        <collision>
          <geometry>
            <box size="0.5 0.5 0.5"/>
          </geometry>
        </collision>
        <inertial>
          <mass value="1.0"/>
          <inertia ixx="0.083" ixy="0.0" ixz="0.0" iyy="0.083" iyz="0.0" izz="0.083"/>
        </inertial>
      </link>

      <!-- Gazebo specific reference to enable physics -->
      <gazebo reference="base_link">
        <material>Gazebo/Red</material>
        <mu1>0.2</mu1>
        <mu2>0.2</mu2>
      </gazebo>

    </robot>
    ```

    *   **Explanation**:
        *   `<robot name="box_robot">`: Defines the root of the robot description.
        *   `<link name="base_link">`: Our single rigid body.
        *   `<visual>`: Defines how the link looks (a 0.5m red box).
        *   `<collision>`: Defines the shape used for physics interactions (also a 0.5m box). It's crucial that collision geometry is simple and accurate.
        *   `<inertial>`: Specifies mass and inertia. Without `inertial` tags, Gazebo will ignore the physics of the object.
        *   `<gazebo reference="base_link">`: This is a Gazebo-specific extension. It tells Gazebo to apply special properties to `base_link`, such as its material appearance in Gazebo's renderer (`Gazebo/Red`) and friction coefficients (`mu1`, `mu2`).

### Checking Your URDF

You can check your URDF for syntax errors and visualize it using the `check_urdf` and `rviz2` tools.

1.  **Build your package**:

    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_robot_description
    source install/setup.bash
    ```

2.  **Check URDF syntax**:

    ```bash
    check_urdf ~/ros2_ws/src/my_robot_description/urdf/box_robot.urdf
    ```

    You should see output indicating parsing success.

3.  **Visualize in RViz2**:

    ```bash
    rviz2 -d $(ros2 pkg prefix my_robot_description)/share/my_robot_description/rviz/urdf.rviz
    ```

    You'll need a simple RViz configuration file. Create `~/ros2_ws/src/my_robot_description/rviz/urdf.rviz` with the following content:

    ```xml
    <?xml version="1.0" ?>
    <rviz>
      <display>
        <type>rviz_default_plugins/Displays/RobotModel</type>
        <enabled>true</enabled>
        <robot_description>robot_description</robot_description>
        <tf_prefix></tf_prefix>
        <update_rate>30</update_rate>
        <visual_enabled>true</visual_enabled>
      </display>
      <frame_id>base_link</frame_id>
      <global_options>
        <fixed_frame>base_link</fixed_options>
      </global_options>
    </rviz>
    ```
    Then run:
    ```bash
    ros2 launch my_robot_description display_box_robot.launch.py
    ```
    You'll need a launch file. Create `~/ros2_ws/src/my_robot_description/launch/display_box_robot.launch.py` (create `launch` directory first):
    ```python
    import os
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration, Command
    from launch_ros.actions import Node

    def generate_launch_description():
        # Get the URDF file path
        urdf_file_name = 'box_robot.urdf'
        urdf_path = os.path.join(
            get_package_share_directory('my_robot_description'),
            'urdf',
            urdf_file_name
        )

        # RViz config file
        rviz_config_dir = os.path.join(
            get_package_share_directory('my_robot_description'),
            'rviz',
            'urdf.rviz'
        )

        return LaunchDescription([
            DeclareLaunchArgument(
                name='urdf_model',
                default_value=urdf_path,
                description='URDF path'
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('urdf_model')])}]
            ),
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
                output='screen'
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_dir]
            )
        ])
    ```
    Now, rebuild and try the launch file:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_robot_description
    source install/setup.bash
    ros2 launch my_robot_description display_box_robot.launch.py
    ```
    You should see a red box in RViz2. Use the `joint_state_publisher_gui` to move any joints (though our box has none). This confirms your URDF is correctly parsed.

### Troubleshooting Tips: URDF

*   **Robot not appearing in RViz2**:
    *   **Check `robot_description` parameter**: Ensure the `robot_state_publisher` node is correctly publishing the URDF string under the `robot_description` parameter. Use `ros2 param get /robot_state_publisher robot_description`.
    *   **`fixed_frame` in RViz2**: Make sure your RViz2 configuration's `fixed_frame` (usually `base_link` or `odom`) matches a link in your URDF.
    *   **URDF path**: Double-check that the URDF file path in your launch file is correct and accessible.
*   **Gazebo ignoring physics**:
    *   **Missing `<inertial>` tags**: Ensure every link you want to have physics has proper `<inertial>` tags defining its mass and inertia.
    *   **Incorrect collision geometry**: Collision geometry should be defined and accurately represent the physical shape. Complex visuals should have simpler collision primitives.
*   **`check_urdf` errors**: XML parsing errors are common. Ensure all tags are correctly closed, attributes are spelled correctly, and the XML is well-formed.

## 1.3 Launching & Interacting with Gazebo Simulations

Now that you understand URDF, let's learn how to spawn your robot in Gazebo and interact with it using ROS 2.

### Spawning a URDF Robot in Gazebo

To get your URDF robot into Gazebo, you typically use a ROS 2 launch file that:
1.  Starts the Gazebo simulator.
2.  Uses the `ros_gz_spawn_entity` utility to load your URDF model into the Gazebo world.
3.  Starts a `robot_state_publisher` to publish the robot's state to ROS 2.

Let's modify our `my_robot_description` package to include a launch file for Gazebo.

1.  **Create a Gazebo World File (Optional but Recommended)**:
    While you can spawn robots into an empty world, it's good practice to define your own. Create `~/ros2_ws/src/my_robot_description/worlds/empty.sdf` (create `worlds` directory if it doesn't exist):

    ```xml
    <?xml version="1.0" ?>
    <sdf version="1.6">
      <world name="empty_world">
        <include>
          <uri>https://fuel.ignitionrobotics.org/1.0/openrobotics/models/sun</uri>
        </include>
        <include>
          <uri>https://fuel.ignitionrobotics.org/1.0/openrobotics/models/ground_plane</uri>
        </include>
      </world>
    </sdf>
    ```

2.  **Create a Gazebo Launch File**:
    Create `~/ros2_ws/src/my_robot_description/launch/spawn_box_robot.launch.py`:

    ```python
    import os
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import LaunchConfiguration, Command
    from launch_ros.actions import Node

    def generate_launch_description():
        # Get paths
        pkg_share_dir = get_package_share_directory('my_robot_description')
        urdf_path = os.path.join(pkg_share_dir, 'urdf', 'box_robot.urdf')
        world_path = os.path.join(pkg_share_dir, 'worlds', 'empty.sdf')

        # Launch arguments
        use_sim_time = LaunchConfiguration('use_sim_time', default='true')
        
        # Robot State Publisher Node
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 
                         'robot_description': Command(['xacro ', urdf_path])}],
            arguments=[urdf_path]
        )

        # Gazebo Launch
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            )]),
            launch_arguments={'gz_args': ['-r -s ', world_path]}.items()
        )

        # Spawn Robot Node
        spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=['-topic', '/robot_description',
                       '-entity', 'box_robot',
                       '-x', '0', '-y', '0', '-z', '1.0']
        )

        return LaunchDescription([
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use simulation (Gazebo) clock if true'
            ),
            gazebo_launch,
            robot_state_publisher_node,
            spawn_entity,
        ])
    ```
    *   **Explanation**:
        *   `gazebo_launch`: Uses `ros_gz_sim`'s launch file to start Gazebo with our `empty.sdf` world. The `-r` argument means "run" and `-s` means "server" (no GUI initially, though it opens).
        *   `robot_state_publisher_node`: Reads the `box_robot.urdf` and publishes the robot's link transforms to ROS 2's TF (Transform Frame) tree.
        *   `spawn_entity`: Uses `ros_gz_sim`'s `create` executable to spawn our `box_robot` entity into Gazebo at coordinates `(0, 0, 1.0)`.

3.  **Build and Launch**:
    Ensure your `~/ros2_ws` is sourced after building.

    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_robot_description
    source install/setup.bash
    ros2 launch my_robot_description spawn_box_robot.launch.py
    ```

    You should see Gazebo launch with a red box dropping and coming to rest on the ground plane. You can use the Gazebo GUI to move the camera around and inspect the box.

### Basic Interaction: Moving the Box (Not Directly in Gazebo)

For a simple static box, direct "movement" in Gazebo usually comes from applying forces or changing its pose via Gazebo commands. However, the standard ROS 2 way to interact with a robot is through its joint controllers or by publishing commands to its base. Since our box has no joints or actuators defined yet, we can't "move" it in the typical sense.

For now, the interaction is primarily visual: observing the physics simulation. In later chapters, we will add joints and controllers to enable dynamic interaction.

### Troubleshooting Tips: Gazebo Launch

*   **Gazebo not launching or showing errors**:
    *   **Check `ros_gz_sim` installation**: Ensure you have `ros-humble-ros-gz-sim` installed (`sudo apt install ros-humble-ros-gz-sim`).
    *   **Source environment**: Always ensure your ROS 2 and Ignition Gazebo environments are sourced. `source /opt/ros/humble/setup.bash` and `source /usr/share/ignition/ignition-fortress/setup.bash`.
    *   **Launch file syntax**: Python launch files are sensitive to indentation. Double-check your `spawn_box_robot.launch.py` for errors.
*   **Robot not appearing in Gazebo**:
    *   **Spawn entity parameters**: Verify the `-topic` and `-entity` arguments in `ros_gz_sim create` match your `robot_description` topic and the `<robot name="...">` in your URDF.
    *   **Z-coordinate**: Ensure the initial `-z` coordinate is high enough for the robot to drop. If it spawns below the ground, it might be stuck or immediately disappear.
*   **"Waiting for /robot_description to be published..."**:
    *   This indicates `robot_state_publisher` is not running or not publishing. Check the node is launched and that your URDF is valid (use `check_urdf`).

## Troubleshooting & Debugging Guide for Chapter 1

This section covers common issues that might arise during the initial setup and simulation with Gazebo and ROS 2, beyond the specific URDF or Gazebo launch problems discussed above.

*   **1. Slow Simulation or Low Frame Rates in Gazebo**:
    *   **Problem**: Gazebo simulation runs very slowly, or the graphical interface (GUI) has a low frame rate, even with simple worlds/models.
    *   **Possible Causes**: Insufficient GPU resources, excessive collision checks, high sensor update rates, too many complex models, or issues with X server/display setup in Docker environments.
    *   **Solution**:
        *   **Reduce Simulation Speed**: In Gazebo GUI, go to `World` -> `Time` -> `Real Time Factor` and set it to a lower value (e.g., 0.5 or 0.1).
        *   **Simplify Models**: Use simpler collision geometries than visual geometries if possible. Remove unnecessary details from models.
        *   **Lower Sensor Update Rates**: If you have sensors, reduce their update frequencies in your URDF/SDF files.
        *   **Check GPU Drivers**: Ensure NVIDIA GPU drivers are correctly installed and updated on your host machine.
        *   **X Server Configuration (Docker)**: Verify `xhost +` is correctly set up before launching Docker containers that use the GUI, and ensure display environment variables (`DISPLAY`) are correct.
        *   **Run Gazebo headless**: For purely physics-based simulations without visual interaction, launch Gazebo in headless mode (server only) to save GPU resources.

*   **2. Coordinate Frame (TF) Mismatch or "Frame Does Not Exist" Errors**:
    *   **Problem**: ROS 2 nodes (e.g., RViz2) complain about missing or unconnected TF frames, or your robot appears at an unexpected location/orientation.
    *   **Possible Causes**: `robot_state_publisher` not running, incorrect `fixed_frame` in RViz2, missing transforms in your URDF, or issues with world-to-base_link transform.
    *   **Solution**:
        *   **Verify `robot_state_publisher`**: Ensure the `robot_state_publisher` node is running (`ros2 node list`) and publishing the `robot_description` parameter.
        *   **Check `ros2 run tf2_tools view_frames`**: This command generates a PDF visualizing your TF tree, which can quickly highlight missing links or unconnected branches.
        *   **RViz2 `fixed_frame`**: Confirm your RViz2 display's `Fixed Frame` setting matches the parent frame of your robot (e.g., `base_link`, `odom`, or `world`).
        *   **URDF Consistency**: Double-check that all `link` and `joint` names in your URDF are consistent and correctly connected.

*   **3. ROS 2 - Gazebo Communication Failure (Topics/Services)**:
    *   **Problem**: ROS 2 nodes are not receiving data from Gazebo topics (e.g., sensor data), or control commands sent from ROS 2 are not affecting the robot in Gazebo.
    *   **Possible Causes**: Mismatch in topic names, incorrect ROS 2-Gazebo bridge configuration, `ros_gz_bridge` not running, or issues with `use_sim_time`.
    *   **Solution**:
        *   **Inspect Topics**: Use `ros2 topic list`, `ros2 topic info <topic_name>`, and `ros2 topic echo <topic_name>` to verify that topics are being published by Gazebo and subscribed to by your ROS 2 nodes with the expected message types.
        *   **Check `ros_gz_bridge`**: Ensure the `ros_gz_bridge` nodes are correctly launched and configured to bridge the necessary topics between ROS 2 and Gazebo.
        *   **Verify `use_sim_time`**: Both Gazebo and all relevant ROS 2 nodes (especially `robot_state_publisher` and your control nodes) should have `use_sim_time` set consistently (usually `true` when simulating). Check launch file parameters and node configurations.
        *   **Gazebo Plugins**: Confirm that your URDF/SDF model includes the necessary Gazebo plugins (e.g., `libignition-ros2-control-system.so`, `libignition-gazebo-ros2-diff-drive-plugin.so`) to enable ROS 2 communication for joints, sensors, or other robot components.