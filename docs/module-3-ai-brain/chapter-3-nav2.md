# Chapter 3: Path Planning for Bipedal Navigation (Nav2)

This chapter focuses on configuring and utilizing ROS 2 Nav2 for bipedal robot navigation, understanding its components for global and local path planning and execution.

## 3.1 Introduction to Nav2 Stack for Bipedal Robots

The ROS 2 Navigation Stack (Nav2) is a powerful framework for enabling autonomous navigation in mobile robots. While primarily designed for wheeled robots, its modular architecture makes it adaptable for various platforms, including bipedal robots. This section introduces the core components of Nav2 and discusses the unique challenges and considerations for bipedal navigation.

### 3.1.1 Overview of the Nav2 Stack

Nav2 is a collection of ROS 2 packages that provide a complete solution for autonomous navigation. Its key components include:
*   **Map Server**: Provides maps of the environment (e.g., from SLAM, or pre-built).
*   **AMCL (Adaptive Monte Carlo Localization)**: Localizes the robot within a known map.
*   **Global Planner**: Plans a collision-free path from the robot's current pose to a goal pose on a global costmap.
*   **Local Planner (Controller)**: Plans a local path and generates velocity commands to follow the global path and avoid immediate obstacles on a local costmap.
*   **Behavior Tree**: Orchestrates the navigation process, handling tasks like goal following, recovery behaviors, and obstacle avoidance.
*   **Costmaps**: 2D occupancy grids representing the environment for planning, with inflated obstacles to ensure clearance. There's usually a global costmap for long-range planning and a local costmap for immediate obstacle avoidance.

### 3.1.2 Challenges of Bipedal Navigation

Adapting Nav2 for bipedal robots presents several unique challenges compared to wheeled platforms:
*   **Kinematic Constraints**: Bipedal robots have complex, non-holonomic kinematics. Their motion is often slower, more complex, and less agile than wheeled robots.
*   **Balance and Stability**: Maintaining balance is paramount. Navigation commands must be filtered or generated to ensure the robot remains stable, especially during turns or over uneven terrain.
*   **Footstep Planning**: Unlike continuous velocity commands for wheels, bipedal navigation often requires discrete footstep planning, which is not directly handled by standard Nav2 local planners. This may require custom plugins or wrappers.
*   **Limited Field of View**: Bipedal robots typically rely on cameras at "head" height, which can miss obstacles close to the ground.
*   **Perception of "Walkable" Surface**: Identifying areas where the robot can safely step is more complex than simple obstacle detection.

### 3.1.3 Adapting Nav2 for Bipedal Robots

To use Nav2 with bipedal robots, a common approach involves:
*   **Custom Local Planner/Controller**: Replacing or extending the default Nav2 local planners (e.g., DWB, TEB) with a custom controller that translates Nav2's desired velocities into bipedal gait patterns and maintains balance. This is the most significant adaptation.
*   **Motion Primitives**: Pre-defined or learned walking gaits that can execute specific movements (e.g., turn in place, step forward/backward).
*   **Terrain Analysis**: Advanced perception to identify traversable terrain, steps, or uneven surfaces, feeding into costmap generation.
*   **Safety Critical Control**: Robust fall detection and recovery mechanisms.

For the purpose of this module, we will focus on understanding how to configure Nav2's standard components and conceptualize how a bipedal controller would interface with it, rather than developing a full bipedal gait controller from scratch (which is a research topic in itself).

## 3.2 Global and Local Planning Configuration

Configuring Nav2 involves setting parameters for its various components, primarily through YAML configuration files. This section focuses on the global and local planners, which are responsible for generating the overall path and controlling the robot's immediate motion, respectively.

### 3.2.1 Nav2 Configuration Files

A typical Nav2 setup uses a `navigation.yaml` file (or a set of modular YAML files) to configure all aspects of the stack. This file is loaded by a main Nav2 launch file.

1.  **Example `navigation.yaml` structure**:
    ```yaml
    # navigation.yaml
    
    # Common parameters across all Nav2 components
    ros__parameters:
      use_sim_time: true # Set to true if using simulated time (e.g., from Gazebo)
    
    # -- Map Server Configuration --
    map_server:
      ros__parameters:
        yaml_filename: "map.yaml" # Path to your pre-built map file
    
    # -- AMCL (Localization) Configuration --
    amcl:
      ros__parameters:
        use_sim_time: true
        set_initial_pose: true
        set_initial_pose_x: 0.0
        set_initial_pose_y: 0.0
        set_initial_pose_a: 0.0
        # ... other AMCL parameters (e.g., laser_model_type, recovery_alpha_slow, recovery_alpha_fast)
    
    # -- Global Costmap Configuration --
    global_costmap:
      global_costmap:
        ros__parameters:
          use_sim_time: true
          # ... other costmap parameters
          plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
          static_layer:
            plugin: "nav2_costmap_2d::StaticLayer"
            map_subscribe_topic: "/map"
          obstacle_layer:
            plugin: "nav2_costmap_2d::ObstacleLayer"
            observation_sources: "laser_scan_sensor point_cloud_sensor"
            laser_scan_sensor:
              topic: "/scan" # Topic from your LiDAR
              max_obstacle_height: 2.0
              clearing_threshold: 0.2
            point_cloud_sensor:
              topic: "/point_cloud" # Topic from your 3D sensor (if available)
              max_obstacle_height: 2.0
          inflation_layer:
            plugin: "nav2_costmap_2d::InflationLayer"
            inflation_radius: 0.5
    
    # -- Local Costmap Configuration --
    local_costmap:
      local_costmap:
        ros__parameters:
          use_sim_time: true
          # ... other costmap parameters
          plugins: ["obstacle_layer", "inflation_layer"]
          obstacle_layer:
            plugin: "nav2_costmap_2d::ObstacleLayer"
            observation_sources: "laser_scan_sensor"
            laser_scan_sensor:
              topic: "/scan" # Topic from your LiDAR
              max_obstacle_height: 2.0
              clearing_threshold: 0.2
          inflation_layer:
            plugin: "nav2_costmap_2d::InflationLayer"
            inflation_radius: 0.5
    
    # -- Planner Server Configuration (Global Planner) --
    planner_server:
      ros__parameters:
        use_sim_time: true
        default_plugin: "NavFnPlanner" # Or "SmacPlanner" for more advanced
        plugins:
          - "NavFnPlanner"
          - "SmacPlanner"
        NavFnPlanner:
          plugin: "nav2_navfn_planner::NavfnPlanner"
          # ... NavFnPlanner specific parameters
        SmacPlanner:
          plugin: "nav2_smac_planner::SmacPlanner"
          # ... SmacPlanner specific parameters
    
    # -- Controller Server Configuration (Local Planner) --
    controller_server:
      ros__parameters:
        use_sim_time: true
        default_plugin: "DWBController" # Or "TEBController"
        plugins:
          - "DWBController"
          - "TEBController"
        DWBController:
          plugin: "nav2_dwb_controller::DWBController"
          # ... DWBController specific parameters (e.g., velocities, accelerations, path_dist_bias, goal_dist_bias)
        TEBController:
          plugin: "nav2_teb_controller::TebController"
          # ... TEBController specific parameters
    
    # -- Behavior Tree Navigator --
    bt_navigator:
      ros__parameters:
        use_sim_time: true
        global_frame: map
        robot_base_frame: base_link
        bt_xml_filename: "path/to/my_behavior_tree.xml" # Custom behavior tree if applicable
    
    # -- Waypoint Follower --
    waypoint_follower:
      ros__parameters:
        use_sim_time: true
        # ...
    ```
    *   **Note**: This is a simplified example. A full `navigation.yaml` can be very extensive. Refer to the official [Nav2 Documentation](https://navigation.ros.org/configuration/packages/index.html) for all available parameters.

### 3.2.2 Global Planner Configuration

The global planner (e.g., `NavFnPlanner`, `SmacPlanner`) computes a collision-free path from the robot's start to the goal pose on the global costmap.

*   **Key Parameters for Bipedal Robots**:
    *   `planner_server.ros__parameters.plugins`: Define which global planners are available.
    *   `planner_server.ros__parameters.default_plugin`: Select the active global planner.
    *   **Costmap Interaction**: Ensure the global costmap has appropriate inflation radii to prevent the bipedal robot from getting too close to obstacles, considering its physical size and stability during walking.
    *   **Path Smoothing**: Some planners offer parameters for path smoothing, which can be beneficial for bipedal robots to avoid sharp turns that might destabilize them.

### 3.2.3 Local Planner (Controller) Configuration

The local planner (e.g., `DWBController`, `TEBController`) generates velocity commands to follow the global path while avoiding dynamic obstacles in the local costmap. This is where most bipedal robot adaptations are needed.

*   **Key Parameters for Bipedal Robots**:
    *   `controller_server.ros__parameters.plugins`: Define available controllers.
    *   `controller_server.ros__parameters.default_plugin`: Select the active controller.
    *   **Velocities and Accelerations**: Parameters like `max_vel_x`, `max_vel_theta`, `acc_lim_x`, `acc_lim_theta` in DWB or TEB need to be tuned to match the bipedal robot's gait capabilities. They will generally be much lower than for wheeled robots.
    *   **Path Distance and Goal Distance Biases**: Adjust these to influence how closely the robot follows the global path and how aggressively it approaches the goal.
    *   **Footprint**: The robot's footprint definition in the costmap is crucial. For bipedal robots, this needs to accurately represent the area swept by the robot's body and legs during walking. A simple circle might not be sufficient.
    *   **Custom Controller Plugin (Advanced)**: For true bipedal locomotion, you would replace the default `DWBController` or `TEBController` with a custom `Controller` plugin that interfaces with your bipedal gait generator. This custom plugin would:
        1.  Receive the global plan from the Nav2 stack.
        2.  Generate footstep plans and balance commands based on the local environment and global plan.
        3.  Publish control commands to the robot's joints.

### 3.2.4 Launching Nav2

A typical Nav2 launch file will start all necessary Nav2 nodes (Map Server, AMCL, Planner Server, Controller Server, BT Navigator, etc.) and load the `navigation.yaml` parameters.

1.  **Example `nav2_bringup.launch.py` snippet**:
    ```python
    # In a custom launch file for your bipedal robot
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, GroupAction
    from launch.substitutions import LaunchConfiguration, PythonExpression
    from launch_ros.actions import PushRosNamespace, Node
    from ament_index_python.packages import get_package_share_directory
    import os

    def generate_launch_description():
        share_dir = get_package_share_directory('my_robot_nav2') # Your custom package
        nav_yaml = os.path.join(share_dir, 'config', 'navigation.yaml')
        map_yaml = os.path.join(share_dir, 'maps', 'my_map.yaml')

        # Declare launch arguments
        # ... (e.g., for use_sim_time, autostart)

        # Nav2 Bringup Group
        nav2_bringup_cmd_group = GroupAction([
            PushRosNamespace(LaunchConfiguration('namespace')),
            
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{'yaml_filename': map_yaml}]
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[nav_yaml]
            ),
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[nav_yaml]
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[nav_yaml]
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[nav_yaml]
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[nav_yaml]
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl', 'controller_server', 'planner_server', 'behavior_server', 'bt_navigator']}]
            )
        ])

        return LaunchDescription([
            DeclareLaunchArgument(
                'namespace', default_value='', description='Namespace for all nodes'
            ),
            # ... declare other arguments like use_sim_time
            nav2_bringup_cmd_group
        ])
    ```
    To launch:
    ```bash
    ros2 launch my_robot_nav2 nav2_bringup.launch.py
    ```
    This will start the entire Nav2 stack for your robot. You can then publish a `nav2_msgs/msg/PoseStamped` message to the `/goal_pose` topic (e.g., using RViz2's "2D Goal Pose" tool) to command your robot to a destination.

## 3.3 Behavior Tree Design for Complex Navigation Tasks

Behavior Trees (BTs) are a powerful tool for designing complex, robust, and modular control logic for robots. In Nav2, the Behavior Tree is the central orchestrator that manages the overall navigation task, including goal following, obstacle avoidance, and recovery behaviors.

### 3.3.1 Understanding Behavior Trees

A Behavior Tree is a directed acyclic graph that describes how a robot should behave. It's composed of various nodes:
*   **Control Flow Nodes**: Define how child nodes are executed.
    *   **Sequence (`->`)**: Executes children in order until one fails, or all succeed.
    *   **Selector (`?`)**: Executes children in order until one succeeds, or all fail.
    *   **Parallel (`=`)**: Executes all children simultaneously.
*   **Decorator Nodes**: Modify the behavior of a single child (e.g., `Retry`, `Inverter`, `Timeout`).
*   **Condition Nodes**: Check the state of the robot or environment (e.g., `is_battery_low`, `is_path_valid`).
*   **Action Nodes**: Perform an action (e.g., `navigate_to_pose`, `spin_recovery`).

BTs offer a clear, human-readable way to specify robot behaviors, making them easy to debug and modify.

### 3.3.2 Nav2's Default Behavior Trees

Nav2 provides several default Behavior Trees (e.g., `navigate_to_pose_w_replanning_and_recovery.xml`) that cover common navigation scenarios. These are often sufficient for wheeled robots but might need customization for bipedal platforms.

### 3.3.3 Customizing Behavior Trees for Bipedal Robots

For bipedal robots, you might need to:
*   **Integrate Custom Recovery Behaviors**: If a bipedal robot falls, a specific recovery behavior (e.g., `get_up_action`) might be needed before attempting to navigate again.
*   **Handle Specific Locomotion Modes**: Switch between different walking gaits or balance modes based on terrain or task.
*   **More Complex Obstacle Avoidance**: Integrate vision-based obstacle avoidance into the BT logic.

### 3.3.4 Behavior Tree XML Structure

Behavior Trees in Nav2 are defined using XML files.

1.  **Example XML Snippet (Simplified `navigate_to_pose` for illustration)**:
    ```xml
    <!-- my_custom_bt.xml -->
    <root main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree">
            <Sequence name="NavigateToPose">
                <Action ID="ComputePathToPose" />
                <Action ID="FollowPath" />
                <Action ID="Spin" />
            </Sequence>
        </BehaviorTree>
    </root>
    ```
    *   **Explanation**: This very simplified example defines a sequence of actions: `ComputePathToPose`, `FollowPath`, and `Spin`. In a real Nav2 BT, these would be more complex, including conditions, selectors, and recovery actions.

2.  **Integrating Custom Action Nodes (Python)**:
    You can extend Nav2's Behavior Trees by creating custom action or condition nodes in Python. These nodes execute custom logic when called by the BT.

<!--
    **Example Python Custom Action Node (`my_custom_action.py`):**
    ```python
    import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, ClientState
from nav2_msgs.action import Spin

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence

class SpinAction(Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.node = None
        self.action_client = None
        self.goal_handle = None

    def setup(self, node: Node):
        self.node = node
        self.action_client = ActionClient(self.node, Spin, 'spin')
        self.node.get_logger().info(f"Initialized SpinAction client for {self.name}")
        return True

    def update(self):
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().warn("Spin action server not available!")
            return Status.FAILURE

        if self.goal_handle is None:
            # Send new goal
            goal_msg = Spin.Goal()
            goal_msg.target_yaw = 3.14 # Spin 180 degrees
            self.node.get_logger().info(f"Sending spin goal: {goal_msg.target_yaw}")
            self.future = self.action_client.send_goal_async(goal_msg)
            self.future.add_done_callback(self.goal_response_callback)
            return Status.RUNNING
        
        if self.goal_handle.status == ClientState.ACCEPTED:
            self.node.get_logger().info("Spin goal accepted, waiting for result...")
            self.future = self.goal_handle.get_result_async()
            self.future.add_done_callback(self.get_result_callback)
            return Status.RUNNING
        
        if self.goal_handle.status == ClientState.SUCCEEDED:
            self.node.get_logger().info("Spin action succeeded!")
            self.goal_handle = None
            return Status.SUCCESS
        
        if self.goal_handle.status in [ClientState.ABORTED, ClientState.CANCELED]:
            self.node.get_logger().warn(f"Spin action {self.goal_handle.status}!")
            self.goal_handle = None
            return Status.FAILURE
        
        return Status.RUNNING

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().warn("Spin goal rejected!")
            self.goal_handle = None
        else:
            self.goal_handle = goal_handle

    def get_result_callback(self, future):
        result = future.result().result
        self.node.get_logger().info(f"Spin result: {result}")


def main():
    rclpy.init(args=None)
    node = Node("custom_bt_node")
    try:
        # Example of how you might integrate this action into a tree
        root = Sequence(name="MyCustomSequence")
        spin_behavior = SpinAction(name="CustomSpin")
        root.add_children([spin_behavior]) # Add other behaviors

        # This part is conceptual. In Nav2, the BT is loaded by the bt_navigator
        # For testing, you could create a simple Python tree and tick it.
        
        node.get_logger().info("Custom BT node running...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    ```
    *   **Explanation**: This Python code defines a custom `SpinAction` that uses the `nav2_msgs/action/Spin` action. When this node is included in a package, Nav2's `BehaviorTreeEngine` can load and execute it. You would then reference this custom action in your XML behavior tree.
-->

### 3.3.5 Registering Custom Nodes

To use custom Python or C++ BT nodes, you need to register them with the Nav2 BehaviorTreeEngine. This is typically done by modifying the `bt_navigator` parameters in your `navigation.yaml` or through a specific plugin.

1.  **Add to `navigation.yaml`**:
    ```yaml
    bt_navigator:
      ros__parameters:
        # ... existing parameters
        plugin_libraries:
          - nav2_compute_path_to_pose_action_bt_node
          - nav2_follow_path_action_bt_node
          # ... other default plugins
          - my_robot_nav2_bt_nodes # Your custom BT node library (e.g., from your package.xml)
    ```
    You would also need to define the custom BT node class in your `my_robot_nav2_bt_nodes` package's `plugin.xml` and make sure it's discoverable by `pluginlib`.

By customizing Behavior Trees and integrating custom action nodes, you can create sophisticated and tailored navigation logic for your bipedal robot, allowing it to adapt to complex scenarios and execute specific gaits or recovery behaviors.

## 3.4 Chapter 3 Project/Checkpoint: Autonomous Bipedal Navigation

This project challenges you to configure and use Nav2 to enable autonomous navigation for a simulated bipedal robot.

### Project Goal

Set up a simulated environment for a bipedal robot, configure the Nav2 stack (including global planner, local planner, and behavior tree), and successfully command the robot to navigate autonomously to a series of goal poses in a known map, demonstrating obstacle avoidance.

### Requirements

1.  **Simulated Robot Environment**:
    *   Use a simulated bipedal robot in a Gazebo or Isaac Sim environment (potentially one you created in Module 1 or 2).
    *   Ensure the robot is equipped with appropriate sensors (e.g., LiDAR, RGB-D camera) that publish data to ROS 2 topics that Nav2 can consume for costmap generation.
    *   The robot should have a mechanism to receive velocity commands (e.g., `cmd_vel` topic) and translate them into bipedal locomotion. For this project, a simplified "differential drive" like control for the bipedal robot can be assumed for initial Nav2 integration, with the understanding that a real bipedal controller would map these to gaits.
2.  **Map Creation**:
    *   Generate a 2D occupancy grid map of the simulated environment. This can be done using a SLAM algorithm (e.g., `slam_toolbox`) or by creating a static map.
3.  **Nav2 Configuration**:
    *   Create or adapt a `navigation.yaml` file to configure the Nav2 stack, including:
        *   Map server (loading your generated map).
        *   AMCL for localization.
        *   Global planner (e.g., `NavFnPlanner`, `SmacPlanner`).
        *   Local planner (e.g., `DWBController`, `TEBController`) tuned for slower, bipedal-like movements.
        *   Behavior Tree Navigator (using a default or slightly customized BT XML file).
    *   Ensure costmaps are correctly configured with the robot's footprint and sensor inputs.
4.  **Launch Nav2**:
    *   Create a ROS 2 launch file to bring up your simulated robot, its sensors, and the entire Nav2 stack (Map Server, AMCL, Planner, Controller, BT Navigator).
5.  **Autonomous Navigation**:
    *   Use RViz2's "2D Pose Estimate" to initialize the robot's position on the map.
    *   Use RViz2's "2D Goal Pose" tool to send navigation goals to the robot.

### Verification Steps

To verify the successful completion of this project:

1.  **Launch Simulation and Nav2**: Start your simulated robot environment and the Nav2 stack.
2.  **Localize Robot**: Ensure AMCL successfully localizes your robot within the loaded map in RViz2.
3.  **Command Goals**: Send several different navigation goals to the robot in RViz2, including goals that require turning and navigating around obstacles.
4.  **Observe Navigation**:
    *   **Path Planning**: Observe the global planner generating a valid path to the goal and the local planner adjusting the path for obstacle avoidance.
    *   **Obstacle Avoidance**: Confirm that the robot successfully avoids static and dynamic obstacles without collisions.
    *   **Stability**: While a full bipedal controller is not required, ensure the simulated robot's movements appear reasonable given its assumed "bipedal" nature (i.e., not unnaturally fast or jerky).
    *   **Goal Achievement**: Verify that the robot reliably reaches the commanded goal poses.
5.  **BT Trace (Optional)**: If you've customized the Behavior Tree, you can enable BT logging (`nav2_bt_navigator --ros-args -p bt_logger.enabled:=true`) and inspect the log output to understand the execution flow of the tree during navigation.

By successfully completing this project, you will have gained practical experience in configuring and operating the Nav2 stack for autonomous navigation, adapting it to the unique challenges of bipedal robotics in simulation.

## Visual Aids & Diagrams

To enhance understanding and engagement in Chapter 3, the following visual aids and diagrams are recommended:

*   **Figure 3.1: Nav2 Stack High-Level Architecture**:
    *   **Description**: A high-level block diagram illustrating the main components of the Nav2 stack (Map Server, AMCL, Planner Server, Controller Server, Behavior Tree Navigator) and their data flow. Highlight the interaction points with the robot's sensors and actuators.
    *   **Placement Hint**: After Section 3.1.1, "Overview of the Nav2 Stack".

*   **Figure 3.2: Bipedal Robot Kinematic Constraints**:
    *   **Description**: A diagram comparing the simple kinematic model of a wheeled robot versus the complex, multi-jointed kinematics of a bipedal robot. Emphasize the challenges for navigation.
    *   **Placement Hint**: After Section 3.1.2, "Challenges of Bipedal Navigation".

*   **Figure 3.3: Example Nav2 Costmap**:
    *   **Description**: A screenshot or visualization of RViz2 showing a Nav2 costmap (both global and local). Clearly indicate obstacle inflation layers and the robot's footprint.
    *   **Placement Hint**: After Section 3.2.1, "Nav2 Configuration Files".

*   **Figure 3.4: Global and Local Planner Interaction**:
    *   **Description**: A diagram illustrating how the Global Planner generates a long-term path and the Local Planner (Controller) dynamically adjusts the robot's motion to follow it and avoid local obstacles.
    *   **Placement Hint**: After Section 3.2.3, "Local Planner (Controller) Configuration".

*   **Figure 3.5: Nav2 Behavior Tree Example**:
    *   **Description**: A graphical representation of a simplified Nav2 Behavior Tree (e.g., a variant of `navigate_to_pose`). Clearly show the different node types (Sequence, Selector, Action, Condition) and their hierarchical structure.
    *   **Placement Hint**: After Section 3.3.1, "Understanding Behavior Trees".

*   **Figure 3.6: Custom Python Action Node Integration**:
    *   **Description**: A diagram showing how a custom Python action node (e.g., `SpinAction`) integrates with the Behavior Tree and interfaces with ROS 2 actions.
    *   **Placement Hint**: After Section 3.3.4, "Integrating Custom Action Nodes (Python)".

These visual aids will help students conceptualize the architectural components of Nav2 and the logic behind Behavior Trees for bipedal navigation.

## Troubleshooting & Debugging Guide

This section addresses common issues encountered when configuring and running the Nav2 stack, especially for bipedal robot applications.

*   **1. Nav2 Fails to Launch or Crashes**:
    *   **Problem**: The Nav2 stack fails to start, or nodes crash shortly after launch.
    *   **Possible Causes**: Incorrect `navigation.yaml` parameters, missing ROS 2 packages, conflicts with `use_sim_time`, or resource limitations.
    *   **Solution**:
        *   **Check `navigation.yaml`**: Carefully review your YAML configuration for syntax errors, incorrect parameter names, or invalid plugin types. Pay close attention to topics and frame IDs.
        *   **Verify ROS 2 Packages**: Ensure all Nav2 packages are installed and built correctly (`ros2 pkg list | grep nav2`).
        *   **`use_sim_time` Consistency**: Ensure `use_sim_time` is set consistently across all relevant nodes and launch files (either all `true` for simulation or all `false` for real hardware).
        *   **Resource Monitoring**: Monitor CPU and RAM usage (`htop`, `top`) and GPU usage (`nvidia-smi`) if using GPU-accelerated sensors/perception. Nav2 can be resource-intensive.
        *   **Launch File Debugging**: Use `ros2 launch --debug <package> <launch_file>` to get more verbose output during launch.

*   **2. Robot Not Localizing (AMCL Issues)**:
    *   **Problem**: The robot's estimated position in RViz2 (`/tf` frame from `map` to `odom`) does not match its actual position in the map, or the localization is highly unstable.
    *   **Possible Causes**: Poor initial pose estimate, insufficient sensor data, incorrect sensor noise parameters, or a mismatch between the map and the environment.
    *   **Solution**:
        *   **Provide Good Initial Pose**: Use RViz2's "2D Pose Estimate" tool to give AMCL a rough starting position on the map.
        *   **Verify Sensor Data**: Ensure your LiDAR and/or other sensors are publishing data to the correct topics and that the data is clean and accurate (`ros2 topic echo /scan`).
        *   **AMCL Parameters**: Tune AMCL parameters (e.g., `min_particles`, `max_particles`, `update_min_d`, `update_min_a`, `resample_interval`, `laser_model_type`) for your robot and environment.
        *   **Map Accuracy**: Ensure the map used by the map server (`yaml_filename`) accurately reflects the environment.

*   **3. Global Planner Fails to Find a Path**:
    *   **Problem**: Nav2 reports "Failed to find a path" or the global path is excessively long/indirect.
    *   **Possible Causes**: Goal unreachable, inflated obstacles blocking all paths, costmap resolution too high/low, or planner parameters are too restrictive.
    *   **Solution**:
        *   **Check Goal Reachability**: Visually inspect the goal location in RViz2 to ensure it's not inside an obstacle or completely surrounded by inflated obstacles.
        *   **Inspect Costmaps**: Visualize the global costmap in RViz2. Look for large areas of "lethal" or "inflated" cells that are incorrectly blocking the robot. Adjust `inflation_radius` in the costmap configuration if needed.
        *   **Planner Parameters**: Experiment with parameters specific to your chosen global planner (e.g., `planner_server.<planner_name>.tolerance`).
        *   **Map Boundary**: Ensure the map covers the entire area the robot needs to navigate.

*   **4. Local Planner (Controller) Stuck or Colliding**:
    *   **Problem**: The robot gets stuck trying to navigate, repeatedly attempts to move in place, or collides with obstacles that appear to have enough clearance.
    *   **Possible Causes**: Controller parameters not tuned for bipedal movement, inaccurate footprint, local costmap issues, or insufficient dynamic window for path generation.
    *   **Solution**:
        *   **Tune Controller Parameters**: Crucially, adjust parameters like `max_vel_x`, `max_vel_theta`, `acc_lim_x`, `acc_lim_theta`, and `path_dist_bias`, `goal_dist_bias` in your chosen local planner (e.g., DWB or TEB) to reflect the slower, less agile motion of a bipedal robot.
        *   **Accurate Footprint**: Double-check that the robot's footprint definition in the local costmap accurately reflects its dimensions and any areas it sweeps during walking.
        *   **Local Costmap Visualization**: Visualize the local costmap in RViz2. Ensure it's correctly detecting local obstacles and that the inflation layers are appropriate.
        *   **Dynamic Window/Trajectory Generation**: For DWB/TEB, ensure the `min_vel_x`, `min_vel_theta` and `max_vel_x`, `max_vel_theta` ranges are suitable for bipedal gaits.
        *   **Behavior Tree Recovery**: If the robot gets stuck, the Behavior Tree should trigger recovery behaviors. Ensure your BT has appropriate recovery actions and conditions.

These troubleshooting tips should help you diagnose and resolve common navigation issues when working with the Nav2 stack and bipedal robots. Remember to always use RViz2 and `ros2 topic/node` commands for inspecting the system state.