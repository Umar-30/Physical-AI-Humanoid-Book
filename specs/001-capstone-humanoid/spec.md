# Feature Specification: Capstone Project: The Autonomous Humanoid

**Feature Branch**: `001-capstone-humanoid`  
**Created**: December 13, 2025  
**Status**: Draft  
**Input**: User description: "You are a senior robotics systems architect and educator. Your task is to generate the complete, ready-to-use specification for the **Capstone Project: The Autonomous Humanoid**. This document will be given to students as their final project brief. Using the detailed outline above, write a comprehensive project specification that includes: 1. **Project Brief & Objectives:** Start with a compelling overview that frames the capstone as the culmination of the Physical AI course. Clearly state the high-level goal and the "mission scenario" (living room, target object, voice command). 2. **Technical System Architecture:** Provide a **detailed, labeled system architecture diagram** described in text or Mermaid.js format. Specify every major ROS 2 node, key topics (e.g., `/voice_cmd`, `/detections`, `/goal_pose`), and the data flow between the four module layers. Instruct students to replicate this diagram in their documentation. 3. **Phase-Wise Development Plan:** Detail the four-phase plan (Research→Foundation→Analysis→Synthesis). For each phase: * List the specific, actionable development tasks. * Specify the parallel "research spike" questions that must be investigated. * Define the concrete **Validation Gate** that must be passed before moving to the next phase. Provide example commands or expected outputs for each gate. 4. **Explicit Integration Instructions:** Offer direct technical guidance on the trickiest integration points: * How should the `orchestrator_node` (FSM) be structured? Provide a Python class skeleton. * What is the exact ROS action/service interface that the vision node and navigation node must expose for the planner to use? Provide sample `.action` and `.srv` file definitions. * How should the simulation environment (Gazebo *and* Unity) be launched alongside the ROS 2 stack? Provide a master launch file example. 5. **Deliverables & Grading Rubric:** Present the final submission requirements (Repo, Docs, Video) in a clear, bulleted list. Lay out the grading rubric as a table with categories (Integration, Autonomy, Code, Innovation), weightings, and specific criteria for earning top marks in each. 6. **Troubleshooting & FAQ:** Preempt common integration pitfalls. Address issues like: * "My LLM planner works but the actions fail." (Answer: Emphasize action server timeouts and precondition checks). * "Everything works in isolation but crashes when launched together." (Answer: Guide on using `ros2 launch` with namespaces and careful resource management). * "The robot gets lost or the detection fails intermittently." (Answer: Discuss the need for robust error states in the FSM to trigger re-planning or recovery behaviors). **Tone:** Write as a challenging but supportive project manager. This document should excite students about the problem while providing them with the concrete scaffolding needed to succeed. Assume they have the knowledge from all four modules but need guidance on the complex system integration."

## User Scenarios & Testing *(mandatory)*

<!--
  IMPORTANT: User stories should be PRIORITIZED as user journeys ordered by importance.
  Each user story/journey must be INDEPENDENTLY TESTABLE - meaning if you implement just ONE of them,
  you should still have a viable MVP (Minimum Viable Product) that delivers value.
  
  Assign priorities (P1, P2, P3, etc.) to each story, where P1 is the most critical.
  Think of each story as a standalone slice of functionality that can be:
  - Developed independently
  - Tested independently
  - Deployed independently
  - Demonstrated to users independently
-->

## Project Brief & Objectives

Welcome, senior roboticists, to the culmination of your journey through Physical AI! This Capstone Project challenges you to synthesize all the knowledge and skills acquired in Modules 1-4 to create a truly autonomous humanoid robot. This isn't just an exercise; it's your opportunity to demonstrate mastery in integrating advanced AI (Vision-Language-Action) with complex robotic systems.

### Mission Scenario: The Living Room Assistant

Your mission, should you choose to accept it, is to program a humanoid robot to act as an intelligent assistant in a simulated living room environment. The robot must be capable of understanding complex voice commands, reasoning about its environment, performing navigation, object perception, and manipulation to fulfill user requests.

**High-Level Goal**: The autonomous humanoid robot must successfully identify, navigate to, interact with (e.g., pick up, push), and deliver a specified target object within the simulated living room environment, all initiated by a natural language voice command.

**Example Voice Command**: "Robot, please find the red cup on the coffee table and bring it to me."

**Core Objectives**:
1- Integrate voice input (Whisper) for natural language command processing.
2- Utilize a Large Language Model (LLM) for cognitive planning, translating high-level commands into a sequence of executable robot actions.
3- Incorporate vision-language models (e.g., Grounding DINO) for open-vocabulary object detection and localization.
4- Implement a robust navigation stack (Nav2) capable of planning and executing paths for a bipedal robot.
5- Develop an Orchestrator Node (Finite State Machine) to manage the entire VLA pipeline, including task sequencing, error handling, and recovery behaviors.
6- Demonstrate the robot's ability to operate autonomously within a simulated environment (Gazebo and/or Unity).

---

## Technical System Architecture

The Autonomous Humanoid operates as a complex, integrated Vision-Language-Action (VLA) system, leveraging ROS 2 as its middleware. The architecture is modular, allowing for independent development and testing of components, yet tightly coupled through well-defined interfaces (ROS 2 topics, actions, and services). Students are expected to understand and replicate this architectural design in their own documentation and implementation.

### System Architecture Diagram

This diagram illustrates the core components and data flow within the VLA system. Arrows indicate data flow, and labels specify ROS 2 topics, actions, or services.

```mermaid
graph TD
    subgraph User Interaction
        A[Human User] -- Voice Command --> B(Microphone);
    end

    subgraph Perception Layer
        B --> C(Voice Input Node);
        C -- /voice_command (std_msgs/String) --> D(Planner Node);
        E[Camera Sensor] --> F(Vision Node / Grounding DINO Server);
    end

    subgraph Cognitive Layer
        D -- Action Sequence --> G(Orchestrator Node);
        F -- object_detection_action (my_vla_interfaces/ScanObject.action) --> G;
    end

    subgraph Actuation & Control Layer
        G -- nav_goal_action (nav2_msgs/NavigateToPose.action) --> H(Navigation Node / Nav2 Stack);
        G -- manip_action (robot_control_interfaces/GraspObject.action) --> I(Manipulation Node);
        H -- /cmd_vel (geometry_msgs/Twist) --> J(Robot Base Controller);
        I -- /joint_commands (sensor_msgs/JointState) --> K(Robot Arm/Gripper Controller);
    end

    subgraph Robot Hardware/Simulation
        J --> L[Humanoid Robot (Simulated/Physical)];
        K --> L;
    end

    subgraph Data Flow
        VoiceInputNode -- ROS 2 Node --> PlannerNode;
        PlannerNode -- LLM Decision --> OrchestratorNode;
        VisionNode -- ROS 2 Action Server --> OrchestratorNode;
        OrchestratorNode -- ROS 2 Action Client --> NavigationNode;
        OrchestratorNode -- ROS 2 Action Client --> ManipulationNode;
        NavigationNode -- ROS 2 Topic --> RobotBaseController;
        ManipulationNode -- ROS 2 Topic --> RobotArmGripperController;
        RobotBaseController --> HumanoidRobot;
        RobotArmGripperController --> HumanoidRobot;
    end

    style A fill:#f9f,stroke:#333,stroke-width:2px;
    style B fill:#bbf,stroke:#333,stroke-width:2px;
    style C fill:#ccf,stroke:#333,stroke-width:2px;
    style D fill:#ddf,stroke:#333,stroke-width:2px;
    style E fill:#fcf,stroke:#333,stroke-width:2px;
    style F fill:#cff,stroke:#333,stroke-width:2px;
    style G fill:#ffc,stroke:#333,stroke-width:2px;
    style H fill:#fcc,stroke:#333,stroke-width:2px;
    style I fill:#cfc,stroke:#333,stroke-width:2px;
    style J fill:#f9f,stroke:#333,stroke-width:2px;
    style K fill:#bbf,stroke:#333,stroke-width:2px;
    style L fill:#ccf,stroke:#333,stroke-width:2px;
```
*Figure 1: Comprehensive VLA System Architecture for the Autonomous Humanoid*

**Key Components & Data Flow**:

1.  **Voice Input Node**:
    *   **Input**: Raw audio from Microphone.
    *   **Function**: Transcribes spoken commands to text using `faster-whisper`.
    *   **Output**: Publishes transcribed text to `/voice_command` (std_msgs/String).
2.  **Planner Node**:
    *   **Input**: Subscribes to `/voice_command`.
    *   **Function**: Uses an LLM (local `llama_cpp` or cloud API) to interpret the text command and generate a sequence of robot actions (tool calls).
    *   **Output**: Publishes action sequence to `/robot_action_sequence` (std_msgs/String).
3.  **Vision Node (Grounding DINO Server)**:
    *   **Input**: Images from Camera Sensor.
    *   **Function**: Performs open-vocabulary object detection based on natural language queries (received via ROS action/service).
    *   **Output**: Provides object detections (pose, label, confidence) via `object_detection_action` (my_vla_interfaces/ScanObject.action).
4.  **Navigation Node (Nav2 Stack)**:
    *   **Input**: Robot state, sensor data (LiDAR, IMU), global goals via `nav_goal_action`.
    *   **Function**: Plans and executes collision-free paths for bipedal locomotion.
    *   **Output**: Publishes velocity commands to `/cmd_vel` (geometry_msgs/Twist) for robot base control.
5.  **Manipulation Node**:
    *   **Input**: Grasp commands via `manip_action`.
    *   **Function**: Controls the robot's arm and gripper to perform manipulation tasks.
    *   **Output**: Publishes joint commands to `/joint_commands` (sensor_msgs/JointState).
6.  **Orchestrator Node (FSM)**:
    *   **Input**: Subscribes to `/robot_action_sequence`, receives results from `object_detection_action`, `nav_goal_action`, `manip_action`.
    *   **Function**: The central decision-making unit, implemented as a Finite State Machine (FSM). It parses LLM-generated action sequences, coordinates calls to Vision, Navigation, and Manipulation nodes, and manages overall task flow, error handling, and recovery behaviors.
    *   **Output**: Sends goals to Vision, Navigation, and Manipulation nodes.
7.  **Robot Controllers**: Low-level controllers that translate commands (e.g., `/cmd_vel`, `/joint_commands`) into physical movements of the Humanoid Robot.
8.  **Humanoid Robot**: The simulated or physical robot that executes the actions.

**Student Task**: You are required to fully understand this system architecture. In your project documentation, you must provide your own version of this detailed architecture diagram, clearly labeling all nodes, topics, actions, and data flows as implemented in your system. Highlight any deviations or custom additions.

---

## Phase-Wise Development Plan

The Capstone Project is structured into four distinct phases. Each phase has specific objectives, tasks, parallel research "spikes," and a mandatory Validation Gate that must be passed to ensure a structured and successful development process.

### Phase 1: Research & Setup

**Objective**: Understand the problem, finalize the project scope, and set up the foundational development environment.

**Development Tasks**:
-   **P1.1**: Thoroughly review Module 4 content (VLA pipeline components) and identify key integration points.
-   **P1.2**: Select the humanoid robot model for simulation (Gazebo/Unity) and ensure its ROS 2 interfaces are understood.
-   **P1.3**: Set up a dedicated ROS 2 workspace for the Capstone Project.
-   **P1.4**: Configure development environment for Python, `rclpy`, `faster-whisper`, `llama_cpp`, etc.

**Research Spikes**:
-   **RS1.1**: Investigate options for humanoid robot models in Gazebo/Unity. What are their existing ROS 2 interfaces? What are their kinematic/dynamic limitations?
-   **RS1.2**: Research best practices for integrating Gazebo/Unity with ROS 2 (e.g., `ros_gz_bridge`, `ros_unity_interface`). Identify potential pitfalls.

**Validation Gate 1: Environment Readiness**
-   **Description**: Demonstrate that your chosen humanoid robot model can be launched in the selected simulation environment and that its basic ROS 2 interfaces (e.g., joint states, command velocities) are accessible and functional.
-   **Expected Output**:
    -   Screenshot of your humanoid robot successfully launched in Gazebo/Unity.
    -   Terminal output showing `ros2 topic echo /joint_states` or `ros2 topic list` with relevant robot topics.
    -   Confirmation that you can send a basic command (e.g., move a joint) via ROS 2.

---

### Phase 2: Foundation & Component Integration

**Objective**: Implement and test individual VLA components (Voice, Planner, Vision, Navigation) in isolation and perform initial ROS 2 integration.

**Development Tasks**:
-   **P2.1**: Implement and test the `voice_input_node` (from Module 4, Chapter 4.1).
-   **P2.2**: Implement and test the `planner_node` (from Module 4, Chapter 4.2).
-   **P2.3**: Implement and test the `scan_for_object` vision node/action server (from Module 4, Chapter 4.3).
-   **P2.4**: Integrate Nav2 for basic path planning and locomotion for your humanoid robot (conceptual adaptation from Module 3).

**Research Spikes**:
-   **RS2.1**: Experiment with different `faster-whisper` model sizes and `llama_cpp` quantization levels on your hardware for optimal performance.
-   **RS2.2**: Dive deeper into Nav2 customization for bipedal robots: What custom controller plugins might be relevant? How to map `cmd_vel` to humanoid gaits? (Focus on conceptual understanding for this phase).

**Validation Gate 2: Core Component Functionality**
-   **Description**: Individually demonstrate the functionality of each core VLA component implemented in your Capstone workspace.
-   **Expected Output**:
    -   `voice_input_node`: Terminal log showing accurate transcription of your voice commands.
    -   `planner_node`: Terminal log showing the `planner_node` receiving a voice command and outputting a valid ROS 2 action sequence (e.g., `TOOL_CALL`).
    -   `scan_for_object` node: Terminal log showing successful detection of a specified object.
    -   Nav2: Robot can receive a goal in RViz2 and a path is planned, and basic movements are observed in simulation.

---

### Phase 3: Analysis & Orchestration

**Objective**: Design and implement the Orchestrator Node (FSM) to coordinate the VLA pipeline and begin full system integration.

**Development Tasks**:
-   **P3.1**: Design the Finite State Machine (FSM) for your `orchestrator_node`, mapping states and transitions to Capstone mission objectives.
-   **P3.2**: Implement the `orchestrator_node` using Python and `rclpy` (referencing the provided skeleton).
-   **P3.3**: Integrate the `orchestrator_node` with the `voice_input_node`, `planner_node`, `vision_node`, and Nav2.
-   **P3.4**: Develop custom ROS 2 action/service interfaces as needed for precise robot control (e.g., manipulation, specialized locomotion).

**Research Spikes**:
-   **RS3.1**: Explore advanced FSM design patterns or libraries in Python for ROS 2.
-   **RS3.2**: Research ROS 2 best practices for robust error handling and recovery mechanisms in complex systems.

**Validation Gate 3: Orchestrated Command Execution**
-   **Description**: Demonstrate the `orchestrator_node` successfully managing a multi-step voice command, coordinating at least two VLA components seamlessly.
-   **Expected Output**:
    -   Terminal logs showing the `orchestrator_node` transitioning through states.
    -   Voice command given (e.g., "Find the red block").
    -   Robot (in simulation) performs vision scan, and the `orchestrator_node` receives the result, then transitions to plan further actions (even if not fully executed yet).

---

### Phase 4: Synthesis & Refinement

**Objective**: Refine the integrated system, ensure robustness, and achieve autonomous execution of the Capstone mission.

**Development Tasks**:
-   **P4.1**: Fine-tune parameters for all VLA components to optimize performance and reliability.
-   **P4.2**: Implement advanced error handling and recovery behaviors within the `orchestrator_node`.
-   **P4.3**: Conduct extensive testing of the full VLA pipeline in various simulated living room scenarios.
-   **P4.4**: Optimize communication protocols and data flow for minimal latency.
-   **P4.5**: Prepare final project deliverables (code repository, documentation, demonstration video).

**Research Spikes**:
-   **RS4.1**: Investigate techniques for robust object manipulation in simulation (e.g., grasp planning, inverse kinematics).
-   **RS4.2**: Research methods for integrating external sensor data for enhanced environmental awareness within Nav2.

**Validation Gate 4: Autonomous Mission Completion**
-   **Description**: The humanoid robot autonomously completes the "Living Room Assistant" mission scenario from start to finish, responding to the voice command and successfully interacting with the target object.
-   **Expected Output**:
    -   A successful run where the robot (in simulation) processes the voice command, navigates, detects, interacts, and delivers the object without critical failure.
    -   Video recording of the complete mission scenario.
    -   Clean terminal logs demonstrating successful component integration.


---

## Explicit Integration Instructions

Successfully integrating the diverse components of the VLA pipeline is the most critical aspect of this Capstone Project. This section provides direct technical guidance and examples for key integration points.

### 4.1 Orchestrator Node Structure (FSM)

The `orchestrator_node` is the heart of your robot's intelligence. It should be implemented as a ROS 2 Python node utilizing a Finite State Machine (FSM) to manage complex multi-step tasks. Here is a Python class skeleton to guide your implementation:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
import time

# Import custom action/service interfaces (you will define these)
# from my_vla_interfaces.action import ScanObject
# from my_robot_control.action import MoveToPose, GraspObject

class RobotState:
    IDLE = 0
    PROCESSING_COMMAND = 1
    PLANNING_ACTION = 2
    EXECUTING_NAVIGATION = 3
    EXECUTING_VISION = 4
    EXECUTING_MANIPULATION = 5
    ERROR_STATE = 6
    RECOVERY_STATE = 7

class OrchestratorNode(Node):
    def __init__(self):
        super().__init__('orchestrator_node')
        self.get_logger().info('Orchestrator Node started. Initializing FSM...')

        # State variable
        self.current_state = RobotState.IDLE
        self.command_queue = [] # Queue for incoming voice commands

        # ROS 2 Subscriptions
        # Subscribe to transcribed voice commands
        self.voice_sub = self.create_subscription(
            String,
            '/voice_command',
            self.voice_command_callback,
            10,
            callback_group=ReentrantCallbackGroup())
        
        # Subscribe to LLM-generated action sequences
        self.planner_sub = self.create_subscription(
            String,
            '/robot_action_sequence',
            self.planner_output_callback,
            10,
            callback_group=ReentrantCallbackGroup())

        # ROS 2 Action Clients (example placeholders)
        self.scan_object_client = ActionClient(self, ScanObject, 'scan_for_object')
        self.move_to_pose_client = ActionClient(self, MoveToPose, 'move_to_pose')
        self.grasp_object_client = ActionClient(self, GraspObject, 'grasp_object')

        # Internal variables for FSM state management
        self.active_goal = None
        self.planner_response = None
        self.current_action_sequence = []
        self.current_action_index = 0

        # FSM loop timer
        self.fsm_timer = self.create_timer(0.1, self.fsm_loop_callback) # Run FSM at 10Hz

        self.get_logger().info("Orchestrator Node FSM initialized to IDLE state.")

    def voice_command_callback(self, msg):
        # Handle incoming voice commands
        self.get_logger().info(f"Received voice command: {msg.data}")
        self.command_queue.append(msg.data) # Enqueue for FSM processing

    def planner_output_callback(self, msg):
        # Handle LLM-generated action sequences
        self.get_logger().info(f"Received planner output: {msg.data}")
        self.planner_response = msg.data # Store for FSM to process

    def fsm_loop_callback(self):
        # This is the core FSM logic. Implement state transitions here.
        if self.current_state == RobotState.IDLE:
            if self.command_queue:
                command = self.command_queue.pop(0)
                self.get_logger().info(f"Processing new command: '{command}'")
                # Trigger planner (e.g., publish to planner_node's input topic, or direct call)
                self.current_state = RobotState.PLANNING_ACTION
                # You might need to publish a String msg to the planner_node if it's not subscribed to voice_command directly
                # self.planner_input_publisher.publish(String(data=command))

        elif self.current_state == RobotState.PLANNING_ACTION:
            if self.planner_response:
                # Parse LLM response (e.g., "TOOL_CALL: move_to_pose(x=1.0,y=0.0)")
                if self.planner_response.startswith("TOOL_CALL:"):
                    # Convert to executable action sequence
                    self.current_action_sequence = self._parse_llm_tool_calls(self.planner_response)
                    self.current_action_index = 0
                    self.get_logger().info(f"LLM planned: {self.current_action_sequence}")
                    self.current_state = RobotState.EXECUTING_NAVIGATION # Or first action state
                else:
                    self.get_logger().warn(f"LLM responded with non-actionable text: {self.planner_response}")
                    self.current_state = RobotState.IDLE # Return to idle if no plan
                self.planner_response = None # Consume response
            # Implement timeout for planning

        elif self.current_state == RobotState.EXECUTING_NAVIGATION:
            if not self.active_goal:
                # Send navigation goal using self.move_to_pose_client
                # self.active_goal = self.move_to_pose_client.send_goal_async(...)
                self.get_logger().info("Executing Navigation (Simulated)")
                self.active_goal = "simulated_nav_goal"
                # For this skeleton, assume success quickly
                time.sleep(1) # Simulate
                self.get_logger().info("Navigation simulated success.")
                self.active_goal = None
                self.current_state = RobotState.EXECUTING_VISION # Next step in sequence
            # Monitor navigation action client status
            # If done and success, move to next action in sequence, else ERROR_STATE

        elif self.current_state == RobotState.EXECUTING_VISION:
            if not self.active_goal:
                # Send scan object goal using self.scan_object_client
                # self.active_goal = self.scan_object_client.send_goal_async(...)
                self.get_logger().info("Executing Vision (Simulated ScanObject)")
                self.active_goal = "simulated_scan_goal"
                # For this skeleton, assume success quickly
                time.sleep(1) # Simulate
                self.get_logger().info("Vision simulated success.")
                self.active_goal = None
                self.current_state = RobotState.EXECUTING_MANIPULATION # Next step
            # Monitor vision action client status
            # If done and success, move to next action, else ERROR_STATE

        elif self.current_state == RobotState.EXECUTING_MANIPULATION:
            if not self.active_goal:
                # Send grasp object goal using self.grasp_object_client
                # self.active_goal = self.grasp_object_client.send_goal_async(...)
                self.get_logger().info("Executing Manipulation (Simulated GraspObject)")
                self.active_goal = "simulated_grasp_goal"
                # For this skeleton, assume success quickly
                time.sleep(1) # Simulate
                self.get_logger().info("Manipulation simulated success.")
                self.active_goal = None
                self.current_state = RobotState.IDLE # Task completed
            # Monitor manipulation action client status
            # If done and success, check if more actions in sequence, else IDLE or ERROR_STATE

        elif self.current_state == RobotState.ERROR_STATE:
            self.get_logger().error("Robot in ERROR_STATE. Attempting recovery...")
            self.current_state = RobotState.RECOVERY_STATE
            # Trigger recovery behaviors

        elif self.current_state == RobotState.RECOVERY_STATE:
            self.get_logger().warn("Robot in RECOVERY_STATE. (Simulated)")
            # Simulate recovery attempt
            time.sleep(3)
            self.get_logger().info("Simulated recovery completed. Returning to IDLE.")
            self.current_state = RobotState.IDLE # Or retry previous action

    def _parse_llm_tool_calls(self, llm_response_string):
        # This function needs robust parsing of LLM tool call format
        # Example: "TOOL_CALL: move_to_pose(x=1.0, y=0.5)" -> ["move_to_pose", {"x": 1.0, "y": 0.5}]
        self.get_logger().warn("'_parse_llm_tool_calls' is a placeholder. Implement robust parsing.")
        return [llm_response_string] # Return as a list of actions

    def destroy_node(self):
        self.get_logger().info("Shutting down Orchestrator Node.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    orchestrator_node = OrchestratorNode()
    executor.add_node(orchestrator_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        orchestrator_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4.2 ROS Action/Service Interfaces for Vision and Navigation

For the Planner Node to effectively command Vision and Navigation, these capabilities must expose clear, well-defined ROS 2 action or service interfaces.

#### Vision Node Interface: `ScanObject.action`

The vision node (e.g., integrating Grounding DINO) should expose a ROS 2 Action for object detection.

```action
# Request
string object_description  # e.g., "red cup", "the monitor on the desk"
---
# Result
bool success
string message
geometry_msgs/Pose object_pose  # Pose of the detected object
---
# Feedback
string status_message # e.g., "Scanning for 'red cup'..."
```
**Explanation**:
-   **Request**: `object_description` is the natural language query for the object to be detected.
-   **Result**: `success` indicates if the object was found, `message` provides details, and `object_pose` gives the 3D pose of the detected object.
-   **Feedback**: Provides real-time status updates during the scanning process.

#### Navigation Node Interface: `MoveToPose.action`

The navigation node (e.g., an adapter for Nav2) should expose a ROS 2 Action to command the robot to move to a specific pose.

```action
# Request
geometry_msgs/PoseStamped target_pose # Goal pose for the robot
---
# Result
bool success
string message
---
# Feedback
float32 distance_remaining
float32 angle_remaining
```
**Explanation**:
-   **Request**: `target_pose` specifies the desired 3D pose (position and orientation) the robot should navigate to.
-   **Result**: `success` indicates if the navigation was successful, and `message` provides details.
-   **Feedback**: Provides real-time updates on the robot's progress toward the goal.

### 4.3 Simulation Environment Launch File Example

A master ROS 2 launch file is essential for bringing up the entire Capstone system, including your chosen simulation environment (Gazebo and/or Unity) and all ROS 2 nodes.

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Add other arguments as needed, e.g., for robot model, map file

    # Paths to your custom ROS 2 packages
    my_vla_voice_pkg_dir = get_package_share_directory('my_vla_voice')
    my_vla_planner_pkg_dir = get_package_share_directory('my_vla_planner')
    my_vla_vision_pkg_dir = get_package_share_directory('my_vla_vision')
    my_vla_orchestrator_pkg_dir = get_package_share_directory('my_vla_orchestrator') # Ensure you create this package

    # Path to Nav2's bringup launch file (adapt as needed for your Nav2 setup)
    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        # 1. Launch Simulation Environment (Example: Gazebo)
        # You would typically include a launch file for your specific humanoid robot and Gazebo world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('my_robot_description'), 'launch', 'spawn_humanoid.launch.py') # Example
            ]),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # 2. Launch Nav2 Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(nav2_bringup_launch_dir, 'bringup_launch.py')
            ]),
            launch_arguments={
                'map': os.path.join(get_package_share_directory('my_robot_nav2'), 'maps', 'my_living_room.yaml'), # Example
                'use_sim_time': use_sim_time
            }.items(),
        ),

        # 3. Launch VLA Core Nodes
        Node(
            package='my_vla_voice',
            executable='voice_input_node',
            name='voice_input_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='my_vla_planner',
            executable='planner_node',
            name='planner_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='my_vla_vision',
            executable='scan_for_object_server',
            name='scan_for_object_action_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='my_vla_orchestrator',
            executable='orchestrator_node',
            name='orchestrator_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Add other robot specific nodes, e.g., joint controllers, manipulation drivers
    ])
```
**Explanation**: This launch file orchestrates the startup of your simulated robot, the Nav2 stack, and all the VLA pipeline nodes. Remember to create the `my_vla_orchestrator` package and the custom action/service definitions. If using Unity, you would integrate its bridge or separate nodes here.

---

## Deliverables & Grading Rubric

Your Capstone Project submission should demonstrate your ability to design, implement, and integrate complex robotic systems. Attention to detail, robust engineering practices, and clear communication are paramount.

### Deliverables

Your final submission will consist of three main components:

1.  **Code Repository (GitHub/GitLab)**:
    *   A well-organized and clearly documented repository containing all your ROS 2 packages, scripts, launch files, robot models, and configuration files.
    *   Includes a `README.md` with setup instructions, a build guide, and execution commands.
    *   Proper use of Git for version control (meaningful commits, branches).
2.  **Documentation (Docusaurus/Markdown)**:
    *   A comprehensive project report (e.g., using Docusaurus, or a detailed Markdown file).
    *   Must include:
        *   **Project Overview**: Your interpretation of the mission scenario and objectives.
        *   **System Architecture**: Your replicated and potentially enhanced system architecture diagram with explanations.
        *   **Implementation Details**: Description of your approach for each VLA component and the Orchestrator Node.
        *   **Development Process**: Summary of your phase-wise development, challenges encountered, and solutions.
        *   **Results & Analysis**: Discussion of your robot's performance, success rate, and observations during the mission.
        *   **Future Work**: Ideas for further improvements and extensions.
3.  **Demonstration Video (5-7 minutes)**:
    *   A high-quality video showcasing your robot successfully completing the Capstone mission scenario.
    *   Clearly narrate the robot's actions and the underlying VLA processes.
    *   Highlight successful task completion, error handling, and any innovative features.
    *   Include snippets of your code or terminal outputs during execution to verify functionality.

### Grading Rubric

Your project will be evaluated based on the following criteria:

| Category               | Weighting | Criteria for Top Marks                                                                                                                                                                                                                                                                                         |
| :--------------------- | :-------- | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **1. Integration & Functionality** | 40%       | - All VLA components (voice, planner, vision, navigation, manipulation) are seamlessly integrated and work together.                                                                                                                                                                                 <br>- The robot consistently (80%+ success rate) completes the core mission scenario.                                                                                                              <br>- Voice commands are accurately interpreted and executed.                                                                                                                             |
| **2. Autonomy & Robustness**     | 30%       | - The Orchestrator Node's FSM effectively manages complex task sequences and coordinates components.                                                                                                                                                                                                 <br>- Robust error handling and recovery behaviors are implemented for common failure modes (e.g., object not found, path blocked).                                                             <br>- The robot operates autonomously with minimal human intervention during the mission.                                                                                                    |
| **3. Code Quality & Documentation** | 20%       | - Code is clean, well-structured, modular, and adheres to ROS 2/Python best practices.                                                                                                                                                                                                               <br>- Comprehensive in-code comments and clear variable naming.                                                                                                                              <br>- Project documentation is thorough, easy to understand, and includes the system architecture diagram and implementation details.                                                             |
| **4. Innovation & Problem Solving** | 10%       | - Demonstrates creative solutions to challenges or novel adaptations of VLA concepts.                                                                                                                                                                                                                <br>- Goes beyond the basic requirements with additional features or advanced control strategies.                                                                                                      <br>- Clear articulation of problem-solving approach in documentation.                                                                                                                    |


---

## Troubleshooting & FAQ

Even the most seasoned roboticists encounter challenges during complex system integration. This section aims to preempt common pitfalls and provide guidance for debugging your Autonomous Humanoid.

### Common Integration Pitfalls and Solutions

1.  **"My LLM planner works, but the robot actions fail."**
    *   **Diagnosis**: This often indicates a mismatch between the LLM's planned action (syntax, parameters) and the robot's actual capabilities (action server interfaces, valid ranges).
    *   **Solution**:
        *   **Action Server Timeouts**: Ensure your action clients (e.g., in the `orchestrator_node`) have appropriate timeouts. If an action server is not available or doesn't respond, the client should not hang indefinitely.
        *   **Precondition Checks**: Implement robust precondition checks within your robot's action servers. Before attempting a `grasp_object` action, for example, ensure the object is within reach and the gripper is open. If preconditions fail, report back to the Orchestrator.
        *   **LLM Output Validation**: Your `planner_node` or `orchestrator_node` must rigorously validate the LLM's output against the defined ROS action/service interfaces. Reject malformed commands and potentially trigger the LLM to re-plan with corrected constraints.
        *   **Semantic vs. Syntactic**: Ensure the LLM understands not just the syntax of your tools, but also their semantic meaning and constraints. This requires careful prompt engineering.

2.  **"Everything works in isolation but crashes when launched together."**
    *   **Diagnosis**: This typically points to resource contention, incorrect ROS 2 graph configuration (namespaces, remappings, QoS), or race conditions during startup.
    *   **Solution**:
        *   **`ros2 launch` with Namespaces**: Use namespaces judiciously in your master launch file (`vla_system.launch.py`) to prevent topic/node name collisions, especially if integrating third-party packages like Nav2.
        *   **Careful Resource Management**: Monitor CPU, GPU, and RAM usage (`htop`, `nvidia-smi` on Jetson). Resource-intensive nodes (LLM inference, vision) might starve others. Consider `nice` or `cgroups` for resource allocation if necessary.
        *   **Node Startup Order**: While ROS 2 launch system handles some dependencies, explicit delays or `LifecycleNodes` might be needed for nodes with strict startup requirements.
        *   **Quality of Service (QoS)**: Ensure consistent QoS settings (e.g., `reliability`, `durability`) across publishers and subscribers, especially for high-bandwidth data like images or sensor streams.
        *   **Logging**: Increase log levels (`rclpy.logging.set_logger_level`) for all nodes during integration testing to pinpoint which node crashes and why.

3.  **"The robot gets lost or the detection fails intermittently."**
    *   **Diagnosis**: This indicates issues with localization, mapping, sensor data quality, or the robustness of your object detection.
    *   **Solution**:
        *   **Robust Error States in FSM**: Design your `orchestrator_node` FSM with dedicated error states. If an object isn't detected or localization is lost, the FSM should transition to a state that triggers re-planning, re-scanning, or recovery behaviors (e.g., "spin in place and re-scan," "return to home pose").
        *   **Sensor Fusion**: For robust localization, integrate multiple sensor modalities (e.g., LiDAR, IMU, odometry) within Nav2's AMCL.
        *   **Map Quality**: Ensure your navigation map is accurate and up-to-date.
        *   **Vision Confidence Thresholds**: Tune the confidence threshold for your Grounding DINO detections. Too low, and you get false positives; too high, and you miss objects.
        *   **Illumination/Occlusion Handling**: Acknowledge that vision performance can degrade in poor lighting or with occlusions. Your system should be able to trigger strategies to mitigate this.

### Key Takeaway

A successful autonomous robot system is not just about individual components working, but about how gracefully they interact, how robustly they handle failures, and how intelligently they adapt. Embrace debugging as a core part of the learning process!

### Edge Cases

- What happens if the voice command is outside the robot's capabilities?
- How should the system handle dynamic obstacles not present in the initial map?
- What if the target object is not found by the vision system after multiple attempts?
- How should the robot recover from a fall or an unrecoverable error state?
- What if communication (ROS topics, external APIs) fails intermittently?
- How to ensure safe operation in scenarios where the LLM might generate ambiguous or unsafe actions?

## Requirements *(mandatory)*

<!--
  ACTION REQUIRED: The content in this section represents placeholders.
  Fill them out with the right functional requirements.
-->

### Functional Requirements

-   **FR-001**: The project specification MUST include a compelling overview that frames the capstone as the culmination of the Physical AI course, stating the high-level goal and the mission scenario (living room, target object, voice command).
-   **FR-002**: The project specification MUST provide a detailed, labeled system architecture diagram (text or Mermaid.js format), specifying every major ROS 2 node, key topics, and data flow between the four module layers.
-   **FR-003**: The project specification MUST detail a four-phase development plan (Research→Foundation→Analysis→Synthesis).
-   **FR-004**: For each phase of the development plan, the specification MUST list specific, actionable development tasks.
-   **FR-005**: For each phase of the development plan, the specification MUST specify parallel "research spike" questions to be investigated.
-   **FR-006**: For each phase of the development plan, the specification MUST define concrete Validation Gates with example commands or expected outputs.
-   **FR-007**: The project specification MUST offer direct technical guidance on integrating the `orchestrator_node`, including a Python class skeleton.
-   **FR-008**: The project specification MUST define the exact ROS action/service interface for the vision node and navigation node for use by the planner, providing sample `.action` and `.srv` file definitions.
-   **FR-009**: The project specification MUST provide instructions on how the simulation environment (Gazebo and Unity) should be launched alongside the ROS 2 stack, with a master launch file example.
-   **FR-010**: The project specification MUST present final submission requirements (Repo, Docs, Video) in a clear, bulleted list.
-   **FR-011**: The project specification MUST lay out a grading rubric as a table with categories (Integration, Autonomy, Code, Innovation), weightings, and specific criteria for earning top marks.
-   **FR-012**: The project specification MUST include a "Troubleshooting & FAQ" section addressing common integration pitfalls, such as LLM planner issues, full system crashes, and robot localization/detection failures.

### Key Entities

-   **Humanoid Robot**: The target platform for the capstone, expected to operate in a simulated (Gazebo/Unity) environment.
-   **Voice Command**: Natural language input from the user to initiate actions.
-   **ROS 2 Nodes**: Major computational units of the robot's software architecture (e.g., `voice_input_node`, `planner_node`, `vision_node`, `navigation_node`, `orchestrator_node`).
-   **ROS 2 Topics**: Communication channels for streaming data (e.g., `/voice_cmd` for transcribed commands, `/detections` for vision output, `/goal_pose` for navigation goals).
-   **ROS 2 Actions/Services**: Inter-node communication for goal-oriented tasks (e.g., `ScanObject.action`, `MoveToPose.action`).
-   **Orchestrator Node (FSM)**: The central control logic, implemented as a finite state machine, to manage the VLA pipeline.
-   **LLM Planner**: Component responsible for converting natural language commands into robot action sequences.
-   **Vision Node**: Component responsible for object detection and localization (e.g., using Grounding DINO).
-   **Navigation Node**: Component responsible for path planning and robot movement (e.g., using Nav2).
-   **Simulation Environment**: Gazebo and Unity platforms used for testing and development.
-   **Development Phases**: Structured approach to project execution (Research, Foundation, Analysis, Synthesis).
-   **Validation Gates**: Specific checkpoints to verify progress and functionality.
-   **Deliverables**: Required submissions for the project (Repository, Documentation, Video).
-   **Grading Rubric**: Criteria for evaluating the project (Integration, Autonomy, Code, Innovation).
-   **Troubleshooting & FAQ**: Common issues and their solutions.

## Assumptions

- Students have successfully completed all four preceding modules of the Physical AI course.
- Students have access to and proficiency with a Jetson Orin kit and a powerful workstation.
- Students have a strong foundational understanding of ROS 2, Python programming, basic robotics, and AI/ML concepts.
- Necessary software environments (ROS 2, simulation tools, Python libraries) are pre-configured or easily installable.
- Access to external LLM APIs (e.g., GPT-4, Claude) is assumed, or students will use local LLMs where appropriate.
- Simulation environments (Gazebo, Unity) are available and can be integrated with ROS 2.
- The humanoid robot model in simulation is functional and provides appropriate ROS 2 interfaces for control and sensing.

## Success Criteria *(mandatory)*

<!--
  ACTION REQUIRED: Define measurable success criteria.
  These must be technology-agnostic and measurable.
-->

### Measurable Outcomes

-   **SC-001**: Students can successfully implement an autonomous humanoid robot (in simulation) that responds to a complex voice command (e.g., "Find the red cup and bring it to me") by identifying the object, navigating to it, picking it up, and delivering it, achieving an 80% success rate across multiple trials.
-   **SC-002**: The implemented VLA system adheres to the specified technical system architecture, with clearly defined ROS 2 nodes, topics, and data flows, as demonstrated in the student's documentation.
-   **SC-003**: Students demonstrate a clear understanding and application of the phase-wise development plan, passing all Validation Gates with documented evidence (e.g., screenshots, logs).
-   **SC-004**: The `orchestrator_node` FSM is correctly structured and robustly manages the VLA pipeline, gracefully handling common error states (e.g., object not found, path blocked).
-   **SC-005**: The implemented ROS action/service interfaces for vision and navigation nodes accurately reflect the provided definitions, enabling seamless integration with the LLM planner.
-   **SC-006**: The simulation environment (Gazebo and Unity) is correctly launched alongside the ROS 2 stack, demonstrating robust inter-simulation communication and control.
-   **SC-007**: The final project submission (repository, documentation, video) is comprehensive, well-documented, and clearly demonstrates the robot's autonomous capabilities.
-   **SC-008**: The robot's behavior demonstrates innovation in handling complex scenarios or user interactions, reflecting advanced problem-solving beyond basic requirements.
