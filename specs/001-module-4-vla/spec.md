# Feature Specification: Module 4: Vision-Language-Action (VLA)

**Feature Branch**: `001-module-4-vla`  
**Created**: December 12, 2025  
**Status**: Draft  
**Input**: User description: "You are an expert at the intersection of Large Language Models, robotics, and edge AI systems. Your task is to write the complete, detailed instructional content for **Module 4: Vision-Language-Action (VLA)**, broken down into its four chapters as specified, culminating in the Capstone Project. For **each chapter and its subsections**, please provide: 1. **Detailed Step-by-Step Instructions:** Assume the student has a Jetson Orin kit and a powerful workstation from previous modules. Provide concrete, copy-pasteable code. * **Chapter 1:** Provide the Python code for the `voice_input_node` using `faster-whisper`, including audio stream handling and ROS 2 topic publishing. * **Chapter 2:** Provide example prompts for GPT-4 and Claude to generate ROS action sequences. Also, provide code for a local planner node using the `llama_cpp` Python library with a quantized model. * **Chapter 3:** Provide a snippet for a ROS action server `scan_for_object` that calls a Grounding DINO-based inference service. * **Chapter 4:** Provide a state machine diagram (as pseudocode or Mermaid JS) for the Orchestrator Node and the master launch file that brings up the entire system. 2. **Clear Explanations of Complex Concepts:** Make advanced topics accessible. * Explain the "tool-use" paradigm for LLMs in robotics with analogies. * Contrast open-vocabulary detection with traditional fixed-class detection. * Discuss the critical trade-offs between cloud vs. local LLMs in terms of latency, cost, and reliability for a moving robot. 3. **System Architecture Diagrams:** Describe the necessary diagrams. * A comprehensive data flow diagram showing the journey from audio signal to joint command, highlighting all ROS topics, nodes, and external services (Whisper, LLM API). * A diagram of the Orchestrator Node's finite state machine. 4. **Troubleshooting & Optimization Guide:** Address real-world issues. * **Chapter 1:** Whisper mishears commands in noisy environments; audio stream latency. * **Chapter 2:** LLM generates invalid or unsafe plans; API timeouts or high costs. * **Chapter 3:** Open-vocabulary detector is slow on Jetson or fails on ambiguous descriptions. * **Chapter 4:** The pipeline works in isolation but breaks in integration; race conditions or timing issues. 5. **Chapter Project/Checkpoint:** Define a clear, testable milestone for each chapter that students must demo (e.g., for Chapter 2: "Show a terminal log where your planner node receives the text 'wave your right hand' and outputs a valid ROS 2 action sequence."). **Structure your output exactly as follows:** ### Module 4: Vision-Language-Action (VLA) #### Chapter 1: Voice as the Primary Interface (Voice-to-Action) *(Proceed with subsections 1.1, 1.2, 1.3)* #### Chapter 2: Language as a Planning Tool (Cognitive Planning) *(Proceed with subsections 2.1, 2.2, 2.3)* #### Chapter 3: Grounding Language in Vision (Vision-Language Integration) *(Proceed with subsections 3.1, 3.2, 3.3)* #### Chapter 4: The Autonomous Humanoid Capstone Integration *(Proceed with subsections 4.1, 4.2, 4.3, 4.4)* **Tone:** Be insightful and forward-thinking. Write for students who are now proficient roboticists and are ready to integrate the most cutting-edge AI technologies to build truly intelligent behavior. Emphasize the "why" behind architectural choices."

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

### User Story 1 - Voice as the Primary Interface (Priority: P1)

This chapter introduces the foundational voice interface, enabling students to process audio input into text commands for robot control.

**Why this priority**: This chapter introduces the foundational voice interface, which is critical for subsequent VLA chapters.

**Independent Test**: Student can successfully run the `voice_input_node` and publish audio data to a ROS 2 topic.

**Acceptance Scenarios**:

1. **Given** a Jetson Orin kit and a workstation with ROS 2 and `faster-whisper` installed, **When** the student launches the `voice_input_node`, **Then** the node starts successfully and processes audio input, publishing transcribed text to a ROS 2 topic.
2. **Given** the `voice_input_node` is running, **When** the student speaks a command, **Then** the transcribed text is accurately published to the designated ROS 2 topic.

---

### User Story 2 - Language as a Planning Tool (Priority: P1)

This chapter covers how LLMs translate natural language commands into robot actions, forming the core cognitive part of the VLA pipeline.

**Why this priority**: This chapter covers how LLMs translate natural language commands into robot actions, forming the core cognitive part of the VLA pipeline.

**Independent Test**: Student can demonstrate a planner node receiving a text command and outputting a valid ROS 2 action sequence.

**Acceptance Scenarios**:

1. **Given** a local planner node running with `llama_cpp`, **When** the student inputs a text command like "wave your right hand", **Then** the planner node processes the command and outputs a valid ROS 2 action sequence that a robot could execute.
2. **Given** example prompts for GPT-4 and Claude, **When** the student uses these prompts with their respective APIs, **Then** the APIs return appropriate ROS action sequences for robotics tasks.

---

### User Story 3 - Grounding Language in Vision (Priority: P1)

This chapter integrates visual perception with language, enabling the robot to understand and interact with its environment based on natural language descriptions.

**Why this priority**: This chapter integrates visual perception with language, enabling the robot to understand and interact with its environment based on natural language descriptions.

**Independent Test**: Student can demonstrate a ROS action server detecting a specified object.

**Acceptance Scenarios**:

1. **Given** a `scan_for_object` ROS action server integrated with a Grounding DINO-based inference service, **When** the student sends a request to detect an object (e.g., "red cup"), **Then** the action server successfully detects the object in its visual field and returns its location.
2. **Given** an ambiguous object description, **When** the student sends a request to detect the object, **Then** the system handles the ambiguity gracefully, potentially asking for clarification or indicating uncertainty.

---

### User Story 4 - The Autonomous Humanoid Capstone Integration (Priority: P1)

This chapter integrates all previous components into a full, autonomous humanoid system, representing the culmination of the module.

**Why this priority**: This chapter integrates all previous components into a full, autonomous humanoid system, representing the culmination of the module.

**Independent Test**: Student can demonstrate the fully integrated VLA pipeline, from voice command to robot action.

**Acceptance Scenarios**:

1. **Given** the Orchestrator Node and all other VLA pipeline components (voice input, planner, vision) are launched via the master launch file, **When** the student issues a voice command (e.g., "Find the blue block and pick it up"), **Then** the humanoid robot executes the necessary actions to fulfill the command.
2. **Given** the state machine diagram for the Orchestrator Node, **When** the student analyzes the system's behavior, **Then** the robot's actions align with the defined states and transitions for various commands.



### Edge Cases

- What happens when the voice command is ambiguous or unclear? (e.g., "pick up that thing")
- How does the system handle noisy environments for voice input?
- What happens if the LLM generates an unsafe or illogical action sequence?
- How does the system recover from API timeouts or high costs associated with cloud LLMs?
- What if the open-vocabulary detector fails to identify an object or identifies it incorrectly?
- How are timing issues and race conditions handled in the integrated pipeline?
- What happens if a required external service (e.g., LLM API, Grounding DINO service) is unavailable?

## Requirements *(mandatory)*

<!--
  ACTION REQUIRED: The content in this section represents placeholders.
  Fill them out with the right functional requirements.
-->

### Functional Requirements

- **FR-001**: The instructional content MUST provide detailed, copy-pasteable Python code for the `voice_input_node` using `faster-whisper`, including audio stream handling and ROS 2 topic publishing.
- **FR-002**: The instructional content MUST provide example prompts for GPT-4 and Claude to generate ROS action sequences.
- **FR-003**: The instructional content MUST provide Python code for a local planner node using the `llama_cpp` library with a quantized model.
- **FR-004**: The instructional content MUST provide a snippet for a ROS action server `scan_for_object` that calls a Grounding DINO-based inference service.
- **FR-005**: The instructional content MUST provide a state machine diagram (as pseudocode or Mermaid JS) for the Orchestrator Node.
- **FR-006**: The instructional content MUST provide the master launch file that brings up the entire VLA system.
- **FR-007**: The instructional content MUST include clear explanations of the "tool-use" paradigm for LLMs in robotics with analogies.
- **FR-008**: The instructional content MUST contrast open-vocabulary detection with traditional fixed-class detection.
- **FR-009**: The instructional content MUST discuss the critical trade-offs between cloud vs. local LLMs in terms of latency, cost, and reliability for a moving robot.
- **FR-010**: The instructional content MUST include a comprehensive data flow diagram showing the journey from audio signal to joint command, highlighting all ROS topics, nodes, and external services.
- **FR-011**: The instructional content MUST include a diagram of the Orchestrator Node's finite state machine.
- **FR-012**: The instructional content MUST provide troubleshooting and optimization guidance for Whisper mishearing commands in noisy environments and audio stream latency.
- **FR-013**: The instructional content MUST provide troubleshooting and optimization guidance for LLM generating invalid or unsafe plans, and API timeouts or high costs.
- **FR-014**: The instructional content MUST provide troubleshooting and optimization guidance for open-vocabulary detector being slow on Jetson or failing on ambiguous descriptions.
- **FR-015**: The instructional content MUST provide troubleshooting and optimization guidance for the VLA pipeline breaking in integration, race conditions, or timing issues.
- **FR-016**: Each chapter MUST define a clear, testable milestone for students to demo.

### Key Entities

- **Voice Input Node (`voice_input_node`)**: A ROS 2 node responsible for capturing audio, transcribing it using `faster-whisper`, and publishing the text to a ROS 2 topic.
- **Planner Node**: A ROS 2 node utilizing `llama_cpp` (or similar LLM library) that takes transcribed text commands and generates ROS action sequences.
- **Scan for Object Action Server (`scan_for_object`)**: A ROS 2 action server that interfaces with a Grounding DINO-based inference service to perform open-vocabulary object detection based on natural language descriptions.
- **Orchestrator Node**: The central ROS 2 node that manages the overall VLA pipeline, coordinating between the voice input, planner, vision, and robot control systems, potentially implemented as a finite state machine.
- **ROS Topics**: Communication channels for transmitting data between ROS 2 nodes (e.g., transcribed text, action commands, sensor data).
- **ROS Actions**: A request/response communication pattern in ROS 2 for long-running tasks, particularly for robot behaviors.
- **External LLM APIs (GPT-4, Claude)**: Cloud-based large language models used for generating complex ROS action sequences.
- **Local LLM (`llama_cpp` with quantized model)**: An on-device large language model used for localized planning.
- **Grounding DINO Inference Service**: A visual perception service that performs open-vocabulary object detection.
- **Humanoid Robot**: The physical or simulated robot receiving joint commands and executing actions.
- **Audio Stream**: Raw audio data captured from microphones.
- **Transcribed Text**: Natural language commands converted from audio.
- **ROS Action Sequences**: Structured commands that define a series of robot movements or behaviors.
- **Object Detections**: Bounding boxes and labels of detected objects in the environment.
- **System Architecture Diagrams**: Visual representations of the data flow and component interactions within the VLA system.

## Assumptions

- Students have a Jetson Orin kit and a powerful workstation set up from previous modules.
- Students have a foundational understanding of ROS 2 concepts and Python programming.
- Necessary hardware (microphones, cameras, humanoid robot platforms) are available or simulated for student use.
- Access to external LLM APIs (GPT-4, Claude) is assumed, or alternatives are provided for local development if cloud access is not feasible.
- Quantized `llama_cpp` models are available and suitable for local planner node implementation.
- Grounding DINO-based inference service can be set up and integrated with ROS 2.

## Success Criteria *(mandatory)*

<!--
  ACTION REQUIRED: Define measurable success criteria.
  These must be technology-agnostic and measurable.
-->

### Measurable Outcomes

- **SC-001**: Students can successfully implement and demonstrate the `voice_input_node` with `faster-whisper`, achieving a minimum of 90% accuracy in transcribing spoken commands in a quiet environment.
- **SC-002**: Students can successfully implement and demonstrate the planner node, translating natural language commands into valid ROS 2 action sequences, with an 80% success rate for predefined commands.
- **SC-003**: Students can successfully implement and demonstrate the `scan_for_object` ROS action server, achieving a 75% accuracy rate in detecting specified objects using Grounding DINO.
- **SC-004**: Students can successfully integrate all VLA pipeline components and demonstrate an autonomous humanoid robot responding to voice commands, executing the commanded actions with a 70% success rate in a controlled environment.
- **SC-005**: Students can clearly explain the "tool-use" paradigm, contrast open-vocabulary vs. fixed-class detection, and discuss cloud vs. local LLM trade-offs, demonstrating a strong understanding of underlying concepts.
- **SC-006**: The generated system architecture and Orchestrator Node state machine diagrams accurately represent the VLA pipeline and its control flow.
- **SC-007**: Students can effectively troubleshoot common issues and apply optimization techniques as described in the module, resolving at least 70% of simulated problems within a reasonable timeframe.