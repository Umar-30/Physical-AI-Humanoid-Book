# Data Model: Module 2 - The Digital Twin (Gazebo & Unity)

**Feature**: 003-digital-twin-simulation
**Date**: 2025-12-17
**Purpose**: Define key entities, configurations, and data structures for Module 2 educational content

## Overview

Module 2 is a technical book module focused on teaching simulation concepts. The "data entities" are primarily **configuration files**, **code examples**, and **educational artifacts** rather than traditional application data models. This document defines the structure and relationships of these educational entities.

## 1. Educational Content Entities

### 1.1 Chapter

**Purpose**: Organizational unit representing one chapter of Module 2

**Attributes**:
- `chapter_number`: Integer (5-8)
- `title`: String (e.g., "Gazebo Physics, Worlds, Robot Spawning")
- `slug`: String (e.g., "chapter-5-gazebo-basics")
- `learning_objectives`: Array of strings (2-5 objectives)
- `prerequisites`: Array of strings (references to prior chapters/modules)
- `estimated_duration`: Duration (e.g., "2-3 hours")
- `sections`: Array of Section entities

**Relationships**:
- Has many Sections (1:N)
- Has many CodeExamples (1:N)
- Has many ConfigurationFiles (1:N)
- Has many Exercises (1:N)

**Validation Rules**:
- chapter_number must be 5-8 for Module 2
- Each chapter must have at least 3 sections
- Each chapter must have at least 1 code example
- Each chapter must have at least 1 exercise

### 1.2 Section

**Purpose**: Subsection within a chapter covering a specific topic

**Attributes**:
- `section_number`: String (e.g., "5.1", "5.2")
- `title`: String (e.g., "Gazebo Harmonic Installation")
- `slug`: String (e.g., "gazebo-installation")
- `content_type`: Enum (`tutorial`, `concept`, `reference`, `troubleshooting`)
- `markdown_content`: MDX string
- `estimated_read_time`: Duration (e.g., "15 minutes")

**Relationships**:
- Belongs to Chapter (N:1)
- Has many CodeExamples (1:N)
- Has many Diagrams (1:N)
- May reference ConfigurationFiles (N:M)

**Validation Rules**:
- section_number must match pattern: `{chapter}.{section}` (e.g., "5.1")
- markdown_content must be valid MDX
- All internal links must resolve
- All code blocks must have language specifier

### 1.3 Code Example

**Purpose**: Executable code snippet demonstrating a concept

**Attributes**:
- `example_id`: String (unique identifier, e.g., "gazebo-spawn-robot-cmd")
- `language`: Enum (`bash`, `xml`, `python`, `csharp`, `yaml`)
- `code_content`: String (the actual code)
- `title`: String (descriptive title for display)
- `description`: String (explains what the code does)
- `expected_output`: String (optional, shows expected terminal output)
- `platform`: Enum (`ubuntu`, `windows`, `macos`, `cross-platform`)
- `tested`: Boolean (whether code has been executed and verified)
- `test_date`: Date (when last tested)

**Relationships**:
- Belongs to Section (N:1)
- May depend on ConfigurationFiles (N:M)

**Validation Rules**:
- code_content must be syntactically valid for specified language
- If expected_output is provided, actual execution must match
- tested must be true before chapter publication
- test_date must be within 30 days of publication

**Example Instance**:
```json
{
  "example_id": "gazebo-spawn-robot-cmd",
  "language": "bash",
  "code_content": "gz sim humanoid_world.sdf\ngz service -s /world/humanoid_world/create \\...",
  "title": "Spawn Robot via Command Line",
  "description": "Demonstrates spawning a URDF robot into running Gazebo simulation",
  "expected_output": "[Msg] Inserted entity [my_robot] with id [42]",
  "platform": "cross-platform",
  "tested": true,
  "test_date": "2025-12-15"
}
```

### 1.4 Configuration File

**Purpose**: Complete configuration file (SDF, URDF, launch file, Unity scene) provided as example

**Attributes**:
- `file_name`: String (e.g., "humanoid_world.sdf", "robot_with_sensors.urdf")
- `file_type`: Enum (`sdf`, `urdf`, `launch_py`, `launch_xml`, `unity_scene`, `csharp_script`, `yaml`)
- `file_content`: String (complete file contents)
- `description`: String (what the file configures)
- `annotations`: Array of Annotation objects (inline comments/explanations)
- `dependencies`: Array of strings (other files this depends on)
- `validated`: Boolean (whether file syntax has been validated)
- `functional_test`: Boolean (whether file has been tested functionally)

**Relationships**:
- Referenced by Sections (N:M)
- May depend on other ConfigurationFiles (N:M)
- Has many Annotations (1:N)

**Validation Rules**:
- file_content must be syntactically valid (XML for SDF/URDF, Python for launch, etc.)
- validated must be true (syntax check passed)
- functional_test must be true (file successfully loads/executes)
- All dependencies must exist

**Example Instance**:
```json
{
  "file_name": "humanoid_world.sdf",
  "file_type": "sdf",
  "description": "Basic Gazebo world with ground plane and lighting for humanoid robot simulation",
  "annotations": [
    {
      "line_number": 5,
      "annotation": "Physics timestep: 0.001s = 1000Hz update rate"
    },
    {
      "line_number": 12,
      "annotation": "Gravity: Standard Earth gravity (9.81 m/s²)"
    }
  ],
  "dependencies": [],
  "validated": true,
  "functional_test": true
}
```

### 1.5 Exercise

**Purpose**: Hands-on practice task for readers

**Attributes**:
- `exercise_number`: String (e.g., "5.1", "5.2")
- `title`: String (e.g., "Create Custom Gazebo World")
- `difficulty`: Enum (`beginner`, `intermediate`, `advanced`)
- `estimated_time`: Duration (e.g., "30 minutes")
- `instructions`: String (step-by-step instructions)
- `success_criteria`: Array of strings (how to verify completion)
- `solution_provided`: Boolean
- `solution_reference`: String (path to solution file/section)

**Relationships**:
- Belongs to Chapter (N:1)
- May reference CodeExamples (N:M)
- May reference ConfigurationFiles (N:M)

**Validation Rules**:
- At least 1 exercise per chapter
- solution_provided must be true for all exercises
- success_criteria must be measurable/verifiable

**Example Instance**:
```json
{
  "exercise_number": "5.1",
  "title": "Create Custom Gazebo World",
  "difficulty": "beginner",
  "estimated_time": "30 minutes",
  "instructions": "1. Create a new SDF file named 'my_world.sdf'\n2. Add ground plane...",
  "success_criteria": [
    "Gazebo launches and displays your world",
    "Ground plane is visible",
    "Lighting illuminates the scene"
  ],
  "solution_provided": true,
  "solution_reference": "/files/module-2/solutions/exercise-5-1-solution.sdf"
}
```

### 1.6 Diagram

**Purpose**: Visual illustration (architecture diagram, flowchart, screenshot)

**Attributes**:
- `diagram_id`: String (unique identifier)
- `file_path`: String (path in static/img/ directory)
- `file_format`: Enum (`svg`, `png`, `jpg`)
- `title`: String (descriptive title)
- `alt_text`: String (accessibility description)
- `caption`: String (optional extended description)
- `source_tool`: Enum (`draw.io`, `excalidraw`, `screenshot`, `rendered`, `other`)

**Relationships**:
- Referenced by Sections (N:M)

**Validation Rules**:
- file_path must exist
- alt_text must be descriptive (>20 characters)
- svg preferred for diagrams (scalable, small file size)
- png required for screenshots

**Example Instance**:
```json
{
  "diagram_id": "gazebo-unity-data-flow",
  "file_path": "/img/module-2/gazebo-unity-pipeline.svg",
  "file_format": "svg",
  "title": "Gazebo-Unity Data Flow Architecture",
  "alt_text": "Diagram showing data flow from Gazebo physics engine through ROS 2 middleware to Unity rendering engine, with joint states, sensor data, and TF transforms labeled",
  "caption": "Multi-simulator pipeline showing how Gazebo publishes robot state to ROS 2 topics, which Unity subscribes to via TCP/IP bridge",
  "source_tool": "draw.io"
}
```

## 2. Simulation Configuration Entities

### 2.1 World Configuration

**Purpose**: Gazebo SDF world file structure

**Key Elements**:
```xml
<world>
  <physics>               # Physics engine settings
    <max_step_size />     # Timestep (typically 0.001-0.002s)
    <real_time_factor />  # Target real-time multiplier
  </physics>
  <gravity />             # Gravity vector (default: 0 0 -9.81)
  <model>                 # Static models (ground, obstacles)
  <light>                 # Lighting configuration
</world>
```

**Validation**:
- Must conform to SDF 1.9 schema
- Physics timestep should be ≤0.002s for humanoid stability
- At least one light source required
- Ground plane strongly recommended

### 2.2 Robot Configuration (URDF)

**Purpose**: Robot description including sensors and physics

**Key Elements**:
```xml
<robot>
  <link>
    <visual />            # Appearance (mesh, material)
    <collision />         # Collision geometry (simplified)
    <inertial>
      <mass />
      <inertia />         # Inertia tensor
    </inertial>
  </link>
  <joint>
    <dynamics>
      <friction />        # Joint friction
      <damping />         # Joint damping
    </dynamics>
    <limit>
      <effort />          # Max torque
      <velocity />        # Max angular velocity
    </limit>
  </joint>
  <gazebo>               # Gazebo-specific extensions
    <sensor />           # Sensor definitions (IMU, camera, lidar)
    <plugin />           # ROS 2 interface plugins
  </gazebo>
</robot>
```

**Validation**:
- Must conform to URDF specification
- All links must have inertial properties (no zero mass)
- Collision geometries should be simpler than visual meshes
- Sensor plugins must specify ROS 2 topics

### 2.3 Unity Scene Configuration

**Purpose**: Unity scene structure for robot visualization

**Key Components**:
- **ROSConnection**: TCP/IP bridge configuration
  - ROS IP address (localhost or remote)
  - Port number (default 10000)
  - Protocol (ROS 2)

- **Robot GameObject**: Unity representation of robot
  - ArticulationBody components (for joints)
  - MeshRenderer components (for visuals)
  - RobotController script (syncs with ROS 2)

- **Human Avatar**: Interactive HRI visualization
  - Animator component (for gestures/animations)
  - NavMesh agent (for autonomous movement)
  - Collision triggers (for proximity detection)

- **UI Canvas**: Sensor data overlays
  - Text elements (IMU readings, joint states)
  - RawImage elements (camera feed display)

**Validation**:
- ROSConnection must successfully connect to ros_tcp_endpoint
- Joint names in Unity must match URDF joint names
- All referenced meshes must exist in Unity project

### 2.4 Launch Configuration

**Purpose**: ROS 2 launch file to coordinate Gazebo, Unity bridge, and nodes

**Key Elements** (Python launch file):
```python
LaunchDescription([
  ExecuteProcess(
    cmd=['gz', 'sim', 'world.sdf', '-r'],  # Start Gazebo in run mode
  ),
  Node(
    package='ros_tcp_endpoint',
    executable='default_server_endpoint',  # Start Unity bridge
    parameters=[{'ROS_IP': '0.0.0.0', 'ROS_TCP_PORT': 10000}]
  ),
  Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',  # Bridge Gazebo topics to ROS 2
    arguments=['joint_states@sensor_msgs/msg/JointState@gz.msgs.Model']
  ),
])
```

**Validation**:
- All referenced packages must be installed
- Node executables must exist
- Arguments must match actual topic names and message types

## 3. Educational Assessment Entities

### 3.1 Learning Objective

**Purpose**: Measurable outcome readers should achieve

**Attributes**:
- `objective_id`: String
- `chapter_number`: Integer (5-8)
- `objective_text`: String (starts with action verb: "Understand", "Configure", "Create")
- `bloom_level`: Enum (`remember`, `understand`, `apply`, `analyze`, `evaluate`, `create`)
- `assessment_method`: String (how objective is verified)

**Example**:
```json
{
  "objective_id": "LO-5-01",
  "chapter_number": 5,
  "objective_text": "Create a custom Gazebo world file with ground plane, lighting, and physics configuration",
  "bloom_level": "create",
  "assessment_method": "Exercise 5.1: Student-created world file successfully loads in Gazebo"
}
```

### 3.2 Success Criterion (from Spec)

**Purpose**: Module-level measurable outcome (from spec.md Success Criteria)

**Mapping to Educational Content**:
- **SC-001**: Chapter 5 exercises achievable in 30 minutes
- **SC-002**: Chapter 6 includes 3+ sensor types with verification steps
- **SC-003**: Chapter 8 provides latency measurement procedure (<100ms target)
- **SC-004**: Chapter 8 includes 10-minute stability test procedure
- **SC-005/006**: Chapters 5 and 7 include performance monitoring instructions
- **SC-007**: 90% success rate target → simplify instructions, add troubleshooting
- **SC-008**: All examples must execute without errors → thorough testing required
- **SC-009**: Troubleshooting sections in each chapter resolve 80%+ of issues
- **SC-010**: Chapter 8 final exercise requires custom robot model modification

## 4. Entity Relationships

### 4.1 Dependency Graph

```
Module 2
├── Chapter 5 (Gazebo Basics)
│   ├── Section 5.1 (Installation)
│   │   ├── Code Example: apt install command
│   │   └── Code Example: version check
│   ├── Section 5.2 (World Files)
│   │   ├── Config File: humanoid_world.sdf
│   │   ├── Diagram: SDF structure
│   │   └── Code Example: launch world
│   ├── Section 5.3 (Robot Spawning)
│   │   ├── Code Example: spawn command
│   │   ├── Config File: spawn_robot.launch.py
│   │   └── Config File: example_robot.urdf (dependency)
│   └── Exercises
│       └── Exercise 5.1 (depends on Sections 5.2, 5.3)
│
├── Chapter 6 (URDF Physics)
│   ├── Section 6.1 (Sensor Integration)
│   │   ├── Config File: robot_with_imu.urdf
│   │   ├── Config File: robot_with_camera.urdf
│   │   └── Code Example: verify sensor topics
│   ├── Section 6.2 (Collision Geometry)
│   │   ├── Diagram: collision primitives vs meshes
│   │   └── Config File: collision_comparison.urdf
│   ├── Section 6.3 (Inertia Calculation)
│   │   ├── Code Example: Python inertia calculator
│   │   └── Config File: robot_with_inertia.urdf
│   └── Exercises
│       └── Exercise 6.1 (add sensors to custom robot)
│
├── Chapter 7 (Unity Rendering)
│   ├── Section 7.1 (Unity Setup)
│   │   └── Code Example: Unity package installation
│   ├── Section 7.2 (ROS Integration)
│   │   ├── Code Example: start ros_tcp_endpoint
│   │   ├── Config File: Unity ROS settings
│   │   └── Config File: C# RobotController.cs
│   ├── Section 7.3 (Scene Creation)
│   │   ├── Diagram: Unity scene hierarchy
│   │   └── Config File: robot_scene.unity (conceptual)
│   └── Exercises
│       └── Exercise 7.1 (create Unity scene)
│
└── Chapter 8 (Multi-Simulator)
    ├── Section 8.1 (Architecture)
    │   └── Diagram: Gazebo-Unity data flow ★★★ (KEY DIAGRAM)
    ├── Section 8.2 (Launch Coordination)
    │   └── Config File: launch_both_simulators.launch.py
    ├── Section 8.3 (Synchronization)
    │   ├── Code Example: Unity joint state sync script
    │   └── Code Example: measure latency
    ├── Section 8.4 (Performance)
    │   └── Code Example: performance monitoring
    └── Exercises
        └── Exercise 8.1 (complete pipeline with custom robot)
```

### 4.2 Cross-Chapter Dependencies

- Chapter 6 **depends on** Chapter 5 (assumes Gazebo installed, world created)
- Chapter 7 **depends on** Chapters 5-6 (needs functional Gazebo sim to visualize)
- Chapter 8 **integrates** Chapters 5-7 (combines all concepts)

## 5. Validation Rules Summary

### File-Level Validation

**Configuration Files**:
- [ ] Syntactically valid (XML for SDF/URDF, Python for launch)
- [ ] All dependencies exist
- [ ] Successfully loads/executes in target environment
- [ ] Tested within 30 days of publication

**Code Examples**:
- [ ] Syntactically correct for specified language
- [ ] Actually executes without errors
- [ ] Expected output matches actual output
- [ ] Platform-specific notes included where applicable

**Diagrams**:
- [ ] File exists at specified path
- [ ] Alt text provided (accessibility)
- [ ] Caption clarifies diagram purpose
- [ ] SVG format for diagrams (PNG only for screenshots)

### Chapter-Level Validation

- [ ] All learning objectives map to assessable content
- [ ] Each section references at least 1 code example or config file
- [ ] Troubleshooting section present
- [ ] At least 1 exercise with solution
- [ ] All internal links resolve
- [ ] Docusaurus build succeeds

### Module-Level Validation

- [ ] All success criteria (SC-001 through SC-010) addressed
- [ ] Chapters build progressively (no forward references to unteaching concepts)
- [ ] Prerequisites from Module 1 accurately referenced
- [ ] Complete end-to-end workflow achievable (Chapter 5 → 8)
- [ ] External beta tester completes module successfully

## 6. Storage and Organization

**Physical Storage**:
```
docs/module-2-digital-twin/          # Educational content (MDX)
static/img/module-2/                 # Diagrams and screenshots
static/files/module-2/               # Downloadable config files
specs/003-digital-twin-simulation/   # Planning documents (this file)
```

**Metadata Storage**:
- **Front matter** (YAML in MDX files): Chapter/section metadata
- **Git history**: Version control for all content
- **Docusaurus search index**: Auto-generated from content

**Version Control Strategy**:
- Feature branch: `003-digital-twin-simulation`
- One commit per completed section
- Pull request for chapter completion (requires review)
- Tag releases: `module-2-v1.0.0` (semantic versioning)

---

**Data Model Status**: Complete
**Next Phase**: Create contracts/ directory with detailed chapter outlines
**Note**: This is an educational content project, so "data entities" are primarily configuration files, code examples, and documentation artifacts rather than traditional application data.
