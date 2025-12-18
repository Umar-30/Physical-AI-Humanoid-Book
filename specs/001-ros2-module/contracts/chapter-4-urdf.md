# Chapter 4 Contract: URDF for Humanoid Robots

**Chapter ID**: ch4-urdf
**Directory**: `docs/module-1-ros2/04-urdf/`
**Estimated Hours**: 2.5 hours
**Status**: Design

---

## Learning Objectives

By the end of this chapter, learners will be able to:

1. Understand URDF (Unified Robot Description Format) XML syntax
2. Define links with visual, collision, and inertial properties
3. Define joints with types, axes, limits, and dynamics
4. Explain coordinate frames and transformations in robotics
5. Create a simplified humanoid robot URDF (8-12 links)
6. Visualize URDF in RViz with joint state control
7. Add sensors (camera, IMU) to robot description
8. Validate URDF using check_urdf tool
9. Follow URDF naming conventions and best practices

---

## Prerequisites

- Completed Chapter 1 (ROS 2 Architecture)
- Basic understanding of XML syntax
- Familiarity with 3D coordinate systems (X, Y, Z axes)
- RViz installed (included in ros-humble-desktop)

---

## Section Breakdown

### 1. index.mdx - Chapter Introduction (5 min)
- What is URDF? (robot description language)
- Why URDF matters (simulation, visualization, motion planning)
- Chapter roadmap

### 2. urdf-syntax.mdx - URDF XML Structure (20 min)
- XML basics review
- `<robot>` root element with name attribute
- `<link>` elements (rigid bodies)
- `<joint>` elements (connections between links)
- `<visual>`, `<collision>`, `<inertial>` sub-elements
- Minimal URDF example (2 links, 1 joint)
- Required vs optional attributes

### 3. frames-transforms.mdx - Coordinate Frames and Transformations (25 min)
- What are coordinate frames? (origin + orientation for each link)
- Parent-child relationships in kinematic tree
- Transform specification: `<origin xyz="x y z" rpy="roll pitch yaw"/>`
- Right-hand rule convention
- TF (transform) tree in ROS 2
- Visualizing frames with `ros2 run tf2_tools view_frames`

### 4. links-properties.mdx - Link Definitions (25 min)
- Link structure: `<link name="...">`
- **Visual properties**: Appearance for visualization
  - Geometry: box, cylinder, sphere, mesh
  - Material: color and texture
  - Example: `<geometry><box size="0.1 0.1 0.1"/></geometry>`
- **Collision properties**: Simplified geometry for collision detection
- **Inertial properties**: Mass and inertia tensor for physics simulation
  - Mass: `<mass value="1.0"/>`
  - Inertia: `<inertia ixx="0.01" ixy="0" ... izz="0.01"/>`
- When to use simplified collision geometry
- Calculating inertia (basic shapes vs tools)

### 5. joints-types.mdx - Joint Types and Constraints (25 min)
- Joint types:
  - **Fixed**: No motion (e.g., mounting plate)
  - **Revolute**: Rotation around axis with limits (e.g., elbow)
  - **Continuous**: Unlimited rotation (e.g., wheel)
  - **Prismatic**: Linear translation (e.g., telescoping arm)
- Joint structure: `<joint name="..." type="...">`
- Parent/child link specification
- Joint axis: `<axis xyz="0 0 1"/>` (rotation/translation direction)
- Joint limits: `<limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>`
- Joint dynamics: `<dynamics damping="0.1" friction="0.1"/>`
- Humanoid joint examples (shoulder, elbow, neck)

### 6. humanoid-example.mdx - Complete Humanoid URDF (40 min)
- **Code Example 1: humanoid_urdf**
- Simplified humanoid structure:
  - `base_link` (root, usually pelvis or ground contact)
  - `torso_link`
  - `head_link` (revolute joint for neck)
  - `left_shoulder_link`, `left_upper_arm_link`, `left_lower_arm_link`, `left_hand_link`
  - `right_shoulder_link`, `right_upper_arm_link`, `right_lower_arm_link`, `right_hand_link`
  - Total: 11 links, 10 joints
- Joint specifications:
  - Neck: 1 DOF yaw (-π/2 to π/2)
  - Shoulders: 2 DOF per side (pitch: -π to π, roll: -π/2 to π/2)
  - Elbows: 1 DOF per side (pitch: 0 to π)
  - Wrists: 1 DOF per side (yaw: -π/2 to π/2)
- Complete, downloadable URDF file
- File structure and organization
- Naming conventions (left/right, upper/lower)

### 7. rviz-viz.mdx - Visualizing in RViz (30 min)
- **Code Example 2: display.launch.py**
- Robot state publisher node (publishes robot description)
- Joint state publisher GUI (controls joint angles)
- RViz configuration:
  - Add RobotModel display
  - Set Fixed Frame to `base_link`
  - Enable TF display
- Launch file structure for visualization
- Running: `ros2 launch humanoid_urdf display.launch.py`
- Interacting with joint sliders
- Troubleshooting visualization issues

### 8. sensors-urdf.mdx - Adding Sensors to URDF (20 min)
- Sensor attachment to links
- Camera on head example:
  ```xml
  <link name="camera_link">
    <visual>...</visual>
    <collision>...</collision>
  </link>
  <joint name="head_to_camera" type="fixed">
    <parent link="head_link"/>
    <child link="camera_link"/>
    <origin xyz="0.1 0 0.05" rpy="0 0 0"/>
  </joint>
  ```
- IMU on torso example
- Sensor frame conventions (camera: X forward, Y left, Z up)
- Gazebo sensor plugins (brief mention, detailed in later modules)

### 9. validation.mdx - URDF Validation and Best Practices (15 min)
- `check_urdf` command:
  ```bash
  check_urdf humanoid.urdf
  # Expected: robot name is: humanoid
  #           ---------- Successfully Parsed XML ---------------
  ```
- Common URDF errors:
  - Missing required attributes
  - Circular dependencies in joints
  - Invalid XML syntax
  - Joint limits reversed (lower > upper)
- `urdf_to_graphviz` for visualization:
  ```bash
  urdf_to_graphviz humanoid.urdf
  # Generates humanoid.pdf with link-joint tree
  ```
- Best practices:
  - Use consistent naming (snake_case)
  - Keep collision geometry simple
  - Use realistic inertia values
  - Comment complex sections
  - Modularize with xacro (mention, not covered in detail)

---

## Code Examples

### 1. humanoid_urdf Package
- **File**: `urdf/humanoid.urdf`
- **Description**: Complete 11-link humanoid URDF
- **Links**: base, torso, head, 4 links per arm (shoulder, upper, lower, hand)
- **Joints**: 10 revolute joints with realistic limits
- **Properties**: Visual (boxes for simplicity), collision (same as visual), inertial (approximated)

### 2. display.launch.py
- **File**: `launch/display.launch.py`
- **Nodes**:
  - `robot_state_publisher`: Publishes URDF to `/robot_description`
  - `joint_state_publisher_gui`: GUI for controlling joint angles
  - `rviz2`: Visualization with pre-configured settings
- **Demonstrates**: Complete visualization workflow

---

## Diagrams

1. **tf-tree.svg**: Example TF tree for humanoid (root to leaves)
2. **link-hierarchy.svg**: Link parent-child structure diagram
3. **humanoid-structure.svg**: Visual representation of 11-link humanoid
4. **sensor-placement.svg**: Camera and IMU attachment locations

---

## Success Criteria

✅ Can explain URDF structure (robot, link, joint)
✅ Can define links with visual/collision/inertial properties
✅ Can define revolute joints with axis and limits
✅ Can create a simplified humanoid URDF (8+ links)
✅ Can visualize URDF in RViz and control joints
✅ Can add sensors to robot description
✅ Can validate URDF with `check_urdf`
✅ Understands coordinate frames and transformations

---

## Fulfills Requirements

FR-025 through FR-033

---

**Contract Status**: Complete ✅
