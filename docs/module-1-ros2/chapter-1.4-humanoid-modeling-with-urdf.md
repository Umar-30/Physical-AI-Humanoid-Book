---
id: chapter-1-4-humanoid-modeling-with-urdf
title: Humanoid Modeling with URDF
sidebar_position: 4
---

# Chapter 1.4: Humanoid Modeling with URDF

# Chapter 1.4: Humanoid Modeling with URDF

## Focus: URDF syntax, joint/link definitions, kinematics basics
## Learning objectives: Create a simple humanoid URDF model

The Unified Robot Description Format (URDF) is an XML-based file format used in ROS to describe all aspects of a robot. It's crucial for visualizing robots, performing simulations, and planning motion. For humanoid robots, URDF allows us to define their complex articulated structures, enabling both simulation and real-world control.

### What is URDF?
URDF provides a standardized way to represent the kinematic and dynamic properties of a robot. It defines:
*   **Links:** The rigid bodies of the robot (e.g., torso, upper arm, forearm, hand).
*   **Joints:** The connections between these links, defining their type (e.g., revolute, prismatic, fixed) and motion constraints.
*   **Sensors, Actuators, and other elements:** Although core URDF focuses on kinematics, extensions and additional XML elements can describe other components.

**Why URDF for Humanoids?**
Humanoid robots are highly articulated, with many degrees of freedom. URDF allows us to:
*   **Visualize:** See the robot model in tools like RViz.
*   **Simulate:** Use the robot model in physics engines like Gazebo.
*   **Kinematics:** Perform forward and inverse kinematics calculations.
*   **Collision Detection:** Define collision properties for safe interaction.

### Basic Structure of a URDF File
A URDF file is a single XML file, typically with a `.urdf` extension, consisting of a `<robot>` tag as its root. Inside, it contains multiple `<link>` and `<joint>` tags.

```xml
<?xml version="1.0"?>
<robot name="my_humanoid_robot">
  <!-- Define links and joints here -->
  <link name="base_link">
    <!-- Visual, Inertial, Collision properties -->
  </link>

  <joint name="joint_name" type="revolute">
    <!-- Parent and child links, axis, limits, origin -->
  </joint>

  <!-- More links and joints -->

</robot>
```

### Defining Links
A `<link>` element describes a rigid body of the robot. Key properties include:

*   **`name`**: A unique identifier for the link.
*   **`<visual>`**: Describes how the link looks (geometry, material, origin).
    *   `<geometry>`: Shape (box, cylinder, sphere, mesh).
    *   `<material>`: Color.
*   **`<inertial>`**: Describes the link's mass properties (mass, inertia matrix, origin). Essential for physics simulation.
*   **`<collision>`**: Describes the link's collision properties (geometry, origin). Used for collision detection.

**Example Link Definition:**
```xml
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.4"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>
```

### Defining Joints
A `<joint>` element describes the connection and relative motion between two links.

*   **`name`**: A unique identifier for the joint.
*   **`type`**: Specifies the joint type, controlling its degrees of freedom:
    *   `revolute`: Rotational joint with a limited range (e.g., elbow, knee).
    *   `continuous`: Rotational joint with unlimited range (e.g., wheel).
    *   `prismatic`: Linear joint with a limited range (e.g., linear actuator).
    *   `fixed`: No motion between links (e.g., base of a robot).
    *   `floating`: All 6 degrees of freedom, used for the base of a free-flying or wheeled robot.
    *   `planar`: Two linear and one rotational degree of freedom.
*   **`<parent>` and `<child>`**: Specify the `name` of the parent and child links connected by this joint.
*   **`<origin>`**: Defines the transform from the parent link's origin to the joint's origin. It includes `xyz` (position) and `rpy` (roll, pitch, yaw rotation).
*   **`<axis>`**: For revolute and prismatic joints, specifies the axis of rotation or translation.
*   **`<limit>`**: For revolute and prismatic joints, defines the `lower` and `upper` bounds, `effort`, and `velocity` limits.

**Example Joint Definition:**
```xml
  <joint name="torso_to_left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.0 0.1 0.2" rpy="0 0 0"/> <!-- Offset from torso to shoulder -->
    <axis xyz="1 0 0"/> <!-- Rotation around X-axis -->
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>
```

### Simple Humanoid Segment Example (Two-Link Arm)

Let's put it together to model a simple two-link arm, which is a fundamental component of a humanoid robot.

```xml
<?xml version="1.0"?>
<robot name="simple_two_link_arm">

  <!-- Link: Base Link (e.g., part of torso) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="grey">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Link: Upper Arm -->
  <link name="upper_arm">
    <visual>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.4"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint: Base to Upper Arm (Shoulder Joint) -->
  <joint name="shoulder_yaw_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/> <!-- Offset from base_link to joint origin -->
    <axis xyz="0 0 1"/> <!-- Rotates around Z-axis -->
    <limit lower="-1.57" upper="1.57" effort="10" velocity="0.5"/>
  </joint>

  <!-- Link: Forearm -->
  <link name="forearm">
    <visual>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.025" length="0.3"/>
      </geometry>
      <material name="green">
        <color rgba="0 0.8 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.025" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- Joint: Upper Arm to Forearm (Elbow Joint) -->
  <joint name="elbow_pitch_joint" type="revolute">
    <parent link="upper_arm"/>
    <child link="forearm"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/> <!-- End of upper_arm, start of forearm -->
    <axis xyz="0 1 0"/> <!-- Rotates around Y-axis -->
    <limit lower="0" upper="2.5" effort="10" velocity="0.5"/>
  </joint>

</robot>
```

This example demonstrates how links and joints are defined to create a kinematic chain. For a full humanoid, this structure would be expanded to include legs, a head, hands, and many more joints, creating a highly complex and articulated model suitable for advanced robotics applications.
