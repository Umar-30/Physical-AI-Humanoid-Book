# Research Findings: Module 2: The Digital Twin Content

## Decision: Gazebo Version for Instructions

**Decision**: The instructional content for Module 2 will standardize on **Ignition Gazebo Fortress**.

**Rationale**:
Ignition Gazebo Fortress is the default and officially supported Gazebo version for ROS 2 Humble on Ubuntu 22.04. This choice ensures the most seamless integration with the specified ROS 2 distribution and operating system, providing students with the most straightforward and stable learning experience. Focusing on the default integration minimizes potential setup complexities and troubleshooting unique to non-standard configurations.

**Alternatives Considered**:

-   **Ignition Gazebo Harmonic**: While newer, using Gazebo Harmonic with ROS 2 Humble might introduce additional setup complexities or require specific bridge packages not part of the standard Humble installation. This could unnecessarily complicate the learning path for students new to robotics simulation.
-   **Gazebo Classic 11**: This is an older generation of Gazebo. While still compatible with ROS 2 (via `ros_gz_bridge`), Ignition Gazebo is its modern successor, actively developed, and offers more advanced features. Standardizing on Classic would not align with teaching current best practices and forward-looking robotics simulation.
