# Chapter 7: Unity Rendering and Human-Robot Interaction

**Priority**: P3 (Requires Chapters 5-6 completed)
**Estimated Duration**: 3-4 hours
**Target Audience**: Readers with functional Gazebo simulation

## Learning Objectives

1. **Install** Unity 2022 LTS and Unity Robotics Hub packages
2. **Configure** ROS-TCP-Connector for Gazebo-Unity communication
3. **Create** Unity scenes for robot visualization with lighting and materials
4. **Integrate** human avatars for HRI scenarios
5. **Build** UI overlays displaying sensor data in real-time

## Chapter Structure

### 7.1 Unity Installation and Setup (30 min)
- Unity Hub and Unity 2022 LTS installation
- Unity Robotics Hub package installation via Package Manager
- URDF Importer package setup
- First project creation and testing

### 7.2 ROS 2 Integration Setup (35 min)
- ros_tcp_endpoint installation (ROS 2 side)
- ROS-TCP-Connector configuration (Unity side)
- Connection testing and verification
- Topic subscription demonstration

### 7.3 Unity Scene Creation for Robot Visualization (40 min)
- URDF import into Unity
- Scene hierarchy organization
- Lighting setup (directional, point, ambient)
- Material configuration for robot visualization
- Camera placement and control

### 7.4 Human Avatar Integration for HRI (35 min)
- Mixamo character import
- Animation controller setup
- Proximity detection with colliders
- Human-robot interaction scenarios

### 7.5 UI Overlays for Sensor Data (30 min)
- Canvas and Text elements for IMU data
- RawImage for camera feed display
- Real-time data updates via ROS messages
- Layout and styling

### 7.6 Hands-On Exercises (45 min)
- Exercise 7.1: Create Unity scene with imported robot
- Exercise 7.2: Display live camera feed from Gazebo
- Exercise 7.3: Add human avatar with proximity trigger

## Key Configuration Files

1. `RobotController.cs` - Unity C# script for joint synchronization
2. `SensorDisplay.cs` - Unity C# script for UI updates
3. `HumanController.cs` - Unity C# script for avatar control

## Validation Checklist

- [ ] Unity successfully imports URDF model
- [ ] ROS connection established (no errors in console)
- [ ] Robot pose syncs with Gazebo (<100ms latency)
- [ ] Sensor data displays in UI
- [ ] Human avatar animates correctly
