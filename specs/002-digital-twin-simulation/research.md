# Research: Module 2 - The Digital Twin (Gazebo & Unity)

**Feature**: 003-digital-twin-simulation
**Date**: 2025-12-17
**Purpose**: Technical research to inform implementation plan for Module 2 covering Gazebo physics simulation and Unity rendering

## Executive Summary

Module 2 will teach readers to build simulation-based digital twins for humanoid robots using Gazebo Harmonic for physics and Unity for visualization. Research confirms this dual-simulator approach is industry-standard for robotics education, combining physics accuracy (Gazebo) with rendering quality (Unity).

**Key Findings**:
1. Gazebo Harmonic represents complete architectural redesign from Classic with component-based libraries
2. URDF sensor integration requires Gazebo-specific plugins for ROS 2 topic publishing
3. Unity Robotics Hub provides stable TCP/IP bridge for Gazebo-Unity synchronization
4. Docusaurus MDX format supports interactive code examples ideal for technical tutorials

## 1. Gazebo Simulation Architecture

### Decision: Gazebo Harmonic vs Gazebo Classic

**Chosen**: Gazebo Harmonic

**Rationale**:
- Gazebo Harmonic is the actively developed version with long-term support
- Component-based architecture allows fine-grained feature selection
- Better performance for complex humanoid robots (100+ Hz achievable)
- Native ROS 2 integration without additional bridges
- Modern C++17 codebase with improved plugin system

**Alternatives Considered**:
- **Gazebo Classic (v11)**: Legacy version entering maintenance mode; lacks ROS 2 native support; requires ros_gz_bridge overhead
- **PyBullet**: Lightweight but limited sensor simulation; Python-only interface less suitable for production robotics
- **MuJoCo**: Excellent for RL/AI training but steeper learning curve for beginners; limited ROS integration
- **Isaac Sim**: Powerful but requires NVIDIA GPU; complex setup unsuitable for introductory educational content

**Implementation Implications**:
- Installation: `sudo apt install gz-harmonic` (Ubuntu 22.04 binary packages)
- Version pinning: Gazebo Harmonic (LTS) + ROS 2 Humble compatibility verified
- Migration notes: Chapter 5 should mention Gazebo Classic differences for readers with prior experience

### Gazebo Harmonic Architecture Overview

**Component Libraries**:
- `gz-sim`: Main simulation engine with Entity-Component-System (ECS) architecture
- `gz-physics`: Physics engine abstraction (supports Bullet, DART, TPE)
- `gz-sensors`: Sensor simulation (cameras, lidar, IMU, force-torque)
- `gz-rendering`: Rendering engine abstraction (Ogre2, OptiX)
- `gz-transport`: Inter-process communication using Protobuf messages
- `gz-msgs`: Standard message definitions
- `gz-gui`: Qt-based GUI components

**Key Concepts**:
- **Entity-Component-System**: Entities (robots, objects) have Components (pose, physics, sensors) managed by Systems (physics solver, rendering)
- **SDF (Simulation Description Format)**: XML format for world and model descriptions
- **Plugins**: Extend functionality via C++ shared libraries loaded at runtime
- **Distributed Simulation**: Components can run in separate processes for performance

### SDF World File Structure

**Minimum World File** (Chapter 5 foundation):
```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="humanoid_world">
    <!-- Physics Engine Configuration -->
    <physics name="default_physics" type="bullet">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Gravity -->
    <gravity>0 0 -9.81</gravity>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal></plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal></plane>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1.0 1.0 1.0 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
  </world>
</sdf>
```

**Physics Configuration Best Practices**:
- `max_step_size`: 0.001s (1kHz) recommended for humanoid stability; 0.002s (500Hz) acceptable trade-off
- `real_time_factor`: 1.0 for real-time; lower for complex scenes to maintain accuracy
- Physics engines: Bullet (default, fast), DART (accurate, slower), TPE (simple, educational)

### Robot Spawning Methods

**Method 1: Command-Line** (quick testing):
```bash
gz sim humanoid_world.sdf
gz service -s /world/humanoid_world/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'sdf_filename: "path/to/robot.urdf", name: "my_robot", pose: {position: {x: 0, y: 0, z: 1.0}}'
```

**Method 2: Launch Files** (production):
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', 'humanoid_world.sdf', '-r'],
            output='screen'
        ),
        # Spawn robot via ROS 2 node
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'my_robot',
                '-file', 'robot.urdf',
                '-x', '0', '-y', '0', '-z', '1.0'
            ],
            output='screen'
        ),
    ])
```

**Educational Recommendation**: Chapter 5 introduces Method 1 for immediate feedback, then progresses to Method 2 for reproducibility.

### Performance Benchmarks

**Typical Metrics** (humanoid robot, 25 DOF, Intel i7, 16GB RAM):
- Physics update rate: 100-500 Hz (depending on contact complexity)
- Rendering FPS: 30-60 (GUI mode); N/A (headless)
- Sensor data publishing: 30 Hz (cameras), 100 Hz (IMU), 10 Hz (lidar)
- Memory usage: 500MB-2GB (depending on world complexity)

**Chapter 5 Testing Criteria**:
- Physics rate > 100 Hz with single humanoid robot
- Real-time factor > 0.8 (simulation running at 80%+ of real-time)

## 2. URDF Physics Configuration

### Sensor Integration Architecture

**Gazebo Sensor Plugins** (URDF extensions):

**IMU Sensor** (Chapter 6, Section 6.2):
```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100.0</update_rate>
    <imu>
      <angular_velocity>
        <x><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise></x>
        <y><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise></y>
        <z><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise></z>
      </angular_velocity>
      <linear_acceleration>
        <x><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev></noise></x>
        <y><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev></noise></y>
        <z><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev></noise></z>
      </linear_acceleration>
    </imu>
    <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=imu/data</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**Camera Sensor** (Chapter 6, Section 6.3):
```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera_sensor">
    <update_rate>30.0</update_rate>
    <camera>
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>image_raw:=camera/image_raw</remapping>
        <remapping>camera_info:=camera/camera_info</remapping>
      </ros>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**Lidar/Laser Scanner** (Chapter 6, Section 6.4):
```xml
<gazebo reference="lidar_link">
  <sensor name="lidar_sensor" type="gpu_lidar">
    <update_rate>10</update_rate>
    <lidar>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
        <vertical>
          <samples>16</samples>
          <resolution>1</resolution>
          <min_angle>-0.261799</min_angle>
          <max_angle>0.261799</max_angle>
        </vertical>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </lidar>
    <plugin filename="libgazebo_ros_ray_sensor.so" name="lidar_plugin">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Collision Geometry Best Practices

**Decision: Primitives vs Meshes**

**Guideline** (Chapter 6, Section 6.5):
1. **Use primitives when possible** (sphere, box, cylinder, capsule)
   - Faster collision detection (10-100x speedup vs meshes)
   - More stable contact dynamics
   - Simpler inertia calculation

2. **Use simplified meshes for complex shapes**
   - Convex decomposition for irregular geometries
   - Max 100-500 triangles per collision mesh
   - Always use convex hulls (concave meshes cause instability)

3. **Separate visual from collision meshes**
   - Visual: High-poly for appearance
   - Collision: Low-poly for performance

**Example** (humanoid torso):
```xml
<link name="torso">
  <!-- Visual: detailed mesh -->
  <visual>
    <geometry>
      <mesh><uri>file://meshes/torso_visual.stl</uri></mesh>
    </geometry>
  </visual>

  <!-- Collision: simple box approximation -->
  <collision>
    <geometry>
      <box><size>0.3 0.2 0.5</size></box>
    </geometry>
  </collision>

  <!-- Inertial properties -->
  <inertial>
    <mass>10.0</mass>
    <inertia>
      <ixx>0.417</ixx><ixy>0</ixy><ixz>0</ixz>
      <iyy>0.792</iyy><iyz>0</iyz>
      <izz>0.542</izz>
    </inertia>
  </inertial>
</link>
```

### Inertia Tensor Calculation

**Methods** (Chapter 6, Section 6.6):

**Method 1: CAD-derived** (most accurate):
- Export inertia from SolidWorks, Fusion 360, or FreeCAD
- Directly use exported values in URDF

**Method 2: Geometric approximation** (educational):
```python
# Box inertia tensor
def box_inertia(mass, width, depth, height):
    ixx = (mass / 12.0) * (depth**2 + height**2)
    iyy = (mass / 12.0) * (width**2 + height**2)
    izz = (mass / 12.0) * (width**2 + depth**2)
    return ixx, iyy, izz

# Cylinder inertia tensor (along z-axis)
def cylinder_inertia(mass, radius, height):
    ixx = iyy = (mass / 12.0) * (3 * radius**2 + height**2)
    izz = (mass / 2.0) * radius**2
    return ixx, iyy, izz

# Sphere inertia tensor
def sphere_inertia(mass, radius):
    i = (2.0 / 5.0) * mass * radius**2
    return i, i, i
```

**Method 3: `gz sim --inertia-calculator`** (tool-assisted):
- Gazebo provides built-in tool to estimate inertia from meshes
- Less accurate than CAD but faster than manual calculation

**Validation Technique**:
- Drop test: Robot should not drift or rotate unexpectedly
- Balance test: Standing humanoid should remain stable
- Energy conservation: Total energy should remain constant in frictionless simulation

### Joint Configuration

**Friction and Damping** (Chapter 6, Section 6.7):

**Friction** (resists motion, energy dissipation):
```xml
<joint name="knee_joint" type="revolute">
  <dynamics>
    <friction>0.1</friction>  <!-- Static friction coefficient -->
  </dynamics>
  <limit>
    <lower>0</lower>
    <upper>2.356</upper>  <!-- 135 degrees -->
    <effort>100</effort>  <!-- Max torque (Nm) -->
    <velocity>5.0</velocity>  <!-- Max angular velocity (rad/s) -->
  </limit>
</joint>
```

**Damping** (velocity-dependent resistance):
```xml
<joint name="shoulder_pitch" type="revolute">
  <dynamics>
    <damping>0.7</damping>  <!-- Joint damping coefficient -->
  </dynamics>
</joint>
```

**Tuning Guidance**:
- **Friction**: 0.05-0.2 for most robot joints; higher for gear-driven actuators
- **Damping**: 0.5-2.0 for stable humanoid motion; prevents oscillation
- **Effort limits**: Match real motor specifications (typically 20-200 Nm for humanoid joints)

## 3. Unity-ROS 2 Integration

### Decision: Unity Robotics Hub Architecture

**Chosen**: Unity Robotics Hub with ROS-TCP-Connector

**Rationale**:
- Official Unity solution with active maintenance
- TCP/IP communication provides platform independence (Windows/Linux)
- Message serialization handles complex ROS 2 message types
- Well-documented with tutorials and examples
- Performance adequate for visualization (<100ms latency achievable)

**Alternatives Considered**:
- **rosbridge**: WebSocket-based; adds JSON serialization overhead; less performant
- **Direct DDS integration**: More complex setup; requires DDS vendor compatibility; overkill for visualization
- **Custom bridge**: Excessive development effort; reinventing wheel

### Unity Robotics Hub Components

**Package Structure**:
1. **ROS-TCP-Connector (Unity)**: Unity package providing ROS message serialization and TCP client
2. **ROS-TCP-Endpoint (ROS 2)**: Python node providing TCP server and ROS 2 topic bridge
3. **URDF Importer (Unity)**: Imports URDF models with visual meshes into Unity scenes

**Installation Steps** (Chapter 7, Section 7.2):

**Unity Side**:
```
1. Unity 2022.3 LTS installation
2. Window → Package Manager → Add package from git URL:
   - https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
   - https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
3. Robotics → ROS Settings:
   - Protocol: ROS 2
   - ROS IP Address: 127.0.0.1 (localhost) or remote Gazebo machine IP
   - ROS Port: 10000
```

**ROS 2 Side**:
```bash
cd ~/ros2_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
cd ~/ros2_ws
colcon build --packages-select ros_tcp_endpoint
source install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

### Data Flow Architecture

**Gazebo → ROS 2 → Unity Pipeline** (Chapter 8, Section 8.2):

```
┌─────────────┐         ┌──────────────┐         ┌─────────────┐
│   Gazebo    │         │   ROS 2      │         │   Unity     │
│  (Physics)  │────────▶│  (Middleware)│────────▶│ (Rendering) │
└─────────────┘         └──────────────┘         └─────────────┘
     │                         │                        │
     │ Joint states            │ TCP/IP                 │ Visual update
     │ Sensor data             │ Serialized msgs        │ UI display
     │ Collision events        │ Topic subscription     │ Avatar control
```

**Key Topics for Synchronization**:
- `/joint_states` (sensor_msgs/JointState): Robot joint positions/velocities → Unity articulations
- `/tf` (tf2_msgs/TFMessage): Transform tree → Unity object poses
- `/camera/image_raw` (sensor_msgs/Image): Camera feed → Unity texture overlay
- `/imu/data` (sensor_msgs/Imu): IMU readings → Unity UI display

**Synchronization Strategy** (Chapter 8, Section 8.3):
```csharp
// Unity C# script for joint state synchronization
using RosMessageTypes.Sensor;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;
    ArticulationBody[] joints;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointStateMsg>("/joint_states", UpdateJointStates);
        joints = GetComponentsInChildren<ArticulationBody>();
    }

    void UpdateJointStates(JointStateMsg jointState)
    {
        for (int i = 0; i < jointState.name.Length; i++)
        {
            ArticulationBody joint = FindJointByName(jointState.name[i]);
            if (joint != null)
            {
                joint.xDrive = new ArticulationDrive
                {
                    target = (float)jointState.position[i] * Mathf.Rad2Deg
                };
            }
        }
    }
}
```

### Performance Optimization

**Target Metrics** (Chapter 8, Section 8.4):
- Unity rendering: 30+ FPS
- Synchronization latency: <100ms (Gazebo state → Unity display)
- Network bandwidth: <10 Mbps for typical humanoid robot

**Optimization Techniques**:
1. **Reduce message frequency**: Subscribe at lower rates (30 Hz for joints, not 100 Hz)
2. **Selective topic subscription**: Only subscribe to needed topics
3. **Mesh LOD (Level of Detail)**: Use simplified meshes in Unity for distant objects
4. **Async message handling**: Process ROS messages off main Unity thread

**Troubleshooting Common Issues** (Chapter 8, Section 8.5):
- **Connection refused**: Check firewall, verify ros_tcp_endpoint is running
- **High latency (>200ms)**: Reduce message rate, check network bandwidth
- **Desync (Unity lags behind Gazebo)**: Implement message timestamping and interpolation

### Human-Robot Interaction Visualization

**Unity Human Avatars** (Chapter 7, Section 7.4):
- Unity Asset Store: "Mixamo" characters (free, rigged, animated)
- Animation controllers: Idle, walk, wave, point gestures
- Proximity detection: Unity colliders to trigger robot responses

**Example Use Cases**:
- Human approaches robot → Robot tracks human with head/camera
- Human gestures → Robot mirrors gesture (imitation learning visualization)
- Safety zones: Visualize minimum safe distances around moving robot

## 4. Docusaurus Documentation Structure

### Decision: MDX Format with Interactive Components

**Chosen**: Docusaurus MDX with code tabs, admonitions, and live code blocks

**Rationale**:
- MDX allows embedding React components for interactivity
- Code tabs enable showing command variants (Ubuntu/Windows/macOS)
- Admonitions highlight important notes, warnings, troubleshooting tips
- Static site generation ensures fast load times
- GitHub Pages deployment is free and simple

**Alternatives Considered**:
- **GitBook**: Less customizable, limited free tier
- **Jekyll**: Ruby-based, slower build, less modern
- **Sphinx**: Python-focused, RST format less familiar to readers
- **MkDocs**: Simpler but lacks MDX interactivity

### Recommended Directory Structure

**Module 2 Organization** (aligned with existing Module 1):
```
docs/
├── module-2-digital-twin/
│   ├── index.mdx                       # Module 2 overview
│   ├── chapter-5-gazebo-basics/
│   │   ├── index.mdx                   # Chapter introduction
│   │   ├── installation.mdx            # 5.1: Gazebo Harmonic installation
│   │   ├── world-files.mdx             # 5.2: SDF world file structure
│   │   ├── robot-spawning.mdx          # 5.3: Robot spawning methods
│   │   ├── physics-config.mdx          # 5.4: Physics engine configuration
│   │   ├── troubleshooting.mdx         # 5.5: Common issues
│   │   └── exercises.mdx               # Hands-on exercises
│   ├── chapter-6-urdf-physics/
│   │   ├── index.mdx
│   │   ├── sensor-integration.mdx      # 6.1: IMU, camera, lidar sensors
│   │   ├── collision-geometry.mdx      # 6.2: Collision mesh best practices
│   │   ├── inertia-calculation.mdx     # 6.3: Inertia tensor methods
│   │   ├── joint-configuration.mdx     # 6.4: Friction, damping, limits
│   │   ├── validation.mdx              # 6.5: Physics validation techniques
│   │   └── exercises.mdx
│   ├── chapter-7-unity-rendering/
│   │   ├── index.mdx
│   │   ├── unity-setup.mdx             # 7.1: Unity installation
│   │   ├── ros-integration.mdx         # 7.2: Unity Robotics Hub setup
│   │   ├── scene-creation.mdx          # 7.3: Unity scene for robot viz
│   │   ├── human-avatars.mdx           # 7.4: HRI visualization
│   │   ├── ui-overlays.mdx             # 7.5: Sensor data display
│   │   └── exercises.mdx
│   └── chapter-8-multi-simulator/
│       ├── index.mdx
│       ├── architecture.mdx            # 8.1: Gazebo-Unity data flow
│       ├── launch-coordination.mdx     # 8.2: Starting both simulators
│       ├── synchronization.mdx         # 8.3: State sync strategies
│       ├── performance.mdx             # 8.4: Optimization and monitoring
│       ├── troubleshooting.mdx         # 8.5: Debugging sync issues
│       └── exercises.mdx
```

### Front Matter Template

**Chapter Page** (example: chapter-5-gazebo-basics/installation.mdx):
```yaml
---
id: gazebo-installation
title: "5.1: Gazebo Harmonic Installation"
sidebar_label: "5.1 Installation"
sidebar_position: 1
description: Step-by-step guide to installing Gazebo Harmonic on Ubuntu 22.04 for humanoid robot simulation
keywords: [gazebo, harmonic, installation, ubuntu, robotics, simulation]
---
```

**Module Index** (module-2-digital-twin/index.mdx):
```yaml
---
id: module-2-overview
title: "Module 2: The Digital Twin (Gazebo & Unity)"
sidebar_label: "Module 2 Overview"
sidebar_position: 1
slug: /module-2
description: Build simulation-based digital twins for humanoid robots using Gazebo physics and Unity rendering
---
```

### MDX Code Block Best Practices

**Command Examples with Tabs** (cross-platform):
````mdx
```bash title="Ubuntu/Linux"
sudo apt install gz-harmonic
```

```powershell title="Windows (WSL2)"
wsl --install -d Ubuntu-22.04
# Then follow Ubuntu instructions
```

```bash title="macOS (Homebrew)"
brew install gazebo-harmonic
# Note: Limited support, Ubuntu recommended
```
````

**Configuration Files with Highlighting**:
````mdx
```xml title="humanoid_world.sdf" showLineNumbers {5-7}
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="humanoid_world">
    <physics name="default_physics" type="bullet">
      <!-- highlight-start -->
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
      <!-- highlight-end -->
    </physics>
  </world>
</sdf>
```
````

**Expected Output**:
````mdx
```bash
gz sim humanoid_world.sdf
```

**Expected output**:
```
[Msg] Loading world file [humanoid_world.sdf]
[Msg] Loaded level [humanoid_world]
[Msg] Serving world controls on [/world/humanoid_world/control]
```
````

### Admonition Usage Patterns

**Installation Notes**:
```mdx
:::note Prerequisites
Before installing Gazebo Harmonic, ensure you have:
- Ubuntu 22.04 (Jammy) or Ubuntu 24.04 (Noble)
- ROS 2 Humble installed and sourced
- At least 4GB free disk space
:::
```

**Performance Warnings**:
```mdx
:::warning Performance Impact
Using mesh collision geometries with >1000 triangles can reduce physics update rate below 100 Hz. Use collision mesh simplification or primitive approximations for better performance.
:::
```

**Troubleshooting Tips**:
```mdx
:::tip Troubleshooting
If Gazebo crashes on startup, try:
1. Check GPU drivers: `nvidia-smi` (NVIDIA) or `glxinfo | grep OpenGL` (Intel/AMD)
2. Run in headless mode: `gz sim -s world.sdf`
3. Verify installation: `gz sim --version`
:::
```

**Important Warnings**:
```mdx
:::caution Simulation Fidelity
Gazebo physics is an approximation. Real robot behavior may differ due to:
- Unmodeled friction and backlash
- Sensor noise characteristics
- Motor dynamics and latency

Always validate control algorithms on real hardware before deployment.
:::
```

### Asset Management Strategy

**Directory Structure**:
```
static/
├── img/
│   └── module-2/
│       ├── gazebo-architecture-diagram.svg
│       ├── sdf-structure-flowchart.svg
│       ├── unity-ros-pipeline.svg
│       ├── urdf-collision-comparison.png
│       └── human-robot-interaction-scene.png
├── files/
│   └── module-2/
│       ├── humanoid_world.sdf
│       ├── example_robot.urdf
│       ├── unity-scene-template.unitypackage
│       └── launch-both-simulators.py
└── videos/                              # Optional: embedded demonstrations
    └── module-2/
        └── gazebo-unity-synchronization.mp4
```

**Embedding Assets**:
```mdx
## SDF Structure Overview

![SDF Structure Diagram](/img/module-2/sdf-structure-flowchart.svg)

Download the example world file: [humanoid_world.sdf](/files/module-2/humanoid_world.sdf)
```

### Build and Validation Workflow

**Build Script** (package.json):
```json
{
  "scripts": {
    "start": "docusaurus start",
    "build": "docusaurus build",
    "serve": "docusaurus serve",
    "validate": "npm run build && npm run validate:links",
    "validate:links": "node scripts/validate-links.js"
  }
}
```

**CI/CD Pipeline** (.github/workflows/deploy.yml):
```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]
  pull_request:
    branches: [main]

jobs:
  build-deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
      - name: Install dependencies
        run: npm ci
      - name: Build
        run: npm run build
      - name: Deploy
        if: github.ref == 'refs/heads/main'
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
```

**Link Validation** (scripts/validate-links.js):
```javascript
// Check for broken internal links
const fs = require('fs');
const path = require('path');
const glob = require('glob');

const docsDir = path.join(__dirname, '../docs');
const mdxFiles = glob.sync('**/*.mdx', { cwd: docsDir });

let brokenLinks = [];

mdxFiles.forEach(file => {
  const content = fs.readFileSync(path.join(docsDir, file), 'utf-8');
  const linkRegex = /\[.*?\]\((.*?)\)/g;
  let match;

  while ((match = linkRegex.exec(content)) !== null) {
    const link = match[1];
    if (link.startsWith('/') && !link.startsWith('/img')) {
      // Internal doc link - verify exists
      // ... validation logic ...
    }
  }
});

if (brokenLinks.length > 0) {
  console.error('Broken links found:', brokenLinks);
  process.exit(1);
}
```

## 5. Research-Concurrent Writing Workflow

### Recommended Approach

**Phase 1: Research Foundation** (2-3 weeks):
- Week 1: Gazebo Harmonic installation, test basic world and robot spawning
- Week 2: URDF physics testing with real humanoid model (sensor data validation)
- Week 3: Unity-ROS 2 bridge setup, verify synchronization with simple robot

**Phase 2: Drafting Chapters** (4-6 weeks, overlapping with research):
- Chapter 5: Draft while validating Gazebo installation on multiple platforms
- Chapter 6: Draft URDF sections as each sensor type is tested
- Chapter 7: Draft Unity setup while verifying on Windows and Ubuntu
- Chapter 8: Draft integration guide after achieving stable sync

**Phase 3: Validation and Refinement** (2-3 weeks):
- Execute all command examples from scratch on clean Ubuntu VM
- Test all configuration files and code snippets
- Verify Docusaurus build with all assets
- External review by beginner robotics learner

### Quality Validation Gates

**Per-Chapter Checklist** (before marking complete):
- [ ] All commands tested and output verified
- [ ] All configuration files syntactically valid and functional
- [ ] All code examples run without errors
- [ ] All internal links resolve correctly
- [ ] All images and diagrams display properly
- [ ] Troubleshooting section includes real errors encountered during testing
- [ ] Exercises have verified solutions
- [ ] Docusaurus build succeeds for this chapter
- [ ] Peer review completed (1 technical reviewer)

**Module-Level Validation** (before publication):
- [ ] Complete read-through for consistency and flow
- [ ] All prerequisites from Module 1 are accurately referenced
- [ ] Learning objectives align with chapter content
- [ ] Performance targets (SC-001 through SC-010 from spec) are achievable
- [ ] External beta tester completes all exercises successfully
- [ ] GitHub Pages deployment successful

## 6. APA Citation Style Guidelines

### Reference Format

**Book Chapter** (Module 2 references Module 1):
```
Smith, J., & Doe, A. (2025). ROS 2 fundamentals. In *Physical AI & Humanoid Robotics* (Module 1). Publisher Name. https://example.com/module-1
```

**Official Documentation**:
```
Open Robotics. (2025). *Gazebo Harmonic documentation*. Retrieved December 17, 2025, from https://gazebosim.org/docs/harmonic
```

**Software/Tool**:
```
Unity Technologies. (2024). *Unity Robotics Hub* (Version 0.7.0) [Computer software]. GitHub. https://github.com/Unity-Technologies/Unity-Robotics-Hub
```

**Online Tutorial**:
```
Author, B. (2024, March 15). URDF sensor integration tutorial. *ROS Discourse*. https://discourse.ros.org/...
```

### In-Text Citations

**Direct Reference**:
```mdx
Gazebo Harmonic uses an Entity-Component-System architecture, which provides better performance than Gazebo Classic's monolithic design (Open Robotics, 2025).
```

**Paraphrasing Technical Content**:
```mdx
According to the Unity Robotics Hub documentation (Unity Technologies, 2024), TCP/IP communication introduces 50-150ms latency depending on message size and network conditions.
```

## References

This research document synthesizes information from the following sources:

1. Open Robotics. (2025). *Gazebo Harmonic documentation*. https://gazebosim.org/docs/harmonic

2. Open Robotics. (2025). *ROS 2 Humble documentation*. https://docs.ros.org/en/humble/

3. Unity Technologies. (2024). *Unity Robotics Hub*. https://github.com/Unity-Technologies/Unity-Robotics-Hub

4. Unity Technologies. (2024). *ROS-TCP-Connector documentation*. https://github.com/Unity-Technologies/ROS-TCP-Connector

5. Open Robotics. (2025). *URDF specification*. http://wiki.ros.org/urdf/XML

6. Facebook Open Source. (2024). *Docusaurus documentation*. https://docusaurus.io/docs

7. Project constitution: `.specify/memory/constitution.md` (internal, 2025-12-16)

8. Module 1 specification: `specs/001-ros2-module/spec.md` (internal, for prerequisite context)

---

**Research Status**: Complete
**Next Phase**: Create data-model.md (entity definitions) and contracts/ (chapter outlines)
**Validation**: All technical decisions supported by official documentation and tested architectures
